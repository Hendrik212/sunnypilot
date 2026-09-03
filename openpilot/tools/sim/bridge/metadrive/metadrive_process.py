import math
import os
import time
import numpy as np

from collections import namedtuple
from multiprocessing.connection import Connection

# Use Panda3D's headless EGL display library (p3headlessgl) instead of the default pandagl
# (GLX). On a headless box pandagl falls back to Mesa's llvmpipe software rasterizer under
# Xvfb, rendering MetaDrive at ~2 Hz instead of 20 Hz. p3headlessgl talks to the GPU directly
# via EGL surfaceless -- no X server -- so the AMD 7900 XT does the rendering. This must be
# set before any panda3d ShowBase/engine import creates the display.
if os.environ.get("MD_EGL", "1") != "0":
  # GPU split, verified by strace: EGL surfaceless opens renderD129 (card1) and tinygrad's
  # AMD backend opens /dev/kfd + renderD128 (card0), so the two already land on separate
  # cards on this box. DRI_PRIME is IGNORED by the EGL surfaceless path -- setting it to 0
  # or 1 still opens renderD129 -- so do not rely on it to steer the renderer.
  #
  # modeld can still segfault in hcq.wait -> as_memoryview (the GPU->CPU output copy) when
  # card0 is low on VRAM: this box runs other GPU services holding ~19.5 of 24.5 GB on BOTH
  # cards, leaving ~5 GB. A failed allocation mid-JIT leaves tinygrad's HCQ signal memory
  # invalid. If modeld segfaults, check free VRAM first (mem_info_vram_used vs _total)
  # rather than assuming GPU contention.

  from panda3d.core import loadPrcFileData
  loadPrcFileData("", "load-display p3headlessgl")
  loadPrcFileData("", "aux-display pandagl")
  os.environ.pop("DISPLAY", None)

  # MetaDrive asks for multisamples=16 on its tonemapping FilterManager buffer. This GPU
  # tops out at 8 and Panda3D refuses rather than downgrading, so make_output returns None,
  # render_scene_into returns None, and RGBCamera._setup_effect / simplepbr blow up on
  # set_shader(None).
  #
  # Do NOT "fix" this by stubbing tonemapping out. The camera renders to a linear float
  # RGBA16 target and the tonemap shader is what maps it to displayable sRGB; without it
  # the road surface clips to pure white (row bands 207/246/255/255 vs a correct ~96) and
  # the model sees a white void where the lane lines are -- laneLineProbs collapses to
  # ~0.007 against ~0.99 on real footage, and it plans a 6 m path instead of 176 m.
  #
  # We disable MSAA rather than just capping it to 8. These are full-res RGBA16-FLOAT
  # targets (1928x1208x8B ~= 19 MB each) and MSAA multiplies that per sample. This box runs
  # other GPU services that already hold ~19.5 of 24.5 GB VRAM on both cards, so 8x MSAA
  # float targets tip it into thrashing -- rendering benchmarks at 11 ms standalone but the
  # full pipeline collapses to 0.3 Hz. Antialiasing does nothing for lane perception.
  # The count is hardcoded in RGBCamera._setup_effect (4 on macOS, 16 elsewhere), and
  # FrameBufferProperties / GraphicsEngine are immutable C++ types that cannot be patched,
  # so is_mac() is the only seam; MD_MSAA can raise it again if VRAM is ever free.
  _MSAA = int(os.environ.get("MD_MSAA", "0"))

  import metadrive.component.sensors.rgb_camera as _rgbcam
  _orig_is_mac = _rgbcam.is_mac
  _orig_setup_effect = _rgbcam.RGBCamera._setup_effect
  _orig_p3d_fbp = _rgbcam.p3d.FrameBufferProperties

  class _NoMsaaFBP(_orig_p3d_fbp):
    def set_multisamples(self, n):
      return super().set_multisamples(_MSAA)

  def _low_ms_setup_effect(self):
    _rgbcam.is_mac = lambda: True
    _rgbcam.p3d.FrameBufferProperties = _NoMsaaFBP
    try:
      return _orig_setup_effect(self)
    finally:
      _rgbcam.is_mac = _orig_is_mac
      _rgbcam.p3d.FrameBufferProperties = _orig_p3d_fbp
  _rgbcam.RGBCamera._setup_effect = _low_ms_setup_effect

  # engine_core passes msaa_samples=16 to simplepbr on non-Mac for the same reason, so its
  # tonemap buffer fails identically. Patch the simplepbr entry point directly rather than
  # the is_mac() branch here -- that branch also forces gl-version 4 1 and an onscreen
  # window, which breaks the headless pipe entirely.
  import metadrive.engine.core.engine_core as _ec
  _orig_pbr_init = _ec.init
  def _low_ms_pbr_init(*a, **kw):
    kw["msaa_samples"] = _MSAA
    return _orig_pbr_init(*a, **kw)
  _ec.init = _low_ms_pbr_init

from panda3d.core import Vec3

from metadrive.engine.core.engine_core import EngineCore
from metadrive.engine.core.image_buffer import ImageBuffer
from metadrive.envs.metadrive_env import MetaDriveEnv
from metadrive.obs.image_obs import ImageObservation
from metadrive.component.lane.circular_lane import CircularLane

from openpilot.common.realtime import Ratekeeper

from openpilot.tools.sim.lib.common import vec3
from openpilot.tools.sim.lib.camerad import W, H

C3_POSITION = Vec3(0.0, 0, 1.22)
C3_HPR = Vec3(0, 0,0)


metadrive_simulation_state = namedtuple("metadrive_simulation_state", ["running", "done", "done_info"])
metadrive_vehicle_state = namedtuple("metadrive_vehicle_state", ["velocity", "position", "bearing", "steering_angle"])

def apply_metadrive_patches(arrive_dest_done=True):
  # By default, metadrive won't try to use cuda images unless it's used as a sensor for vehicles, so patch that in
  def add_image_sensor_patched(self, name: str, cls, args):
    if self.global_config["image_on_cuda"]:# and name == self.global_config["vehicle_config"]["image_source"]:
      sensor = cls(*args, self, cuda=True)
    else:
      sensor = cls(*args, self, cuda=False)
    assert isinstance(sensor, ImageBuffer), "This API is for adding image sensor"
    self.sensors[name] = sensor

  EngineCore.add_image_sensor = add_image_sensor_patched

  # we aren't going to use the built-in observation stack, so disable it to save time
  def observe_patched(self, *args, **kwargs):
    return self.state

  ImageObservation.observe = observe_patched

  # disable destination, we want to loop forever
  def arrive_destination_patch(self, *args, **kwargs):
    return False

  if not arrive_dest_done:
    MetaDriveEnv._is_arrive_destination = arrive_destination_patch

def metadrive_process(dual_camera: bool, config: dict, camera_array, wide_camera_array, image_lock,
                      controls_recv: Connection, simulation_state_send: Connection, vehicle_state_send: Connection,
                      exit_event, op_engaged, test_duration, test_run):
  arrive_dest_done = config.pop("arrive_dest_done", True)
  apply_metadrive_patches(arrive_dest_done)

  road_image = np.frombuffer(camera_array.get_obj(), dtype=np.uint8).reshape((H, W, 3))
  if dual_camera:
    assert wide_camera_array is not None
    wide_road_image = np.frombuffer(wide_camera_array.get_obj(), dtype=np.uint8).reshape((H, W, 3))

  env = MetaDriveEnv(config)

  # Ground-truth geometry logging for the corner-cutting experiment. Off unless MD_GT_LOG is set,
  # so normal bridge use is untouched. time.monotonic() is exactly the clock cereal stamps into
  # logMonoTime (messaging/__init__.py), so these rows join straight onto the rlog.
  gt_path = os.environ.get("MD_GT_LOG")
  gt_file = None
  if gt_path:
    gt_file = open(gt_path, "w", buffering=1)
    gt_file.write("t,pos_x,pos_y,heading,speed,lat,long,width,radius,clockwise,lane_heading,on_lane\n")

  def log_ground_truth():
    if gt_file is None:
      return
    v = env.vehicle
    try:
      lane, _, on_lane = v.navigation._get_current_lane(v)
    except Exception:
      return
    if lane is None:
      return
    longitudinal, lateral = lane.local_coordinates(v.position)
    if isinstance(lane, CircularLane):
      radius, clockwise = float(lane.radius), (1.0 if lane.is_clockwise() else -1.0)
    else:
      radius, clockwise = 0.0, 0.0
    fields = (
      f"{time.monotonic():.6f}", f"{v.position[0]:.4f}", f"{v.position[1]:.4f}",
      f"{v.heading_theta:.6f}", f"{float(np.linalg.norm(v.velocity)):.4f}",
      f"{lateral:.4f}", f"{longitudinal:.4f}", f"{lane.width_at(longitudinal):.3f}",
      f"{radius:.3f}", f"{clockwise:.1f}", f"{lane.heading_theta_at(longitudinal):.6f}",
      str(int(bool(on_lane))),
    )
    gt_file.write(",".join(fields) + "\n")

  def get_current_lane_info(vehicle):
    _, lane_info, on_lane = vehicle.navigation._get_current_lane(vehicle)
    lane_idx = lane_info[2] if lane_info is not None else None
    return lane_idx, on_lane

  def reset():
    env.reset()
    env.vehicle.config["max_speed_km_h"] = 1000
    lane_idx_prev, _ = get_current_lane_info(env.vehicle)

    simulation_state = metadrive_simulation_state(
      running=True,
      done=False,
      done_info=None,
    )
    simulation_state_send.send(simulation_state)

    return lane_idx_prev

  lane_idx_prev = reset()
  start_time = None

  def get_cam_as_rgb(cam):
    cam = env.engine.sensors[cam]
    cam.get_cam().reparentTo(env.vehicle.origin)
    cam.get_cam().setPos(C3_POSITION)
    cam.get_cam().setHpr(C3_HPR)
    img = cam.perceive(to_float=False)
    if not isinstance(img, np.ndarray):
      img = img.get() # convert cupy array to numpy
    return img

  rk = Ratekeeper(100, None)

  steer_ratio = 8
  vc = [0,0]

  while not exit_event.is_set():
    vehicle_state = metadrive_vehicle_state(
      velocity=vec3(x=float(env.vehicle.velocity[0]), y=float(env.vehicle.velocity[1]), z=0),
      position=env.vehicle.position,
      bearing=float(math.degrees(env.vehicle.heading_theta)),
      steering_angle=env.vehicle.steering * env.vehicle.MAX_STEERING
    )
    vehicle_state_send.send(vehicle_state)

    if controls_recv.poll(0):
      while controls_recv.poll(0):
        steer_angle, gas, should_reset = controls_recv.recv()

      steer_metadrive = steer_angle * 1 / (env.vehicle.MAX_STEERING * steer_ratio)
      steer_metadrive = np.clip(steer_metadrive, -1, 1)

      vc = [steer_metadrive, gas]

      if should_reset:
        lane_idx_prev = reset()
        start_time = None

    is_engaged = op_engaged.is_set()
    if is_engaged and start_time is None:
      start_time = time.monotonic()

    if rk.frame % 5 == 0:
      _, _, terminated, _, _ = env.step(vc)
      timeout = True if start_time is not None and time.monotonic() - start_time >= test_duration else False
      lane_idx_curr, on_lane = get_current_lane_info(env.vehicle)
      out_of_lane = lane_idx_curr != lane_idx_prev or not on_lane
      lane_idx_prev = lane_idx_curr

      log_ground_truth()

      if terminated or ((out_of_lane or timeout) and test_run):
        if terminated:
          done_result = env.done_function("default_agent")
        elif out_of_lane:
          done_result = (True, {"out_of_lane" : True})
        elif timeout:
          done_result = (True, {"timeout" : True})

        simulation_state = metadrive_simulation_state(
          running=False,
          done=done_result[0],
          done_info=done_result[1],
        )
        simulation_state_send.send(simulation_state)

      if dual_camera:
        wide_road_image[...] = get_cam_as_rgb("rgb_wide")
      road_image[...] = get_cam_as_rgb("rgb_road")
      image_lock.release()

    rk.keep_time()
