"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import base64
import pickle
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

from openpilot.common.parameterized import parameterized

import openpilot.sunnypilot.modeld_v2.modeld as modeld_module
import openpilot.sunnypilot.modeld_v2.model_adapters as model_adapters
from openpilot.sunnypilot.modeld_v2.frame_resize import FrameResize

SOURCE_COPY_SIZE = 3735552  # 1928x1208 NV12, stride-padded
TARGET_COPY_SIZE = 1622016  # 1344x760


def native_jits():
  # the shape of a converted upstream model (MD v3), shrunk: warped frames in, one recurrent state
  specs = {'new_img': ((2, 6, 4, 8), '|u1', 'CPU'), 'desire': ((8,), '<f4', 'CPU'), 'traffic_convention': ((1, 2), '<f4', 'CPU'),
           'state_feat_q': ((4, 1, 8), '<f4', 'CPU')}
  slices = base64.b64encode(pickle.dumps({'plan': slice(0, 4)})).decode()
  return {'input_specs': specs, 'run': Mock(),
          'metadata': {'metadata': {'output_slices': slices}, 'output_shapes': {'outputs': (1, 4), 'next_state_feat_q': (4, 1, 8)}},
          'output_specs': {'outputs': ((1, 4), '<f4', 'CPU'), 'next_state_feat_q': ((4, 1, 8), '<f4', 'CPU')}}


def make_adapter(comma_hardware=True, device_type='tizi', chestnut=True, cam_size=(1928, 1208)):
  with patch.object(model_adapters, 'COMMA_HARDWARE', comma_hardware), \
       patch.object(model_adapters.HARDWARE, 'get_device_type', lambda: device_type):
    return model_adapters.get_model_adapter(native_jits(), *cam_size, 'CPU', 'CPU', 'CPU', chestnut=chestnut)


def source_frame(seed, size=SOURCE_COPY_SIZE):
  return ((np.arange(size, dtype=np.uint32) * seed) % 251).astype(np.uint8)


class TestNativeFrameResize(unittest.TestCase):
  def test_comma_3x_chestnut_ships_resized_frames(self):
    adapter = make_adapter()
    self.assertIsInstance(adapter, model_adapters.NativeTinygradAdapter)
    self.assertIsNotNone(adapter.frame_resize)
    self.assertEqual(adapter.frame_copy_size, TARGET_COPY_SIZE)
    self.assertEqual(adapter.frames.shape, (2, TARGET_COPY_SIZE))

    frames = {'img': SimpleNamespace(data=memoryview(source_frame(3))), 'big_img': source_frame(7)}
    adapter.copy_frames(frames)
    expected = np.empty(TARGET_COPY_SIZE, dtype=np.uint8)
    for i, key in enumerate(('img', 'big_img')):
      FrameResize().resize(frames[key].data if key == 'img' else frames[key], expected)
      np.testing.assert_array_equal(adapter.frames[i], expected)

  def test_transform_is_corrected_per_camera_without_compounding(self):
    adapter = make_adapter()
    road = np.array([[1, .1, 30], [.2, 1.1, 40], [.001, .002, 1]], dtype=np.float32)
    wide = np.array([[.9, 0, 10], [0, .8, 20], [0, .003, 1]], dtype=np.float32)
    sx, sy = 1344 / 1928, 760 / 1208
    scale = np.array([[sx, 0, 0.5 * sx - 0.5], [0, sy, 0.5 * sy - 0.5], [0, 0, 1]], dtype=np.float32)
    for _ in range(3):  # modeld rewrites tfm before every run
      adapter.numpy_inputs['tfm'][0], adapter.numpy_inputs['tfm'][1] = road, wide
      adapter.run()
      np.testing.assert_allclose(adapter.numpy_inputs['tfm'][0], scale @ road, rtol=1e-6, atol=1e-5)
      np.testing.assert_allclose(adapter.numpy_inputs['tfm'][1], scale @ wide, rtol=1e-6, atol=1e-5)
    self.assertEqual(adapter.run_model.call_count, 3)

  @parameterized.expand([((1928, 1208), SOURCE_COPY_SIZE), ((1344, 760), TARGET_COPY_SIZE)])
  def test_warmup_frames_fit_copy_frames(self, cam_size, frame_size):
    adapter = make_adapter(cam_size=cam_size)
    dummy_frames, _, _ = adapter.get_dummy_inputs()
    self.assertEqual({len(f) for f in dummy_frames.values()}, {frame_size})
    adapter.copy_frames(dummy_frames)

  def test_short_source_raises_before_the_model_runs(self):
    adapter = make_adapter()
    with self.assertRaises(ValueError):
      adapter.copy_frames({'img': np.zeros(SOURCE_COPY_SIZE - 1, dtype=np.uint8)})
    adapter.run_model.assert_not_called()

  @parameterized.expand([
    (True, 'tici', True, (1928, 1208)),
    (True, 'mici', True, (1344, 760)),
    (False, 'pc', True, (1928, 1208)),
    (True, 'tizi', False, (1928, 1208)),
    (True, 'tizi', True, (1344, 760)),
  ])
  def test_ineligible_keep_native_frames_and_transform(self, comma_hardware, device_type, chestnut, cam_size):
    adapter = make_adapter(comma_hardware, device_type, chestnut, cam_size)
    self.assertIsNone(adapter.frame_resize)
    stride, y_height, uv_height, _ = model_adapters.get_nv12_info(*cam_size)
    self.assertEqual(adapter.frame_copy_size, stride * (y_height + uv_height))
    source = source_frame(5, adapter.frame_copy_size)
    adapter.copy_frames({'img': source, 'big_img': source})
    np.testing.assert_array_equal(adapter.frames[0], source)
    transform = np.array([[1, 2, 3], [4, 5, 6], [.01, .02, 1]], dtype=np.float32)
    adapter.numpy_inputs['tfm'][:] = transform
    adapter.run()
    np.testing.assert_array_equal(adapter.numpy_inputs['tfm'][1], transform)

  @parameterized.expand([('tizi', (1344, 760)), ('tici', (1928, 1208))])
  def test_loader_fuses_the_matching_warp(self, device_type, warp_size):
    fused = Mock(return_value='extend')
    with patch.object(model_adapters, 'COMMA_HARDWARE', True), \
         patch.object(model_adapters.HARDWARE, 'get_device_type', lambda: device_type), \
         patch.object(modeld_module, 'fused_with_warp', fused), \
         patch.object(modeld_module, 'load_model', Mock(side_effect=StopIteration)):
      with self.assertRaises(StopIteration):
        modeld_module.ModelState._init_combined(SimpleNamespace(chestnut=True), 'model.pkl', 1928, 1208, None)
    fused.assert_called_once_with(*warp_size, True)


if __name__ == '__main__':
  unittest.main()
