#include "selfdrive/pandad/pandad.h"

#include <algorithm>
#include <array>
#include <bitset>
#include <cassert>
#include <cerrno>
#include <cinttypes>
#include <cstdio>
#include <ctime>
#include <memory>
#include <thread>
#include <utility>

#include <sys/stat.h>

#include "openpilot/cereal/gen/cpp/car.capnp.h"
#include "openpilot/cereal/messaging/messaging.h"
#include "openpilot/cereal/services.h"
#include "common/ratekeeper.h"
#include "common/swaglog.h"
#include "common/timing.h"
#include "common/util.h"
#include "common/hardware/hw.h"

#define MAX_IR_PANDA_VAL 50
#define CUTOFF_IL 400
#define SATURATE_IL 1000

ExitHandler do_exit;

bool check_connected(Panda *panda) {
  if (!panda->connected()) {
    do_exit = true;
    return false;
  }
  return true;
}

bool process_mads_heartbeat(SubMaster *sm) {
  const auto &mads = (*sm)["selfdriveStateSP"].getSelfdriveStateSP().getMads();
  return sm->allAliveAndValid({"selfdriveStateSP"}) && mads.getEnabled();
}

Panda *connect(std::string serial) {
  std::unique_ptr<Panda> panda;
  try {
    panda = std::make_unique<Panda>(serial);
  } catch (std::exception &e) {
    return nullptr;
  }

  // common panda config
  if (getenv("BOARDD_LOOPBACK")) {
    panda->set_loopback(true);
  }
  //panda->enable_deepsleep();

  for (int i = 0; i < PANDA_CAN_CNT; i++) {
    panda->set_can_fd_auto(i, true);
  }

  bool is_supported_panda = std::find(SUPPORTED_PANDA_TYPES.begin(), SUPPORTED_PANDA_TYPES.end(), panda->hw_type) != SUPPORTED_PANDA_TYPES.end();

  if (!is_supported_panda) {
    LOGW("panda %s is not supported (hw_type: %i), skipping firmware check...", panda->hw_serial().c_str(), static_cast<uint16_t>(panda->hw_type));
    return panda.release();
  }

  if (!panda->up_to_date() && !getenv("BOARDD_SKIP_FW_CHECK")) {
    throw std::runtime_error("Panda firmware out of date. Run pandad.py to update.");
  }

  return panda.release();
}

void can_send_thread(Panda *panda, bool fake_send) {
  util::set_thread_name("pandad_can_send");

  AlignedBuffer aligned_buf;
  std::unique_ptr<Context> context(Context::create());
  std::unique_ptr<SubSocket> subscriber(SubSocket::create(context.get(), "sendcan", "127.0.0.1", false, true, services.at("sendcan").queue_size));
  assert(subscriber != NULL);
  subscriber->setTimeout(100);

  // run as fast as messages come in
  while (!do_exit && check_connected(panda)) {
    std::unique_ptr<Message> msg(subscriber->receive());
    if (!msg) {
      continue;
    }

    capnp::FlatArrayMessageReader cmsg(aligned_buf.align(msg.get()));
    cereal::Event::Reader event = cmsg.getRoot<cereal::Event>();

    // Don't send if older than 1 second
    if ((nanos_since_boot() - event.getLogMonoTime() < 1e9) && !fake_send) {
      LOGT("sending sendcan to panda: %s", (panda->hw_serial()).c_str());
      panda->can_send(event.getSendcan());
      LOGT("sendcan sent to panda: %s", (panda->hw_serial()).c_str());
    } else {
      LOGE("sendcan too old to send: %" PRIu64 ", %" PRIu64, nanos_since_boot(), event.getLogMonoTime());
    }
  }
}

// Receive-only CAN capture for a parked car (2026-11-23, Bluelink stop/start charging test). On CanListenRequest =
// seconds (default 600, 10..900; offroad, ignition off) leave power save for that window with safety NO_OUTPUT, so
// nothing is transmitted, and append every received frame on all buses to /data/can_listen/<local time>.csv
// (mono_ns,wall_ns,bus,address,data). Timestamps are per USB batch (pandad reads at 100 Hz). CanWake requests are
// refused while it runs (they transmit UDS). Ends early if the car starts; CanListenLast = "<path> <frames>".
class CanListen {
public:
  bool active() const { return active_; }

  bool update(Panda *panda, uint64_t frame, bool is_onroad, bool ignition) {
    if (active_) {
      if (is_onroad || ignition) {
        stop(panda, "car started");
      } else if (nanos_since_boot() >= end_ns_) {
        stop(panda, "done");
      }
      return active_;
    }
    if (frame % 100 != 0) {
      return false;
    }
    const std::string req = params_.get("CanListenRequest");
    if (req.empty()) {
      return false;
    }
    params_.remove("CanListenRequest");
    if (is_onroad || ignition) {
      LOGW("can listen: request ignored, car is on");
      return false;
    }
    int secs = DEFAULT_S;
    try {
      secs = std::stoi(req);
    } catch (...) {
      secs = DEFAULT_S;
    }
    secs = std::clamp(secs, MIN_S, MAX_S);

    mkdir(DIR, 0775);
    char ts[32];
    const time_t now = time(nullptr);
    struct tm lt;
    localtime_r(&now, &lt);
    strftime(ts, sizeof(ts), "%Y%m%d-%H%M%S", &lt);
    path_ = std::string(DIR) + "/" + ts + ".csv";
    file_ = fopen(path_.c_str(), "w");
    if (file_ == nullptr) {
      LOGE("can listen: cannot open %s", path_.c_str());
      return false;
    }
    fprintf(file_, "mono_ns,wall_ns,bus,address,data\n");
    panda->set_safety_model(cereal::CarParams::SafetyModel::NO_OUTPUT);
    panda->set_power_saving(false);
    active_ = true;
    frames_ = 0;
    end_ns_ = nanos_since_boot() + uint64_t(secs) * 1000000000ULL;
    LOGW("can listen: start %d s -> %s", secs, path_.c_str());
    return true;
  }

  void log(const std::vector<can_frame> &frames) {
    if (!active_ || file_ == nullptr || frames.empty()) {
      return;
    }
    const uint64_t mono = nanos_since_boot();
    struct timespec w;
    clock_gettime(CLOCK_REALTIME, &w);
    const uint64_t wall = uint64_t(w.tv_sec) * 1000000000ULL + uint64_t(w.tv_nsec);
    for (const auto &f : frames) {
      fprintf(file_, "%" PRIu64 ",%" PRIu64 ",%ld,%ld,", mono, wall, f.src, f.address);
      for (unsigned char c : f.dat) {
        fprintf(file_, "%02x", c);
      }
      fputc('\n', file_);
    }
    frames_ += frames.size();
  }

private:
  static constexpr const char *DIR = "/data/can_listen";
  static constexpr int DEFAULT_S = 600;
  static constexpr int MIN_S = 10;
  static constexpr int MAX_S = 900;

  void stop(Panda *panda, const char *why) {
    active_ = false;
    if (file_ != nullptr) {
      fclose(file_);
      file_ = nullptr;
    }
    panda->set_safety_model(cereal::CarParams::SafetyModel::NO_OUTPUT);
    panda->set_power_saving(true);
    params_.put("CanListenLast", path_ + " " + std::to_string(frames_));
    LOGW("can listen: %s, %" PRIu64 " frames -> %s", why, frames_, path_.c_str());
  }

  Params params_;
  bool active_ = false;
  FILE *file_ = nullptr;
  std::string path_;
  uint64_t frames_ = 0;
  uint64_t end_ns_ = 0;
};

void can_recv(Panda *panda, PubMaster *pm, CanListen *listen = nullptr) {
  static std::vector<can_frame> raw_can_data;
  {
    raw_can_data.clear();
    bool comms_healthy = panda->can_receive(raw_can_data);
    if (listen != nullptr) {
      listen->log(raw_can_data);
    }

    MessageBuilder msg;
    auto evt = msg.initEvent();
    evt.setValid(comms_healthy);
    auto canData = evt.initCan(raw_can_data.size());
    for (size_t i = 0; i < raw_can_data.size(); ++i) {
      canData[i].setAddress(raw_can_data[i].address);
      canData[i].setDat(kj::arrayPtr((uint8_t*)raw_can_data[i].dat.data(), raw_can_data[i].dat.size()));
      canData[i].setSrc(raw_can_data[i].src);
    }
    pm->send("can", msg);
  }
}

void fill_panda_state(cereal::PandaState::Builder &ps, cereal::PandaState::PandaType hw_type, const health_t &health) {
  ps.setVoltage(health.voltage_pkt);
  ps.setCurrent(health.current_pkt);
  ps.setUptime(health.uptime_pkt);
  ps.setSafetyTxBlocked(health.safety_tx_blocked_pkt);
  ps.setSafetyRxInvalid(health.safety_rx_invalid_pkt);
  ps.setIgnitionLine((health.flags_pkt & HEALTH_FLAG_IGNITION_LINE) != 0U);
  ps.setIgnitionCan((health.flags_pkt & HEALTH_FLAG_IGNITION_CAN) != 0U);
  ps.setControlsAllowed((health.flags_pkt & HEALTH_FLAG_CONTROLS_ALLOWED) != 0U);
  ps.setTxBufferOverflow(health.tx_buffer_overflow_pkt);
  ps.setRxBufferOverflow(health.rx_buffer_overflow_pkt);
  ps.setPandaType(hw_type);
  ps.setSafetyModel(cereal::CarParams::SafetyModel(health.safety_mode_pkt));
  ps.setSafetyParam(health.safety_param_pkt);
  ps.setFaultStatus(cereal::PandaState::FaultStatus(health.fault_status_pkt));
  ps.setPowerSaveEnabled((health.flags_pkt & HEALTH_FLAG_POWER_SAVE_ENABLED) != 0U);
  ps.setHeartbeatLost((health.flags_pkt & HEALTH_FLAG_HEARTBEAT_LOST) != 0U);
  ps.setAlternativeExperience(health.alternative_experience_pkt);
  ps.setHarnessStatus(cereal::PandaState::HarnessStatus(health.car_harness_status_pkt));
  ps.setInterruptLoad(health.interrupt_load_pkt / 255.0f);
  ps.setFanPower(health.fan_power);
  ps.setSafetyRxChecksInvalid((health.flags_pkt & HEALTH_FLAG_SAFETY_RX_CHECKS_INVALID) != 0U);
  ps.setSpiErrorCount(health.spi_error_count_pkt);
  ps.setSbu1Voltage(health.sbu1_voltage_mV / 1000.0f);
  ps.setSbu2Voltage(health.sbu2_voltage_mV / 1000.0f);
  ps.setSoundOutputLevel(health.sound_output_level_pkt);
  ps.setControlsAllowedLateral(health.controls_allowed_sp_pkt & 1);
  ps.setControlsAllowedLongitudinal((health.controls_allowed_sp_pkt >> 1) & 1);
}

void fill_panda_can_state(cereal::PandaState::PandaCanState::Builder &cs, const can_health_t &can_health) {
  cs.setBusOff((bool)can_health.bus_off);
  cs.setBusOffCnt(can_health.bus_off_cnt);
  cs.setErrorWarning((bool)can_health.error_warning);
  cs.setErrorPassive((bool)can_health.error_passive);
  cs.setLastError(cereal::PandaState::PandaCanState::LecErrorCode(can_health.last_error));
  cs.setLastStoredError(cereal::PandaState::PandaCanState::LecErrorCode(can_health.last_stored_error));
  cs.setLastDataError(cereal::PandaState::PandaCanState::LecErrorCode(can_health.last_data_error));
  cs.setLastDataStoredError(cereal::PandaState::PandaCanState::LecErrorCode(can_health.last_data_stored_error));
  cs.setReceiveErrorCnt(can_health.receive_error_cnt);
  cs.setTransmitErrorCnt(can_health.transmit_error_cnt);
  cs.setTotalErrorCnt(can_health.total_error_cnt);
  cs.setTotalTxLostCnt(can_health.total_tx_lost_cnt);
  cs.setTotalRxLostCnt(can_health.total_rx_lost_cnt);
  cs.setTotalTxCnt(can_health.total_tx_cnt);
  cs.setTotalRxCnt(can_health.total_rx_cnt);
  cs.setTotalFwdCnt(can_health.total_fwd_cnt);
  cs.setCanSpeed(can_health.can_speed);
  cs.setCanDataSpeed(can_health.can_data_speed);
  cs.setCanfdEnabled(can_health.canfd_enabled);
  cs.setBrsEnabled(can_health.brs_enabled);
  cs.setCanfdNonIso(can_health.canfd_non_iso);
  cs.setIrq0CallRate(can_health.irq0_call_rate);
  cs.setIrq1CallRate(can_health.irq1_call_rate);
  cs.setIrq2CallRate(can_health.irq2_call_rate);
  cs.setCanCoreResetCnt(can_health.can_core_reset_cnt);
}

std::optional<bool> send_panda_states(PubMaster *pm, Panda *panda, bool is_onroad, bool spoofing_started, bool always_offroad, bool can_wake_active) {
  // build msg
  MessageBuilder msg;
  auto evt = msg.initEvent();
  auto pss = evt.initPandaStates(1);

  auto health_opt = panda->get_state();
  if (!health_opt) {
    return std::nullopt;
  }

  health_t health = *health_opt;

  std::array<can_health_t, PANDA_CAN_CNT> can_health{};
  for (uint32_t i = 0; i < PANDA_CAN_CNT; i++) {
    auto can_health_opt = panda->get_can_state(i);
    if (!can_health_opt) {
      return std::nullopt;
    }
    can_health[i] = *can_health_opt;
  }

  if (spoofing_started) {
    health.flags_pkt |= HEALTH_FLAG_IGNITION_LINE;
  }

  bool ignition_local = ((health.flags_pkt & (HEALTH_FLAG_IGNITION_LINE | HEALTH_FLAG_IGNITION_CAN)) != 0U) && !always_offroad;

  // Make sure CAN buses are live: safety_setter_thread does not work if Panda CAN are silent and there is only one other CAN node
  if (health.safety_mode_pkt == (uint8_t)(cereal::CarParams::SafetyModel::SILENT)) {
    panda->set_safety_model(cereal::CarParams::SafetyModel::NO_OUTPUT);
  }

  // a CanWake window (offroad, ignition off) keeps the transceivers up and ELM327 safety until it ends
  bool power_save_desired = !ignition_local && !can_wake_active;
  if (((health.flags_pkt & HEALTH_FLAG_POWER_SAVE_ENABLED) != 0U) != power_save_desired) {
    panda->set_power_saving(power_save_desired);
  }

  // set safety mode to NO_OUTPUT when car is off or we're not onroad. ELM327 is an alternative if we want to leverage athenad/connect
  bool should_close_relay = !ignition_local || !is_onroad;
  if (should_close_relay && !can_wake_active && (health.safety_mode_pkt != (uint8_t)(cereal::CarParams::SafetyModel::NO_OUTPUT))) {
    panda->set_safety_model(cereal::CarParams::SafetyModel::NO_OUTPUT);
  }

  if (!panda->comms_healthy()) {
    evt.setValid(false);
  }

  auto ps = pss[0];
  fill_panda_state(ps, panda->hw_type, health);

  auto cs = std::array{ps.initCanState0(), ps.initCanState1(), ps.initCanState2()};
  for (uint32_t j = 0; j < PANDA_CAN_CNT; j++) {
    fill_panda_can_state(cs[j], can_health[j]);
  }

  // Convert faults bitset to capnp list
  std::bitset<sizeof(health.faults_pkt) * 8> fault_bits(health.faults_pkt);
  auto faults = ps.initFaults(fault_bits.count());

  size_t j = 0;
  for (size_t f = size_t(cereal::PandaState::FaultType::RELAY_MALFUNCTION);
       f <= size_t(cereal::PandaState::FaultType::HEARTBEAT_LOOP_WATCHDOG); f++) {
    if (fault_bits.test(f)) {
      faults.set(j, cereal::PandaState::FaultType(f));
      j++;
    }
  }

  pm->send("pandaStates", msg);
  return ignition_local;
}

void send_peripheral_state(Panda *panda, PubMaster *pm) {
  auto health_opt = panda->get_state();
  if (!health_opt) {
    return;
  }

  // build msg
  MessageBuilder msg;
  auto evt = msg.initEvent();
  evt.setValid(panda->comms_healthy());

  auto ps = evt.initPeripheralState();
  ps.setPandaType(panda->hw_type);

  health_t health = *health_opt;
  ps.setVoltage(health.voltage_pkt);
  ps.setCurrent(health.current_pkt);

  uint16_t fan_speed_rpm = panda->get_fan_speed();
  ps.setFanSpeedRpm(fan_speed_rpm);

  pm->send("peripheralState", msg);
}

std::optional<bool> process_panda_state(Panda *panda, PubMaster *pm, bool engaged, bool engaged_mads, bool is_onroad, bool spoofing_started,
                                        bool always_offroad, bool can_wake_active) {
  auto ignition_opt = send_panda_states(pm, panda, is_onroad, spoofing_started, always_offroad, can_wake_active);
  if (!ignition_opt) {
    LOGE("Failed to get ignition_opt");
    return std::nullopt;
  }

  // check if we should have pandad reconnect
  if (!ignition_opt.value()) {
    if (!panda->comms_healthy()) {
      LOGE("Reconnecting, communication to panda not healthy");
      do_exit = true;
    }
  }

  panda->send_heartbeat(engaged, engaged_mads);
  return ignition_opt;
}

// Offroad CAN wake for mqttd's charge-cable probe. A parked Ioniq 6 sleeps its buses, so a cable plugged in later
// is never reported. On CanWakeRequest (offroad, ignition off) leave power save, switch to ELM327 safety (it only
// passes 8-byte diagnostic frames such as 0x7DF) and send UDS session control + tester present on buses 0 and 1
// for 3 s, then hold 10 s so the car's periodic status frames (0x405 plug state, 5 Hz) are received. Measured
// 2026-11-09: the car answers within 0.2 s. Ends early if the car starts.
class CanWake {
public:
  // true while a wake window is active
  bool update(Panda *panda, uint64_t frame, bool is_onroad, bool ignition, bool listening = false) {
    if (active_) {
      if (is_onroad || ignition) {
        active_ = false;
        LOGW("can wake: car started, window ended");
        return false;
      }
      const uint64_t n = frame - start_;
      if (n < SEND_FRAMES && n % 5 == 0) {
        send(panda);
      }
      if (n >= WINDOW_FRAMES) {
        active_ = false;
        panda->set_safety_model(cereal::CarParams::SafetyModel::NO_OUTPUT);
        panda->set_power_saving(true);
        LOGW("can wake: done");
      }
      return active_;
    }
    if (frame % 100 != 0 || !params_.getBool("CanWakeRequest")) {
      return false;
    }
    params_.remove("CanWakeRequest");
    if (listening) {
      LOGW("can wake: request ignored, a receive-only CAN listen is running");
      return false;
    }
    if (is_onroad || ignition) {
      LOGW("can wake: request ignored, car is on");
      return false;
    }
    panda->set_power_saving(false);
    panda->set_safety_model(cereal::CarParams::SafetyModel::ELM327, 1U);
    active_ = true;
    start_ = frame;
    LOGW("can wake: start");
    return true;
  }

private:
  static constexpr uint64_t SEND_FRAMES = 300;     // 3 s at 100 Hz
  static constexpr uint64_t WINDOW_FRAMES = 1300;  // + 10 s listening

  void send(Panda *panda) {
    static const std::array<std::array<uint8_t, 8>, 2> uds = {{
      {0x02, 0x10, 0x01, 0, 0, 0, 0, 0},  // diagnostic session control, default session
      {0x02, 0x3e, 0x80, 0, 0, 0, 0, 0},  // tester present, no response
    }};
    MessageBuilder msg;
    auto frames = msg.initEvent().initSendcan(4);
    int i = 0;
    for (uint8_t bus : {0, 1}) {
      for (const auto &dat : uds) {
        auto f = frames[i++];
        f.setAddress(0x7DF);
        f.setSrc(bus);
        f.setDat(kj::arrayPtr(dat.data(), dat.size()));
      }
    }
    panda->can_send(frames.asReader());
  }

  Params params_;
  bool active_ = false;
  uint64_t start_ = 0;
};

void process_peripheral_state(Panda *panda, PubMaster *pm, bool no_fan_control, bool is_onroad) {
  static Params params;
  static SubMaster sm({"deviceState", "cabinCameraState"});

  static uint64_t last_cabin_camera_t = 0;
  static uint16_t prev_fan_speed = 999;
  static int ir_pwr = 0;
  static int prev_ir_pwr = 999;
  static uint32_t prev_frame_id = UINT32_MAX;
  static bool driver_view = false;
  static bool not_car = false;
  static bool not_car_checked = false;

  // TODO: can we merge these?
  static FirstOrderFilter integ_lines_filter(0, 30.0, 0.05);
  static FirstOrderFilter integ_lines_filter_driver_view(0, 5.0, 0.05);

  {
    sm.update(0);
    if (sm.updated("deviceState") && !no_fan_control) {
      // Fan speed
      uint16_t fan_speed = sm["deviceState"].getDeviceState().getFanSpeedPercentDesired();
      if (fan_speed != prev_fan_speed || sm.frame % 100 == 0) {
        panda->set_fan_speed(fan_speed);
        prev_fan_speed = fan_speed;
      }
    }

    if (sm.updated("cabinCameraState")) {
      auto event = sm["cabinCameraState"];
      int cur_integ_lines = event.getCabinCameraState().getIntegLines();

      // reset the filter when camerad restarts
      if (event.getCabinCameraState().getFrameId() < prev_frame_id) {
        integ_lines_filter.reset(0);
        integ_lines_filter_driver_view.reset(0);
        driver_view = params.getBool("IsDriverViewEnabled");
      }
      prev_frame_id = event.getCabinCameraState().getFrameId();

      cur_integ_lines = (driver_view ? integ_lines_filter_driver_view : integ_lines_filter).update(cur_integ_lines);
      last_cabin_camera_t = event.getLogMonoTime();

      if (cur_integ_lines <= CUTOFF_IL) {
        ir_pwr = 0;
      } else if (cur_integ_lines > SATURATE_IL) {
        ir_pwr = 100;
      } else {
        ir_pwr = 100 * (cur_integ_lines - CUTOFF_IL) / (SATURATE_IL - CUTOFF_IL);
      }
    }

    // Disable IR on input timeout or when requested offroad.
    if (nanos_since_boot() - last_cabin_camera_t > 1e9 || (!is_onroad && params.getBool("DisableDriverCameraIR"))) {
      ir_pwr = 0;
    }

    // turn off IR leds if body
    if (!not_car_checked && is_onroad) {
      std::string cp_bytes = params.get("CarParams");
      if (cp_bytes.size() > 0) {
        AlignedBuffer aligned_buf;
        capnp::FlatArrayMessageReader cmsg(aligned_buf.align(cp_bytes.data(), cp_bytes.size()));
        cereal::CarParams::Reader CP = cmsg.getRoot<cereal::CarParams>();
        not_car = CP.getNotCar();
        not_car_checked = true;
      }
    }
    if (not_car) {
      ir_pwr = 0;
    }

    if (ir_pwr != prev_ir_pwr || sm.frame % 100 == 0) {
      int16_t ir_panda = util::map_val(ir_pwr, 0, 100, 0, MAX_IR_PANDA_VAL);
      panda->set_ir_pwr(ir_panda);
      Hardware::set_ir_power(ir_pwr);
      prev_ir_pwr = ir_pwr;
    }
  }
}

void pandad_run(Panda *panda) {
  const bool no_fan_control = getenv("NO_FAN_CONTROL") != nullptr;
  const bool spoofing_started = getenv("STARTED") != nullptr;
  const bool fake_send = getenv("FAKESEND") != nullptr;

  // Start helper thread for event-driven sendcan.
  std::thread send_thread(can_send_thread, panda, fake_send);

  RateKeeper rk("pandad", 100);
  SubMaster sm({"selfdriveState", "deviceState", "selfdriveStateSP"});
  PubMaster pm({"can", "pandaStates", "peripheralState"});
  PandaSafety panda_safety(panda);
  bool engaged = false;
  bool engaged_mads = false;
  bool is_onroad = false;
  bool always_offroad = false;
  bool ignition = false;
  bool can_wake_active = false;
  CanWake can_wake;
  CanListen can_listen;

  // Main loop: receive CAN first, then process lower priority panda and peripheral state.
  while (!do_exit && check_connected(panda)) {
    can_recv(panda, &pm, &can_listen);

    // Process peripheral state at 20 Hz
    if (rk.frame() % 5 == 0) {
      process_peripheral_state(panda, &pm, no_fan_control, is_onroad);
    }

    // Process panda state at 10 Hz
    if (rk.frame() % 10 == 0) {
      sm.update(0);
      engaged = sm.allAliveAndValid({"selfdriveState"}) && sm["selfdriveState"].getSelfdriveState().getEnabled();
      if (sm.updated("deviceState")) {
        is_onroad = sm["deviceState"].getDeviceState().getStarted();
      }
      engaged_mads = process_mads_heartbeat(&sm);
      always_offroad = panda_safety.getOffroadMode();
      if (auto ign = process_panda_state(panda, &pm, engaged, engaged_mads, is_onroad, spoofing_started, always_offroad, can_wake_active || can_listen.active())) {
        ignition = *ign;
      }
      panda_safety.configureSafetyMode(is_onroad);
    }

    can_wake_active = can_wake.update(panda, rk.frame(), is_onroad, ignition, can_listen.active());
    can_listen.update(panda, rk.frame(), is_onroad, ignition);

    // Send out peripheralState at 2Hz
    if (rk.frame() % 50 == 0) {
      send_peripheral_state(panda, &pm);
    }

    // Forward logs from panda to cloudlog if available
    std::string log = panda->serial_read();
    if (!log.empty()) {
      if (log.find("Register 0x") != std::string::npos) {
        // Log register divergent faults as errors
        LOGE("%s", log.c_str());
      } else {
        LOGD("%s", log.c_str());
      }
    }

    rk.keepTime();
  }

  // Close relay on exit to prevent a fault
  if (is_onroad && !engaged) {
    if (panda->connected()) {
      panda->set_safety_model(cereal::CarParams::SafetyModel::NO_OUTPUT);
    }
  }

  send_thread.join();
}

void pandad_main_thread(std::string serial) {
  if (serial.empty()) {
    auto serials = Panda::list();

    if (serials.empty()) {
      LOGW("no pandas found, exiting");
      return;
    }
    serial = serials[0];
  }

  LOGW("connecting to panda: %s", serial.c_str());

  Panda *panda = nullptr;
  while (!do_exit) {
    panda = connect(serial);
    if (panda) break;
    util::sleep_for(100);
  }

  if (!do_exit) {
    LOGW("connected to panda");
    pandad_run(panda);
  }

  delete panda;
}
