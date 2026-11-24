import json
import time

from motorbridge import Controller

from hw_mit_five_poses import CONFIG, NAMES, OUT

dm = CONFIG["hardware"]["dm"]
settings = CONFIG["tools"]["dm_probe"]
bus = Controller.from_dm_serial(dm["device"], dm["baudrate"])
try:
    motors = {}
    for name in NAMES:
        cfg = CONFIG["joints"][name]["motor"]
        motors[name] = bus.add_damiao_motor(cfg["id"], cfg["feedback_id"], cfg["model"])
    latest = {}
    deadline = time.monotonic() + settings["duration_sec"]
    while time.monotonic() < deadline:
        for motor in motors.values():
            motor.request_feedback()
        bus.poll_feedback_once()
        for name, motor in motors.items():
            state = motor.get_state()
            if state is not None and getattr(state, "has_value", True):
                latest[name] = state
        time.sleep(settings["poll_sec"])
    if len(latest) != len(motors):
        raise RuntimeError(f"缺少反馈：{set(motors) - set(latest)}")
    results = {}
    for name, motor in motors.items():
        params = CONFIG["joints"][name]["motor"]
        state = latest[name]
        factor = params["direction"] * params["scale"]
        row = {
            "q": state.pos * factor + params["zero_offset"],
            "dq": state.vel * factor, "torque_nm": state.torq,
            "temperature_c": max(state.t_mos, state.t_rotor), "status": state.status_code,
            "firmware_mode": motor.get_register_u32(10, settings["register_timeout_ms"]),
            "pmax": motor.get_register_f32(21, settings["register_timeout_ms"]),
            "vmax": motor.get_register_f32(22, settings["register_timeout_ms"]),
            "tmax": motor.get_register_f32(23, settings["register_timeout_ms"]),
        }
        results[name] = row
        print(name, row, flush=True)
    (OUT / "probe.json").write_text(json.dumps(results, indent=2) + "\n")
finally:
    bus.close()
