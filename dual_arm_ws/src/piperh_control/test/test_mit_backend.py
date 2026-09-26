from types import SimpleNamespace
import sys

import pytest

from piperh_control.mit_backend import OfficialMitBackend, firmware_profile, read_firmware


@pytest.mark.parametrize("version,profile", [
    ("S-V1.8-2", "default"), ("S-V1.8-3", "v183"),
    ("S-V1.8-8", "v188"), ("S-V1.9-0", "v189"),
])
def test_firmware_profile(version, profile):
    assert firmware_profile(version) == profile


@pytest.mark.parametrize("version", ["", "S-V2.0-0", "S-V1.8", "Piper-H"])
def test_unknown_firmware_fails_closed(version):
    with pytest.raises(ValueError):
        firmware_profile(version)


def test_firmware_query_is_read_only_except_query(monkeypatch):
    calls = []
    class Bus:
        def __enter__(self): return self
        def __exit__(self, *_args): pass
        def set_filters(self, filters): calls.append(("filters", filters))
        def send(self, message, timeout):
            calls.append(("send", message.arbitration_id, bytes(message.data), timeout))
        def recv(self, timeout):
            return SimpleNamespace(arbitration_id=0x4AF, is_extended_id=False,
                                   data=b"S-V1.9-02607")
    monkeypatch.setitem(sys.modules, "can", SimpleNamespace(
        Bus=lambda **_kwargs: Bus(),
        Message=lambda **kwargs: SimpleNamespace(**kwargs),
    ))
    assert read_firmware("can0") == "S-V1.9-0"
    assert calls[1][1:] == (0x4AF, bytes([1, 0, 0, 0, 0, 0, 0, 0]), 0.2)


def test_backend_selects_piper_h_v189_and_public_mit_api(monkeypatch):
    calls = []
    arm = SimpleNamespace(
        connect=lambda: calls.append(("connect",)),
        set_auto_set_motion_mode_enabled=lambda value: calls.append(("auto", value)),
        set_motion_mode=lambda value: calls.append(("mode", value)),
        move_mit=lambda **kwargs: calls.append(("mit", kwargs)),
    )
    monkeypatch.setitem(sys.modules, "pyAgxArm", SimpleNamespace(
        AgxArmFactory=SimpleNamespace(create_arm=lambda cfg: (calls.append(("config", cfg)) or arm)),
        ArmModel=SimpleNamespace(PIPER_H="piper_h"),
        create_agx_arm_config=lambda **kwargs: kwargs,
    ))
    backend = OfficialMitBackend("can0", "S-V1.9-0")
    backend.connect()
    backend.enter_mit()
    backend.send_joint(2, 0.5, 0.0, 0.8, 0.2)
    assert calls[0][1] == {
        "robot": "piper_h", "firmeware_version": "v189", "channel": "can0"
    }
    assert calls[-1][1]["t_ff"] == pytest.approx(0.2)
