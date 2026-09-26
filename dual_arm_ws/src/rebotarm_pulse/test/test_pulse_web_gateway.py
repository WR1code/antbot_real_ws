from rebotarm_pulse.pulse_web_gateway import PulseEventBuffer


def test_event_buffer_delivers_new_events_in_order():
    events = PulseEventBuffer(maxlen=3)
    first = events.publish({"type": "sample", "channel": "S1"})
    second = events.publish({"type": "sample", "channel": "S2"})

    assert first == 1
    assert second == 2
    assert events.events_after(0, timeout=0.0) == [
        (1, {"type": "sample", "channel": "S1"}),
        (2, {"type": "sample", "channel": "S2"}),
    ]
    assert events.events_after(1, timeout=0.0) == [
        (2, {"type": "sample", "channel": "S2"})
    ]


def test_event_buffer_tracks_bridge_status():
    events = PulseEventBuffer()
    events.update_status("serial_connected", True)
    events.update_status("zero_calibrated", True)

    assert events.snapshot() == {
        "serial_connected": True,
        "zero_calibrated": True,
    }
    assert events.events_after(0, timeout=0.0)[-1][1] == {
        "type": "status",
        "serial_connected": True,
        "zero_calibrated": True,
    }
