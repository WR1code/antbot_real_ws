from rebotarm_pulse.pressure_protocol import parse_pressure_line


def test_firmware_measurement_and_diagnostic_lines():
    assert parse_pressure_line(
        "S1 Pressure=1013.250 hPa Temperature=24.400 C\r"
    ) == ("S1", 1013.250, 24.400)
    assert parse_pressure_line(
        "s3 Pressure=1002.001 hPa Temperature=-2.500 C"
    ) == ("S3", 1002.001, -2.500)
    assert parse_pressure_line("S2 communication error: result=-1 count=10") is None
    assert parse_pressure_line("Initialization finished.") is None


def test_rejects_invalid_and_out_of_range_measurements():
    assert parse_pressure_line("S4 Pressure=1013 hPa Temperature=24 C") is None
    assert parse_pressure_line("S1 Pressure=9999 hPa Temperature=24 C") is None
    assert parse_pressure_line("S1 Pressure=1013 hPa Temperature=999 C") is None
