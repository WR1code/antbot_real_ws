import unittest

from antbot_h743_bridge.chassis_uart_protocol import (
    Ack,
    AckStreamParser,
    CONTROL_IDS,
    SequenceCounter,
    crc16_ccitt,
    decode_ack,
    encode_ack,
    encode_cmd_vel,
    encode_control,
    limit_linear_velocity,
)


class ProtocolTests(unittest.TestCase):
    def test_command_vector_and_length(self):
        frame = encode_cmd_vel(0x2A, 0.05, -0.02, 0.0)
        self.assertEqual(len(frame), 12)
        self.assertEqual(frame.hex(), "aa55012a3200ecff000036a9")

    def test_crc_known_vector(self):
        self.assertEqual(crc16_ccitt(b"123456789"), 0x29B1)

    def test_signed_and_zero_values(self):
        self.assertNotEqual(encode_cmd_vel(1, 0.1, 0, 0), encode_cmd_vel(1, -0.1, 0, 0))
        self.assertEqual(encode_cmd_vel(1, 0, 0, 0)[4:10], bytes(6))

    def test_wire_range_and_linear_limit(self):
        with self.assertRaises(ValueError):
            encode_cmd_vel(0, 33.0, 0, 0)
        vx, vy = limit_linear_velocity(3.0, 4.0, 0.5)
        self.assertAlmostEqual(vx, 0.3)
        self.assertAlmostEqual(vy, 0.4)

    def test_nonzero_angular_is_encoded_for_firmware_rejection(self):
        self.assertEqual(encode_cmd_vel(0, 0, 0, 0.1)[8:10], b"\x64\x00")

    def test_sequence_wrap(self):
        sequence = SequenceCounter(255)
        self.assertEqual((sequence.take(), sequence.take()), (255, 0))

    def test_ack_round_trip_and_crc_failure(self):
        ack = Ack(7, 9, 5, 0, 0x1234, 7, 3, 10, 20, 30, 123456,
                  (100, 200, 300, 400))
        frame = encode_ack(ack)
        self.assertEqual(len(frame), 52)
        self.assertEqual(decode_ack(frame), ack)
        damaged = bytearray(frame)
        damaged[10] ^= 1
        with self.assertRaisesRegex(ValueError, "CRC"):
            decode_ack(bytes(damaged))

    def test_ack_stream_split_sticky_and_resync(self):
        first = encode_ack(Ack(1, 0, 4, 0, 0, 0, 0, 1, 2, 3, 4))
        second = encode_ack(Ack(2, 10, 6, 0, 0, 7, 3, 2, 4, 6, 8))
        parser = AckStreamParser()
        self.assertEqual(parser.feed(b"bad" + first[:8]), [])
        output = parser.feed(first[8:] + second)
        self.assertEqual([item.sequence for item in output], [1, 2])

    def test_crc_error_recovers_next_ack(self):
        bad = bytearray(encode_ack(Ack(1, 0, 4, 0, 0, 0, 0, 1, 2, 3, 4)))
        bad[5] ^= 1
        good = encode_ack(Ack(2, 0, 4, 0, 0, 0, 0, 2, 3, 4, 5))
        parser = AckStreamParser()
        output = parser.feed(bytes(bad) + good)
        self.assertEqual([item.sequence for item in output], [2])
        self.assertEqual(parser.crc_errors, 1)

    def test_control_frame(self):
        frame = encode_control(0x12, 7, b"\x01\x02\x03\x04")
        self.assertEqual(len(frame), 16)
        self.assertEqual(frame[:6], b"\xAA\x55\x02\x12\x07\x04")
        self.assertEqual(crc16_ccitt(frame[:14]), int.from_bytes(frame[14:], "little"))

    def test_system_reset_control_frame_requires_magic_payload(self):
        frame = encode_control(CONTROL_IDS["SYSTEM_RESET"], 8, b"RST!")
        self.assertEqual(frame[3:10], b"\x32\x08\x04RST!")
        self.assertEqual(crc16_ccitt(frame[:14]), int.from_bytes(frame[14:], "little"))

    def test_system_health_command_ids_and_event_page(self):
        self.assertEqual(CONTROL_IDS["QUERY_SYSTEM_BOOT"], 0x02)
        self.assertEqual(CONTROL_IDS["QUERY_POWER"], 0x08)
        frame = encode_control(CONTROL_IDS["QUERY_EVENT_LOG"], 9, b"\x1f")
        self.assertEqual(frame[3:7], b"\x07\x09\x01\x1f")

    def test_system_health_ack_detail_round_trip(self):
        ack = Ack(
            1, 0, 4, 0, 0, 7, 3, 1, 2, 3, 4,
            control_id=CONTROL_IDS["QUERY_SYSTEM_BOOT"],
            detail_type=10,
            detail_values=(7, -1, 2, 3),
            detail_valid_mask=0x0F,
        )
        self.assertEqual(decode_ack(encode_ack(ack)), ack)


if __name__ == "__main__":
    unittest.main()
