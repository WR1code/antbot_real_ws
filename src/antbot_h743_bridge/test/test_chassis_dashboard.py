import unittest

from antbot_h743_bridge.chassis_dashboard import (
    FAULT_NAMES,
    TelemetryStore,
    bit_names,
    format_uptime,
    unsigned32,
    version_string,
)
from antbot_h743_bridge.chassis_uart_protocol import Ack


def detail_ack(detail_type, values, valid_mask=0x0F):
    return Ack(
        sequence=1,
        status=0,
        chassis_state=4,
        reject_reason=0,
        fault_flags=0,
        steering_flags=7,
        can_flags=3,
        uart_valid_count=10,
        can_tx_count=20,
        can_rx_count=30,
        stm32_tick=4000,
        steering_position_mrad=(0, 1571, 3142, -32768),
        control_id=1,
        detail_type=detail_type,
        detail_values=values,
        detail_valid_mask=valid_mask,
    )


class DashboardModelTests(unittest.TestCase):
    def test_accumulates_paged_steering_and_drive_feedback(self):
        store = TelemetryStore()
        packed = 0x00_2C_16_1D
        store.apply(detail_ack(8, (33, 0, 0, packed)), now=1.0)
        store.apply(detail_ack(9, (10, 20, 30, 40)), now=1.1)
        store.apply(detail_ack(2, (100, 200, 300, 400)), now=1.2)
        store.apply(detail_ack(3, (50, -25, 0, 100)), now=1.3)
        store.apply(detail_ack(4, (1000, 2000, 3000, 4000)), now=1.4)
        store.apply(detail_ack(5, (31, 32, 33, 34)), now=1.5)
        store.apply(detail_ack(6, (0, 1, 2, 4)), now=1.6)
        store.apply(detail_ack(7, (31, 31, 31, 31)), now=1.7)
        store.apply(detail_ack(17, (11, 12, 13, 14)), now=1.8)
        store.apply(detail_ack(18, (0, 2, 0, 16)), now=1.9)
        store.apply(detail_ack(19, (24, 24, 23, 25)), now=2.0)

        steering = store.steering_rows()
        self.assertAlmostEqual(steering[1][1], 90.01, places=1)
        self.assertEqual(steering[2][2], 30)
        self.assertTrue(steering[0][4])
        self.assertTrue(steering[0][5])
        self.assertTrue(steering[0][6])
        self.assertTrue(steering[2][7])
        self.assertIsNone(steering[3][1])

        drive = store.drive_rows()
        self.assertEqual(drive[0][1], 100)
        self.assertEqual(drive[0][2], 0.5)
        self.assertEqual(drive[1][2], -0.25)
        self.assertEqual(drive[3][3], 40.0)
        self.assertEqual(drive[2][4], 33)
        self.assertEqual(drive[0][5], 24)
        self.assertEqual(drive[1][6], 12)
        self.assertEqual(drive[3][7], 4)
        self.assertEqual(drive[3][8], 16)

    def test_invalid_detail_mask_is_shown_as_missing(self):
        store = TelemetryStore()
        store.apply(detail_ack(2, (1, 2, 3, 4), valid_mask=0x05))
        values = store.detail_values(2)
        self.assertEqual(values, [1.0, None, 3.0, None])

    def test_helpers(self):
        self.assertEqual(format_uptime(90_061_000), "1天 01:01:01")
        self.assertEqual(bit_names(0, FAULT_NAMES), "无")
        self.assertIn("CAN Bus-Off", bit_names(1 << 3, FAULT_NAMES))
        self.assertEqual(unsigned32(-1), 0xFFFFFFFF)
        self.assertEqual(version_string(0x00010203), "1.2.3")

    def test_accumulates_system_health_pages(self):
        store = TelemetryStore()
        store.apply(detail_ack(10, (7, -1, 2, 3)))
        store.apply(detail_ack(11, (10, 30, 15, 2048)))
        store.apply(detail_ack(12, (0x00010000, 0x12345678, -1, 0x3F)))
        self.assertEqual(store.details[10].detail_values[0], 7)
        self.assertEqual(store.details[11].detail_values[3], 2048)
        self.assertEqual(unsigned32(store.details[12].detail_values[2]), 0xFFFFFFFF)


if __name__ == "__main__":
    unittest.main()
