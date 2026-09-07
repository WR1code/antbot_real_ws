import pathlib
import tempfile
import unittest

from antbot_h743_bridge.serial_port import PREFERRED_PORT, resolve_uart_port


class SerialPortTests(unittest.TestCase):
    def test_explicit_and_environment_override_discovery(self):
        self.assertEqual(
            resolve_uart_port("/dev/test-explicit", environ={}),
            "/dev/test-explicit",
        )
        self.assertEqual(
            resolve_uart_port(environ={"RS00_UART_PORT": "/dev/test-env"}),
            "/dev/test-env",
        )

    def test_discovers_only_known_single_serial_pattern(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            (root / "usb-HDSC_CDC_Device_123-if00").touch()
            expected = root / "usb-1a86_USB_Single_Serial_NEW-if00"
            expected.touch()
            self.assertEqual(
                resolve_uart_port(
                    environ={}, by_id_dir=root,
                    preferred_port=str(root / "preferred"),
                ),
                str(expected),
            )

    def test_does_not_guess_unrelated_or_ambiguous_devices(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            (root / "usb-HDSC_CDC_Device_123-if00").touch()
            self.assertEqual(
                resolve_uart_port(
                    environ={}, by_id_dir=root,
                    preferred_port=PREFERRED_PORT,
                ),
                PREFERRED_PORT,
            )
            (root / "usb-1a86_USB_Single_Serial_A-if00").touch()
            (root / "usb-1a86_USB_Single_Serial_B-if00").touch()
            with self.assertRaisesRegex(RuntimeError, "多个"):
                resolve_uart_port(
                    environ={}, by_id_dir=root,
                    preferred_port=str(root / "preferred"),
                )


if __name__ == "__main__":
    unittest.main()
