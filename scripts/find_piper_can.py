#!/usr/bin/env python3
"""Read-only lookup of this Piper USB-CAN by actual USB hardware identity."""
from pathlib import Path
import subprocess
import sys


def find_interface():
    matches = []
    for interface in Path('/sys/class/net').iterdir():
        result = subprocess.run(
            ['udevadm', 'info', '-q', 'property', '-p', str(interface)],
            capture_output=True, text=True, check=False)
        properties = dict(line.split('=', 1) for line in result.stdout.splitlines() if '=' in line)
        if (properties.get('ID_VENDOR_ID') == '1d50'
                and properties.get('ID_MODEL_ID') == '606f'
                and properties.get('ID_SERIAL_SHORT') == '001D00354648571720303731'
                and properties.get('ID_NET_DRIVER') == 'gs_usb'):
            matches.append(interface.name)
    if len(matches) != 1:
        raise RuntimeError(f'Piper USB-CAN identity lookup expected one interface, found {matches}')
    return matches[0]


if __name__ == '__main__':
    try:
        print(find_interface())
    except RuntimeError as error:
        print(error, file=sys.stderr)
        sys.exit(1)
