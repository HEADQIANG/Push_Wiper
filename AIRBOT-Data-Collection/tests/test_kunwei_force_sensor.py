import struct
import unittest
from unittest.mock import patch

from airbot_ie.force_control.kunwei import KunweiKwr75Reader
from airbot_ie.force_control.hardware import make_force_reader


def frame(values):
    return b"\x48\xaa" + struct.pack("<6f", *values) + b"\x0d\x0a"


class KunweiProtocolTests(unittest.TestCase):
    def test_decode_little_endian_and_si_conversion(self):
        values = KunweiKwr75Reader.decode_frame(frame((1, -2, 0.5, 0, 0, 3)))
        self.assertAlmostEqual(values[0], 9.81, places=5)
        self.assertAlmostEqual(values[1], -19.62, places=5)
        self.assertAlmostEqual(values[2], 4.905, places=5)
        self.assertAlmostEqual(values[5], 29.43, places=5)

    def test_rejects_bad_frame(self):
        with self.assertRaises(ValueError):
            KunweiKwr75Reader.decode_frame(b"noise")

    @patch("airbot_ie.force_control.hardware.KunweiKwr75Reader")
    def test_factory_defaults_to_kunwei(self, reader):
        make_force_reader({"port": "/dev/test"})
        reader.assert_called_once()

    @patch("airbot_ie.force_control.hardware.LFS6D65Reader")
    def test_factory_can_select_lfs(self, reader):
        make_force_reader({"driver": "lfs6d65", "port": "/dev/test"})
        reader.assert_called_once()


if __name__ == "__main__":
    unittest.main()
