"""Hardware-independent tests: python -m unittest discover -s tests."""

import math
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import MagicMock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from imu_reader import ThreadedIMUReader


class IMUReaderTests(unittest.TestCase):
    def setUp(self):
        self.bus = MagicMock()
        self.sensor = MagicMock()
        self.sensor.game_quaternion = (0.0, 0.0, 2.0, 2.0)
        self.sensor.gyro = (0.1, -0.2, 0.3)
        self.sensor.acceleration = (9.80665, -9.80665, 0.0)
        extended = types.ModuleType("adafruit_extended_bus")
        extended.ExtendedI2C = MagicMock(return_value=self.bus)
        bno = types.ModuleType("adafruit_bno08x")
        bno.BNO_REPORT_ACCELEROMETER = 1
        bno.BNO_REPORT_GYROSCOPE = 2
        bno.BNO_REPORT_GAME_ROTATION_VECTOR = 8
        i2c = types.ModuleType("adafruit_bno08x.i2c")
        i2c.BNO08X_I2C = MagicMock(return_value=self.sensor)
        self.extended, self.i2c = extended, i2c
        modules = patch.dict(sys.modules, {
            extended.__name__: extended, bno.__name__: bno, i2c.__name__: i2c,
        })
        modules.start()
        self.addCleanup(modules.stop)

    def reader(self, **kwargs):
        reader = ThreadedIMUReader(i2c_bus=1, **kwargs)
        self.addCleanup(reader.stop)
        return reader

    def tick(self, reader):
        # Run exactly one iteration without sleeping or starting a real thread.
        with patch.object(reader._stop_event, "is_set", side_effect=[False, True]), \
             patch.object(reader._stop_event, "wait"), patch("builtins.print"):
            reader._run_loop()

    def test_reports_address_and_units(self):
        reader = self.reader(address=0x4B)
        self.extended.ExtendedI2C.assert_called_once_with(1)
        self.i2c.BNO08X_I2C.assert_called_once_with(self.bus, address=0x4B)
        self.assertEqual([c.args[0] for c in self.sensor.enable_feature.call_args_list], [1, 2, 8])
        for call in self.sensor.enable_feature.call_args_list:
            self.assertEqual(call.kwargs, {"report_interval": 5000})
        self.assertFalse(reader.get_latest().valid)
        self.tick(reader)
        snap = reader.get_latest()
        self.assertTrue(snap.valid)
        self.assertEqual(snap.acc, (1.0, -1.0, 0.0))
        self.assertEqual(snap.gyro, (0.1, -0.2, 0.3))
        for actual, expected in zip(snap.quat, (math.sqrt(0.5), 0, 0, math.sqrt(0.5))):
            self.assertAlmostEqual(actual, expected)

    def test_failed_read_preserves_sample_and_recovers(self):
        reader = self.reader()
        self.tick(reader)
        good = reader.get_latest()
        self.sensor.game_quaternion = None
        self.tick(reader)
        bad = reader.get_latest()
        self.assertFalse(bad.valid)
        self.assertEqual(bad.timestamp_s, good.timestamp_s)
        self.assertEqual(bad.quat, good.quat)
        self.assertEqual(bad.error_count, 1)
        self.sensor.game_quaternion = (0, 0, 0, 1)
        self.tick(reader)
        self.assertTrue(reader.get_latest().valid)
        self.assertEqual(reader.get_latest().error_count, 1)

    def test_invalid_quaternions(self):
        reader = self.reader()
        for q in [(0, 0, 0, 0), (float("nan"), 0, 0, 1)]:
            self.sensor.game_quaternion = q
            self.tick(reader)
            self.assertFalse(reader.get_latest().valid)

    def test_initialization_failure_closes_bus(self):
        self.sensor.enable_feature.side_effect = OSError("I2C failure")
        with self.assertRaises(OSError):
            ThreadedIMUReader(i2c_bus=1)
        self.bus.deinit.assert_called_once()

    def test_stop_closes_bus_once(self):
        reader = self.reader()
        reader.stop()
        reader.stop()
        self.bus.deinit.assert_called_once()
        with self.assertRaises(RuntimeError):
            reader.start()

    def test_invalid_frequency_does_not_open_bus(self):
        for hz in [0, -1, float("nan"), float("inf"), 1e10]:
            with self.assertRaises(ValueError):
                ThreadedIMUReader(i2c_bus=1, frequency_hz=hz)
        self.extended.ExtendedI2C.assert_not_called()


if __name__ == "__main__":
    unittest.main()
