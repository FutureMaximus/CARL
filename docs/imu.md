# BNO085 / BNO080 IMU

CARL uses the [Adafruit BNO085/BNO080 breakout, product 4754](https://www.adafruit.com/product/4754)
over I2C. The BMI088 driver is no longer used. The HAT's existing IMU can remain
on the bus; software only opens the BNO085 address.

## Wiring and installation

With power disconnected, connect the breakout as follows:

| Breakout | Raspberry Pi header |
| --- | --- |
| VIN | 3.3 V, physical pin 1 |
| GND | Ground, physical pin 6 |
| SDA | GPIO2 / SDA1, physical pin 3 |
| SCL | GPIO3 / SCL1, physical pin 5 |

Leave P0 and P1 low (the factory default) for I2C mode. The default address is
`0x4A`; pulling DI high selects `0x4B`. Set `IMU_I2C_ADDRESS` in
`src/constants.py` accordingly. `IMU_I2C_BUS` defaults to bus 1.
See [Adafruit's pinout](https://learn.adafruit.com/adafruit-9-dof-orientation-imu-fusion-breakout-bno085/pinouts).

Enable I2C on the Pi. The repository's `docs/dev/config.txt` selects 400 kHz;
retain that setting for the shared audio-codec bus. Actual I2C timing and the
requested 200 Hz report rate must be checked on the robot.

After copying the updated project to the Pi, run `uv sync` from its project
directory to resolve and install the new Adafruit dependencies. An old image or
`uv sync --frozen` with an old lockfile will not install the changed dependencies.
Then run `uv run src/imu.py` on the Pi, or `make imu` from the development machine.

## Orientation and verification

The reader requests acceleration, calibrated gyro and game rotation vector
reports at 200 Hz. Onboard fusion replaces the old host Madgwick filter. The game
rotation vector avoids magnetometer heading corrections near motors; yaw is
relative and can drift. Acceleration includes gravity and is converted to g,
gyro stays in rad/s, and quaternions are reordered to `(w, x, y, z)` and normalized.
See the [Adafruit API](https://docs.circuitpython.org/projects/bno08x/en/latest/api.html).

The existing `IMU_MOUNT_QUAT` assumes the sensor axes have the original mounting
orientation. Align the new board's sensor axes accordingly, or update that
constant to match its actual mounting. No mechanical bracket change is included.
Before walking, keep motor torque disabled and check `make imu`: the upright
body should show near-zero roll/pitch and projected gravity near `(0, 0, -1)`.
Tilt the body around each axis to verify signs, check acceleration magnitude is
near 1 g at rest, and confirm reads stay valid without increasing errors.

The status timestamp measures host reads, not sensor acquisition: Adafruit's
properties return their latest cached reports. It does not guarantee every read
contains a new report. Read failures preserve the last values and timestamp but
mark the snapshot invalid. Hardware validation is required to establish the
effective report rate and I2C reliability on the Pi.
