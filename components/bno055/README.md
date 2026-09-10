# GY-BNO055

ESP-IDF component using the `esp_driver_i2c` master API. Initialization checks
the chip ID, resets the sensor, and enables AMG (accelerometer, magnetometer,
gyroscope) mode. The component returns signed 16-bit register values, with no
host-side scaling or sensor fusion.

Connect GND to ESP GND, use a module supply compatible with your board, and
connect SDA/SCL to the GPIOs configured in `idf.py menuconfig` under
**Vigilant Engine Configuration: I2C** (current defaults: SDA GPIO 1, SCL GPIO 2).
Use pull-ups to 3.3 V on SDA/SCL and set PS1/PS0 low for I2C operation. Check
your board's pin assignments before wiring. SCL defaults to GPIO 2 for this
application to avoid the existing RGB status LED's GPIO 0 assignment.

Under **GY-BNO055 Configuration**, select address `0x28` (ADR low) or `0x29`
(ADR high), and the output interval (default 10 ms, targeting 100 Hz). The application enables
the existing Vigilant I2C bus automatically.

`main/main.c` prints one CSV sample per line, without labels or timestamps:

```text
ax,ay,az,mx,my,mz,gx,gy,gz
```

The line above describes column order; it is not printed. Startup/framework
logs and read errors still appear on the console. The example telemetry task
is no longer started. No quaternion, Euler angle, or other fusion output is used.

With the initialized units, acceleration has 100 LSB per m/s², magnetic field
has 16 LSB per µT, and angular velocity has 16 LSB per degree/s. The driver
prints the original integer counts. Acceleration retains the ±4 g range and
62.5 Hz filter bandwidth; the gyroscope retains ±2000 degrees/s and 32 Hz
filter bandwidth. These bandwidths are not the host polling rate. The
magnetometer is configured for 30 Hz output in the regular preset and normal
power mode, so magnetic values repeat between updates in the 100 Hz CSV stream.

The polling loop uses a fixed schedule instead of adding a delay after every
read. The actual rate depends on FreeRTOS tick resolution, I2C clock stretching,
and console throughput; missed deadlines are skipped. Existing `sdkconfig`
files retain their saved interval: set it to 10 ms in `menuconfig` when upgrading.

For reuse, call `bno055_init(bus, address, &device)` with an existing synchronous
I2C master bus, then `bno055_read_raw(device, &sample)`. Remove the device with
`i2c_master_bus_rm_device(device)` before deleting the bus. Initialization
resets the sensor, including any previous calibration/configuration.

Register layout and timing reference:
[Bosch BNO055 datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf).
