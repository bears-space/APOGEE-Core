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
(ADR high), and the output interval (default 100 ms). The application enables
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
prints the original integer counts. Sensor defaults are ±4 g / 62.5 Hz for
acceleration, 10 Hz magnetometer output, and ±2000 degrees/s / 32 Hz for the
gyroscope; faster polling can repeat samples.

For reuse, call `bno055_init(bus, address, &device)` with an existing synchronous
I2C master bus, then `bno055_read_raw(device, &sample)`. Remove the device with
`i2c_master_bus_rm_device(device)` before deleting the bus. Initialization
resets the sensor, including any previous calibration/configuration.

Register layout and timing reference:
[Bosch BNO055 datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf).
