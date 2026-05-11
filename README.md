# XIAO nRF52840 Sense IMU Data Logger

A CircuitPython application for the Seeed Studio XIAO nRF52840 Sense that continuously logs 6-axis IMU data (accelerometer + gyroscope) with BLE UART interface, motion-triggered sleep/wake, and battery monitoring.

## Hardware

- **Microcontroller**: Seeed Studio XIAO nRF52840 Sense
- **IMU Sensor**: LSM6DS3TR-C (6-axis: 3-axis accelerometer + 3-axis gyroscope)
- **Communication**: BLE UART (Nordic UART Service), USB Serial (debug)
- **Power**: LiPo battery with protection circuit
- **Firmware**: CircuitPython 10.2.0

## Features

- Real-time IMU data logging at configurable frequency
- Motion magnitude calculations (accel_mag, gyro_mag)
- BLE UART interface for remote commands
- Motion-triggered light sleep with wake-up confirmation
- Battery voltage and charge status monitoring
- Dynamic IMU frequency and sensitivity adjustment
- Data buffering with auto-save to CSV every N records
- System status reporting (RAM, Flash, battery, file size, IMU config)
- RGB LED status indicators (active-low logic)
- RTC time setting via BLE command

## Installation

### 1. Flash Firmware (CircuitPython or MicroPython)

#### Option A: Using Thonny IDE

1. Install [Thonny](https://thonny.org/)
2. Connect your XIAO nRF52840 via USB
3. Put the device in **bootloader mode**:
   - Press the small button on the device **twice rapidly** (double-click)
   - The device should appear as a USB mass storage device
4. Open Thonny and go to **Tools → Options → Interpreter**
5. Select **MicroPython (nRF52)** or browse for the connected device
6. Click **Install or update firmware**
7. Select the firmware file (CircuitPython .uf2 or MicroPython .hex)
8. Click **Install** and wait for completion

#### Option B: Manual Installation

Download the latest CircuitPython for nRF52840 from [circuitpython.org](https://circuitpython.org/board/seeed_xiao_nrf52840_sense/).

Connect via USB, put the device in bootloader mode (double-press the button), and drag the `.uf2` file to the USB mass storage device. The device will reboot automatically.

For more details, follow the [CircuitPython guide](https://learn.adafruit.com/welcome-to-circuitpython).

### 2. Install Dependencies

Copy `/lib` content to the device.

or

Install `circup` (CircuitPython library manager):
```bash
pip install circup
```

Connect your device via USB, then install required libraries:
```bash
circup install adafruit-circuitpython-ble
circup install adafruit-circuitpython-lsm6ds
circup install adafruit-circuitpython-busdevice
circup install adafruit-circuitpython-register
circup install seeed_xiao_nrf52840
```

### 3. Upload Code

Using `mpremote`:
```bash
pip install mpremote
mpremote cp boot.py code.py :
```

Or use **Thonny IDE**:
1. Install [Thonny](https://thonny.org/)
2. Configure interpreter: MicroPython (nRF52)
3. Right-click files → Save to device

## Configuration

Edit constants in `code.py`:

```python
DEBUG = True                        # Enable debug output
RECORDING_FREQUENCY = 5             # Hz (samples per second)
AUTO_SAVE_RECORDS_INTERVAL = 10000  # Save to file every N records
SAVE_TO_FILE = False                # RAM-only mode (False) or file mode (True)
WAKEUP_REQUIRED_HITS = 10           # Motion hits to wake from sleep
WAKEUP_WINDOW_SECONDS = 10.0        # Time window for motion detection
```

## BLE Commands

Connect via BLE UART (Nordic UART Service) and send commands:

| Command | Args | Description |
|---------|------|-------------|
| `data` | - | Stream CSV data from file |
| `save_buff` | - | Save unsaved records to file |
| `clear_data` | - | Clear data file |
| `sensors` | - | Read CPU temp, voltage, and current IMU values |
| `status` | - | System status (RAM, Flash, battery, IMU config) |
| `sleep` | - | Enter motion-triggered sleep mode |
| `set_freq` | `[1-5]` | Set IMU frequency (1=12.5Hz to 5=208Hz) |
| `set_sens` | `[0-6]` | Set recording sensitivity (0=hair-trigger to 6=max) |
| `set_wakeup_sens` | `[0-6]` | Set wake-up sensitivity for sleep mode |
| `set_time` | `YYYY-MM-DD HH:MM:SS` | Set RTC time |

## Data Format

### CSV Output (imu_data.csv)
```
timestamp,accel_magnitude,gyro_magnitude
123.45,2.34,0.12
124.56,2.56,0.14
```

- **timestamp**: Monotonic time in seconds
- **accel_magnitude**: √(x² + y² + z²) in m/s²
- **gyro_magnitude**: √(x² + y² + z²) in rad/s

### Status Output
```
datetime: 2026-05-10 15:30:45
Battery: 3.85V (Charging)
SRAM: 45678/262144 bytes (44.5/256.0 KB) (17% used)
Flash: 234567/1048576 bytes (229.1/1024.0 KB) (22% used)
Unsaved records: 1234
Record file size: 45678 bytes
IMU Frequency: 0x3 (52 Hz)
Sensitivity: ±4g (0x02)
Wake-up Sensitivity: 40
```

## Power Management

- **Battery protection**: Prevents writes below 3.5V

**To maximize battery life:**
1. Reduce `RECORDING_FREQUENCY`
2. Increase `set_freq` level (lower ODR = lower power)
3. Use `sleep` command for inactive periods
4. Disable `DEBUG` output

## Troubleshooting

### Device Not Detected

```bash
# Check USB connection
ls /dev/tty* | grep -i usb

# Try mpremote list
mpremote list
```
## References

- [Seeed XIAO nRF52840 Docs](https://wiki.seeedstudio.com/XIAO_BLE/)
- [LSM6DS3TR-C Datasheet](https://www.st.com/resource/en/datasheet/lsm6ds3tr-c.pdf)
- [CircuitPython Docs](https://docs.circuitpython.org/)
- [Adafruit LSM6DS Library](https://github.com/adafruit/Adafruit_CircuitPython_LSM6DS)
