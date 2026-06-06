"""
IMU Data Logger for Seeed Studio XIAO nRF52840 Sense

Continuously records accelerometer and gyroscope data at 5 Hz.
Data is stored in RAM and auto-saved to CSV file every 10 records.
Provides BLE UART interface for real-time commands and data retrieval.
"""

import microcontroller
import board
import digitalio

from adafruit_ble import BLERadio
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.services.nordic import UARTService
from adafruit_bus_device.i2c_device import I2CDevice

import time
import busio
import os
import alarm
import rtc
import struct
import gc
import storage
import json
from adafruit_lsm6ds import Rate, AccelRange, GyroRange
from adafruit_lsm6ds.lsm6ds3trc import LSM6DS3TRC
from seeed_xiao_nrf52840 import Battery


# LSM6DS3TR-C Wake-up Threshold Levels (Register 0x5B)
# Register 0x5B uses 6 bits for the threshold.
# Values range from 0x00 (0 decimal) to 0x3F (63 decimal).
# 1 LSB = FS / 64. At 2g range, 1 LSB = ~31.25mg
IMU_SENSITIVITY_LEVEL_0 = 0x00 # (0)  Hair-trigger (Might never sleep, triggers on noise)
IMU_SENSITIVITY_LEVEL_1 = 0x0A # (10) Balanced (Standard "picked up" or "tilted")
IMU_SENSITIVITY_LEVEL_2 = 0x14 # (20) Low (A solid "bump" or "shove")
IMU_SENSITIVITY_LEVEL_3 = 0x1E # (30) Tough (A hard "tap" or "thump")
IMU_SENSITIVITY_LEVEL_4 = 0x28 # (40) Very Tough (Requires a deliberate shake)
IMU_SENSITIVITY_LEVEL_5 = 0x32 # (50) Extreme (A light drop or impact)
IMU_SENSITIVITY_LEVEL_6 = 0x3C # (60) Maximum (Hard impact only)
IMU_SENSITIVITY_LEVELS = [
    IMU_SENSITIVITY_LEVEL_0,
    IMU_SENSITIVITY_LEVEL_1,
    IMU_SENSITIVITY_LEVEL_2,
    IMU_SENSITIVITY_LEVEL_3,
    IMU_SENSITIVITY_LEVEL_4,
    IMU_SENSITIVITY_LEVEL_5,
    IMU_SENSITIVITY_LEVEL_6
]

# --- IMU FREQUENCY LEVELS ---
# Always keep the IMU hardware frequency at least 2x to 4x faster than your code's sampling frequency.
IMU_FREQUENCY_LEVEL_1 = 0x10  # 12.5 Hz (Lowest power; slow response)
IMU_FREQUENCY_LEVEL_2 = 0x20  # 26 Hz
IMU_FREQUENCY_LEVEL_3 = 0x30  # 52 Hz (Good for N-hit logic)
IMU_FREQUENCY_LEVEL_4 = 0x40  # 104 Hz (Standard; very responsive)
IMU_FREQUENCY_LEVEL_5 = 0x50  # 208 Hz (High performance)
IMU_FREQUENCY_LEVELS = [
    IMU_FREQUENCY_LEVEL_1, 
    IMU_FREQUENCY_LEVEL_2, 
    IMU_FREQUENCY_LEVEL_3, 
    IMU_FREQUENCY_LEVEL_4, 
    IMU_FREQUENCY_LEVEL_5
]

# --- Config ---
DEBUG = True                                            # Set to True to enable print statements

RECORDING_FREQUENCY         = 5                         # Hz. Rate to sample IMU data.
AUTO_SAVE_RECORDS_INTERVAL  = 100                       # Save to file every N records.
BINARY_FILE                 = "imu_data.bin"            # Binary file for optimized storage (not human-readable)
MARKS_FILE                  = "marks.csv"               # CSV file for timestamped marks/annotations
SAVE_TO_DISK                = True                      # Set to True to save records to disk
IMU_DEFAULT_SENSITIVITY     = IMU_SENSITIVITY_LEVEL_1   # Default sensitivity for general motion detection
IMU_DEFAULT_FREQUENCY       = IMU_FREQUENCY_LEVEL_3     # Default frequency for wake-up detection
WAKEUP_REQUIRED_HITS        = 10                        # 'n' values: Number of triggers needed to fully wake up
WAKEUP_WINDOW_SECONDS       = 10.0                      # Period to detect those 'n' hits
WAKEUP_IMU_SENSITIVITY      = IMU_SENSITIVITY_LEVEL_4   # Sensitivity level for wake-up detection
IMU_SLEEP_FREQUENCY         = IMU_FREQUENCY_LEVEL_2     # Lower frequency during sleep to save power
EMIT_VALUES                 = False                     # Set to True to emit sensor values using BLE, False otherwise

READ_ACCELEROMETER          = True                      # Whether to read accelerometer magnitude values
READ_GYROSCOPE              = False                     # Whether to read gyroscope magnitude values
READ_BATTERY                = False                     # Whether to read battery voltage values

BATTERY_SAFETY_THRESHOLD    = 3.3                       # Batterry voltage threshold to consider the device safe to operate
MIN_DISK_BUFFER_BYTES       = 1024 * 5                  # Min free disk space to keep

# --- High-Resolution Timer Configuration ---
# Hardware monotonic clock for microsecond-precision timestamps (always enabled)
TIMER_CALIBRATION_INTERVAL  = 3600                      # Recalibrate monotonic vs RTC every N seconds (1 hour)

# --- Dynamic Binary Formatting ---
# Calculate expected byte size based on active flags (4 bytes per float32)
RECORD_PACK_FORMAT = '<f'  # Always include timestamp
if READ_ACCELEROMETER:
    RECORD_PACK_FORMAT += 'f'
if READ_GYROSCOPE:
    RECORD_PACK_FORMAT += 'f'
if READ_BATTERY:
    RECORD_PACK_FORMAT += 'f'

RECORD_SIZE = struct.calcsize(RECORD_PACK_FORMAT)

# --- LEDS ---
# LED logic is inverted on this board (True = off, False = on)
LED_RED     = digitalio.DigitalInOut(board.LED_RED)
LED_GREEN   = digitalio.DigitalInOut(board.LED_GREEN)
LED_BLUE    = digitalio.DigitalInOut(board.LED_BLUE)

LED_RED.direction   = digitalio.Direction.OUTPUT
LED_GREEN.direction = digitalio.Direction.OUTPUT
LED_BLUE.direction  = digitalio.Direction.OUTPUT

# Start with all LEDs off
LED_BLUE.value  = True  
LED_GREEN.value = True
LED_RED.value   = True

# --- IMU Setup ---

# On the Seeed XIAO Sense the LSM6DS3TR-C IMU is connected on a separate
# I2C bus and it has its own power pin that we need to enable.
imupwr           = digitalio.DigitalInOut(board.IMU_PWR)
imupwr.direction = digitalio.Direction.OUTPUT
imupwr.value     = True

time.sleep(0.1) # Allow sensor to power on

imu_i2c         = busio.I2C(board.IMU_SCL, board.IMU_SDA)
imu_device      = I2CDevice(imu_i2c, 0x6A)
sensor          = LSM6DS3TRC(imu_i2c)

# Turn off the gyroscope completely
if not READ_GYROSCOPE:
    sensor.gyro_data_rate = 0  # 0 Hz completely powers down the gyro circuitry

if not READ_ACCELEROMETER:
    sensor.accelerometer_data_rate = 0  # 0 Hz completely powers down the accelerometer circuitry

# --- Globals ---

ble = BLERadio()
uart_server = UARTService()

current_save_to_disk            = SAVE_TO_DISK
current_wakeup_sensitivity      = WAKEUP_IMU_SENSITIVITY    # Track current wake-up sensitivity level
current_recording_frequency     = RECORDING_FREQUENCY       # Track current recording frequency in Hz
current_imu_frequency_level     = IMU_DEFAULT_FREQUENCY     # Track current IMU frequency level
current_imu_sensitivity_level   = IMU_DEFAULT_SENSITIVITY   # Track current IMU sensitivity level
current_emit_values             = False                     # Whether to emit sensor values in BLE status command
last_imu_text                   = ""                        # Last IMU reading as text for broadcasting


# --- Functions ---

# ============================================================================
# IMU Register Operations (Low-Level I2C)
# ============================================================================

def write_reg(register, value):
    """Helper to write to LSM6DS3TR-C registers (Default Address 0x6A)"""
    while not imu_i2c.try_lock():
        pass
    imu_i2c.writeto(0x6A, bytes([register, value]))
    imu_i2c.unlock()

def read_reg(reg):
    """Reads a single byte from a specific register."""
    result = bytearray(1)
    with imu_device as device:
        # Write register address, then read 1 byte
        device.write_then_readinto(bytes([reg]), result)
    return result[0]


# ============================================================================
# IMU Data & Sensor Configuration
# ============================================================================

def read_imu():
    """Read the IMU data and compute magnitudes, plus battery voltage if enabled."""
    accel_mag = 0.0
    if READ_ACCELEROMETER:
        accel_x, accel_y, accel_z = sensor.acceleration
        accel_mag = (accel_x**2 + accel_y**2 + accel_z**2)**0.5
    
    gyro_mag = 0.0
    if READ_GYROSCOPE:
        gyro_x, gyro_y, gyro_z = sensor.gyro
        gyro_mag = (gyro_x**2 + gyro_y**2 + gyro_z**2)**0.5
    
    battery_voltage = 0.0
    if READ_BATTERY:
        try:
            with Battery() as bat:
                battery_voltage = bat.voltage
        except Exception as e:
            DEBUG and print(f"Error reading battery: {e}")
            battery_voltage = 0.0
    
    return {
        "accel_mag": accel_mag,
        "gyro_mag": gyro_mag,
        "battery_voltage": battery_voltage,
    }

def read_imu_status():
    # Read the two core registers
    reg_10 = read_reg(0x10)  # Frequency and General Sensitivity
    reg_5B = read_reg(0x5B)  # Wake-up Threshold

    # 1. Frequency (Top 4 bits of 0x10)
    freq_val = (reg_10 >> 4)
    
    # 2. General Sensitivity / Scale (Bits 3 and 2 of 0x10)
    # 00 = 2g, 01 = 16g, 10 = 4g, 11 = 8g
    scale_bits = (reg_10 >> 2) & 0x03
    scales = {0: "±2g", 1: "±16g", 2: "±4g", 3: "±8g"}
    current_scale = scales.get(scale_bits, "Unknown")

    # 3. Wake-up Sensitivity (Register 0x5B)
    wake_sens = reg_5B & 0x3F # Mask to 6 bits

    return {
        "frequency_bits": freq_val,
        "sensitivity_bits": scale_bits,
        "sensitivity_scale": current_scale,
        "wake_sensitivity": wake_sens,
    }

def set_imu_wakeup_motion_detection(sensitivity_level=WAKEUP_IMU_SENSITIVITY, frequency_level=IMU_SLEEP_FREQUENCY):
    """
    Configure LSM6DS3TR-C hardware wake-up engine via direct register writes.
    This allows motion detection even in deep sleep mode.
    """
    try:
        set_imu_frequency(frequency_level)
        set_imu_sensitivity(sensitivity_level)  
        
        # Register 0x5E (MD1_CFG): Route wake-up event to INT1 pin (0x20 = INTERRUPTS_ENABLE)
        write_reg(0x5E, 0x20)
            
        # Register 0x58 (TAP_CFG): 
        # Current: 0x01 (LIR only)
        # Correct: 0x81 (INTERRUPTS_ENABLE + LIR)
        write_reg(0x5B, 0x81) 
        
        DEBUG and print("IMU hardware registers configured for motion wakeup.")
        return True
    except Exception as e:
        DEBUG and print(f"Hardware config failed: {e}")
        return False

def set_imu_sensitivity(sensitivity_level):
    """
    Use predefined constants: SENSITIVITY_LEVEL_1, SENSITIVITY_LEVEL_2, etc.)
    """
    write_reg(0x10, sensitivity_level)

def set_imu_frequency(frequency_level):
    """
    Use predefined constants: FREQUENCY_LEVEL_1, FREQUENCY_LEVEL_2, etc.)
    """
    write_reg(0x10, frequency_level)

def clear_imu_interrupt():
    """Reads the wake-up source register to reset the INT1 pin."""
    try:
        # Resetthe pin for the next trigger.
        # Reading 0x1B (WAKE_UP_SRC) clears the latched wake-up interrupt.
        # This allows the INT1 pin to return to LOW
        read_reg(0x1B) 
        DEBUG and print("Interrupt cleared.")
    except Exception as e:
        DEBUG and print(f"Failed to clear interrupt: {e}")


# ============================================================================
# LED Control
# ============================================================================

def blink_led(led, num_blinks, delay=0.2):
    """
    Blink the built-in LED a specified number of times.
    Args:
        led (DigitalInOut): The LED to blink (e.g., LED_RED, LED_GREEN, LED_BLUE)
        num_blinks (int): Number of times to blink the LED
        delay (float): Delay in seconds between LED on and off states
    """
    if led is None:
        DEBUG and print("LED not available")
        return
    
    try:
        for _ in range(num_blinks):
            led.value = False # LED on
            time.sleep(delay)
            led.value = True  # LED off
            time.sleep(delay)
    except Exception as e:
        DEBUG and print(f"Error blinking LED: {e}")


# ============================================================================
# System Information & Monitoring
# ============================================================================

def get_free_space_bytes():
    """Returns the free space in bytes on the filesystem."""
    s = os.statvfs('/')
    return s[0] * s[3]

def get_ram_info():
    """Returns total and free SRAM in bytes."""
    gc.collect()
    free = gc.mem_free()
    alloc = gc.mem_alloc()
    total = free + alloc
    return total, free

def get_flash_info():
    """Returns total and free Internal Flash (QSPI) in bytes."""
    try:
        s = os.statvfs('/')
        block_size = s[0]
        total_blocks = s[2]
        free_blocks = s[3]
        total_bytes = block_size * total_blocks
        free_bytes = block_size * free_blocks
        return total_bytes, free_bytes
    except Exception as e:
        print(f"Error getting flash info: {e}")
        return 0, 0

def is_battery_safe():
    """
    Checks if voltage is high enough for safe flash writing.
    3.5V is a safe 'low battery' threshold to prevent corruption.
    """
    try:
        with Battery() as bat:
                voltage = bat.voltage
                return voltage > BATTERY_SAFETY_THRESHOLD
    except Exception as e:
        DEBUG and print(f"Error reading battery: {e}")
        return False

def get_formatted_time():
    """Returns the current date and time as a formatted string."""
    current_time = time.localtime()
    return "{:04d}-{:02d}-{:02d} {:02d}:{:02d}:{:02d}".format(
        current_time[0], current_time[1], current_time[2],
        current_time[3], current_time[4], current_time[5]
    )

def get_config():
    # Get current date and time
    date_time_str = get_formatted_time()
    
    imu_status = read_imu_status()  # Read IMU registers for debug output

    response = {
        "datetime": date_time_str,
        "imu": {
            "frequency_bits": imu_status['frequency_bits'],
            "frequencency_level": current_imu_frequency_level,
            "sensitivity_scale": imu_status['sensitivity_scale'],
            "sensitivity_level": current_imu_sensitivity_level,
            "sensitivity_bits": imu_status['sensitivity_bits'],
            "wake_sensitivity": imu_status['wake_sensitivity'],
            "read_accel": READ_ACCELEROMETER,
            "read_gyro": READ_GYROSCOPE,
        },
        "save_to_disk": current_save_to_disk,
        "emit_values": current_emit_values,
        "wakeup_sensitivity": current_wakeup_sensitivity,
        "recording_frequency": current_recording_frequency
    }

    return json.dumps(response)


def get_status():
    total_ram, free_ram = get_ram_info()
    used_ram = total_ram - free_ram
    total_flash, free_flash = get_flash_info()
    used_flash = total_flash - free_flash
    
    # Get current date and time
    date_time_str = get_formatted_time()
    
    # Get battery status
    battery_voltage = 0.0
    vbatt = 0
    charge_status = "Unknown"
    try:
        with Battery() as bat:
            battery_voltage = bat.voltage
            vbatt = bat.vbatt
            charge_status = "Charged" if bat.charge_status else "Charging"
    except Exception as e:
        DEBUG and print(f"Error reading battery: {e}")
    
    response = {
        "datetime": date_time_str,
        "battery": {
            "voltage": round(battery_voltage, 2),
            "vbatt": vbatt,
            "charge_status": charge_status,
            "read_battery": READ_BATTERY
        },
        "ram": {
            "used_bytes": used_ram,
            "total_bytes": total_ram,
        },
        "flash": {
            "used_bytes": used_flash,
            "total_bytes": total_flash,
        },
    }

    return json.dumps(response)


# ============================================================================
# Data Storage & File Operations
# ============================================================================

def has_enough_space_for_record(data_length_bytes):
    """
    Check if we have enough free space to save a new record, including buffer.
    Args:
        data_length_bytes (int): The length of the data to be saved in bytes.
    
    Returns:
        bool: True if there is enough space, False otherwise.
    """
    available_bytes = get_free_space_bytes()
    if available_bytes - data_length_bytes < MIN_DISK_BUFFER_BYTES:
        print(f"Warning: Low disk space. Available: {available_bytes} bytes")
        return False
    return True

def clear_datafile():
    """Clear binary data file by truncating."""
    try:
        with open(BINARY_FILE, 'wb') as f:
            pass  # Opening in write mode truncates the file
        DEBUG and print(f"Cleared binary data file: {BINARY_FILE}")
        return True
    except OSError as e:
        DEBUG and print(f"Error clearing data file: {e}")
        return False

def save_to_disk(records, binary_file):
    """
    Save records to optimized binary format for compact storage.
    """
    if not current_save_to_disk or not records or not is_battery_safe():
        if not is_battery_safe():
            DEBUG and print("Battery voltage too low for safe binary file writing.")
            blink_led(LED_RED, 3, 0.1)
        return False
    
    try:
        # Calculate binary size dynamically
        binary_size = len(records) * RECORD_SIZE
        
        # Check if we have enough space
        if not has_enough_space_for_record(binary_size):
            DEBUG and print(f"Warning: Insufficient disk space for binary save. Available: {get_free_space_bytes()} bytes")
            return False
        
        # Open binary file in append mode
        with open(binary_file, "ab") as f:
            for binary_data in records:
                try:
                    # Validate dynamically
                    if isinstance(binary_data, bytes) and len(binary_data) == RECORD_SIZE:
                        f.write(binary_data)
                    else:
                        DEBUG and print(f"Error: Invalid record format (expected {RECORD_SIZE}-byte binary, got {len(binary_data)} bytes)")
                except Exception as e:
                    DEBUG and print(f"Error writing binary record: {e}")
                    continue
        
        DEBUG and print(f"Saved {len(records)} binary records to {binary_file} ({binary_size} bytes written - {get_free_space_bytes()} bytes free)")
        return True
        
    except OSError as e:
        if e.errno == 30:  # EROFS - Read-only file system
            DEBUG and print(f"Warning: Filesystem is read-only. Data kept in RAM only.")
        else:
            DEBUG and print(f"Error saving binary file: {e}")
        return False


def download_bin():
    
    DEBUG and print(f"Start uploading sensor data")
    
    """ Streams the optimized binary file over BLE with dynamic MTU sizing """
    global current_emit_values
    
    # 1. Pause live emissions so they don't corrupt the raw binary stream
    was_emitting = current_emit_values
    current_emit_values = False 
    
    # 2. Save any pending RAM data to flash
    if unsaved_records:
        save_to_disk(unsaved_records, BINARY_FILE)
        unsaved_records.clear()
        
    # 3. Determine exact file size
    try:
        stat_result = os.stat(BINARY_FILE)
        file_size = stat_result[6]
    except OSError:
        file_size = 0
        
    # 4. Send protocol header so desktop knows how many bytes to read
    header = f"START_BIN:{file_size}\n"
    uart_server.write(header.encode())
    
    # Small yield to ensure the header transmits cleanly before the binary flood
    time.sleep(0.05) 
    
    # 5. Stream raw binary data in chunks using dynamic MTU
    if file_size > 0:
        try:
            # Dynamically get the max packet length negotiated with the desktop
            try:
                # ble.connections[0] gets the active BLE connection
                connection = ble.connections[0]
                # adafruit_ble UART handles some overhead, so we subtract 4 for safety
                chunk_size = connection.max_packet_length - 4 
            except Exception:
                chunk_size = 120 # Safe fallback if MTU cannot be determined

            if DEBUG:
                print(f"Streaming {file_size} bytes with MTU chunk size: {chunk_size}")
            
            with open(BINARY_FILE, "rb") as f:
                while True:
                    chunk = f.read(chunk_size) 
                    if not chunk:
                        break
                    
                    # Write the chunk directly. 
                    # No time.sleep() needed here; adafruit_ble will block automatically 
                    # if the hardware buffer is temporarily full.
                    uart_server.write(chunk)
                    
                    DEBUG and print(f"Sent chunck...")
                    
        except OSError as e:
            if DEBUG:
                print(f"Error reading bin: {e}")
                
    # Restore live emissions if they were active
    current_emit_values = was_emitting
    DEBUG and print(f"File uploaded.")
    return "" # Return empty so we don't accidentally send a trailing \r\n

# ============================================================================
# Power Management
# ============================================================================

def enter_sleep_mode():
    """Puts the device into light sleep mode and configures wake-up on motion detection."""
    set_imu_wakeup_motion_detection(sensitivity_level=current_wakeup_sensitivity)
    while True:
        hits = 0
        start_time = None
        
        DEBUG and print(f"Waiting for {WAKEUP_REQUIRED_HITS} motion events...")

        while hits < WAKEUP_REQUIRED_HITS:
            # 1. Clear latch so the pin can transition again
            clear_imu_interrupt()

            # 2. Power off flash to save energy during sleep
            poweroff_flash()
            
            # 3. Sleep until the NEXT motion event
            motion_alarm = alarm.pin.PinAlarm(pin=board.IMU_INT1, value=True)
            alarm.light_sleep_until_alarms(motion_alarm)
            
            # 4. Handle the hit
            now = time.monotonic()
            if hits == 0:
                # This is the first hit, start the timer window
                start_time = now
                hits = 1
                DEBUG and print("Hit 1/{} detected. Timer started.".format(WAKEUP_REQUIRED_HITS))
            else:
                # Check if we are still within the time window
                if now - start_time <= WAKEUP_WINDOW_SECONDS:
                    hits += 1
                    DEBUG and print("Hit {}/{} detected.".format(hits, WAKEUP_REQUIRED_HITS))
                else:
                    # Window expired; reset and treat this as the new "first" hit
                    print("Window expired. Resetting count.")
                    start_time = now
                    hits = 1
        
        # If we exit the inner while loop, we hit our 'N' count
        DEBUG and print("MOTION CONFIRMED: {} hits in {}s".format(hits, WAKEUP_WINDOW_SECONDS))
        DEBUG and blink_led(LED_BLUE, 3, 0.1)  # Indicate wake-up with LED pattern
        
        clear_imu_interrupt()
        set_imu_frequency(current_imu_frequency_level)        # Restore normal frequency
        set_imu_sensitivity(current_imu_sensitivity_level)    # Restore normal sensitivity

        break # Fully wake up and continue code.py

def poweroff_flash():
    """
    Manages the SPI Flash chip power state.
    Note: 
    - CircuitPython 10.x requires all files to be closed for the flash to actually enter low-power mode during sleep.
    - To "Wake" the flash, the system usually handles this when you attempt a filesystem operation.
    """
    try:
        # 1. Force the system to flush any pending data to the disk
        storage.remount("/", readonly=False)
        # 2. On many nRF boards, this is the command to tell the flash to enter Deep Power Down.
        microcontroller.nvm.view = microcontroller.nvm.view # dummy access
    except:
        pass
    # Note: The flash chip automatically enters Deep Power Down 
    # during light_sleep() if no files are currently open.


# ============================================================================
# Time Management
# ============================================================================

# High-resolution timer state
# Option A: Store elapsed time (seconds since calibration) instead of absolute timestamps
# This preserves float32 precision: elapsed times (0.2, 0.4, 0.6s) have full precision,
# whereas large Unix timestamps lose fractional parts to float32 rounding.
_timer_state = {
    "monotonic_reference": None,     # monotonic time at calibration
    "calibration_time_unix": None,   # Unix timestamp at calibration (for config metadata)
}

def _calibrate_timer():
    """Calibrate the monotonic clock against RTC/system time."""
    try:
        mono = time.monotonic()
        unix_time = time.time()  # Get Unix timestamp for config metadata
        
        _timer_state["monotonic_reference"] = mono
        _timer_state["calibration_time_unix"] = unix_time
        
        DEBUG and print(f"[Timer] Calibration: Unix time={unix_time:.1f}, Monotonic={mono:.3f}")
    except Exception as e:
        DEBUG and print(f"[Timer] Calibration error: {e}")

def get_elapsed_time_since_calibration():
    """
    Get elapsed time (in seconds) since calibration with microsecond precision.
    
    Returns a small float (0 to 3600) that can be stored as float32 without precision loss.
    The config metadata stores the absolute calibration time; records use elapsed time.
    
    This solves the float32 precision issue:
    - WRONG: Unix timestamp (1780618496) + fraction (0.2) = precision loss
    - RIGHT: Elapsed time (0.0, 0.2, 0.4, 0.6) = full precision in float32
    """
    # Check if we need calibration (first call or after long pause)
    if (_timer_state["monotonic_reference"] is None):
        _calibrate_timer()
    
    try:
        # Return elapsed seconds since calibration
        # This is a small value (0-3600 per hour) with full float32 precision
        elapsed = time.monotonic() - _timer_state["monotonic_reference"]
        
        # Recalibrate every hour to prevent drift
        if elapsed > TIMER_CALIBRATION_INTERVAL:
            _calibrate_timer()
            elapsed = 0.0  # Reset elapsed after recalibration
        
        return elapsed
    except Exception as e:
        DEBUG and print(f"[Timer] get_elapsed_time error: {e}")
        return 0.0

def set_time(cmd):
    """
    Set the RTC time from external command.
    
    Args:
        cmd (str): Command string containing the date and time.
    
    Returns:
        bool or str: True if time was set successfully, error message otherwise.
    """
    try:
        parts = cmd.split()
        if len(parts) == 3:
            date_part = parts[1]
            time_part = parts[2]
            dt_parts = date_part.split('-') + time_part.split(':')
            if len(dt_parts) == 6:
                year, month, day, hour, minute, second = map(int, dt_parts)
                # Validate date/time values
                if not (1 <= month <= 12 and 1 <= day <= 31 and 0 <= hour <= 23 and 0 <= minute <= 59 and 0 <= second <= 59):
                    return "Error setting time: Invalid date/time values\r\n"
                # Create struct_time (weekday=0, yearday=0, isdst=-1)
                tm = time.struct_time((year, month, day, hour, minute, second, 0, 0, -1))
                r = rtc.RTC()
                r.datetime = tm
                # Recalibrate timer after setting system time
                _calibrate_timer()
                return True
            else:
                return "Error setting time: Invalid date/time format. Use: set_time YYYY-MM-DD HH:MM:SS\r\n"
        else:
            return "Error setting time: Invalid command format. Use: set_time YYYY-MM-DD HH:MM:SS\r\n"
    except Exception as e:
        return f"Error setting time: {e}\r\n"


# ============================================================================
# BLE Command Handlers
# ============================================================================

def handle_ble_commands():

    global current_save_to_disk
    """
    BLE command handler. Processes any pending UART data.
    """
    if uart_server.in_waiting:
        try:
            raw_bytes = uart_server.read(uart_server.in_waiting)
            text = raw_bytes.decode().strip().lower()
            DEBUG and print(f"RX: {text}")
            
            response = ""
            if text == 'save_buff':
                response = handle_save_buff()
            elif text == 'clear_data':
                response = clear_datafile()
            elif text == 'sleep':
                response = enter_sleep_mode()
            elif text.startswith('set_freq'):
                response = handle_set_freq(text)
            elif text.startswith('set_sens'):
                response = handle_set_sens(text)
            elif text.startswith('set_wakeup_sens'):
                response = handle_set_wakeup_sens(text)
            elif text.startswith('set_recording_freq'):
                response = handle_set_recording_freq(text)
            elif text.startswith('mark'):
                response = handle_set_mark(text)
            elif text.startswith('set_time'):
                response = set_time(text)
            elif text == 'config':
                response = get_config()
            elif text == 'status':
                response = get_status()
            elif text == 'download_bin':
                response = download_bin()
            elif text.startswith('save_to_disk'):
                response = handle_set_to_disk(text)
            elif text.startswith('emit_values'):
                response = handle_set_emit_values(text)
            else:
                response = "Commands: data, clear_buff, clear_data, sensors, config, status, sleep, set_freq [1-5], set_sens [0-6], set_wakeup_sens [0-6], set_recording_freq [1-10], mark <value>, set_time, save_to_disk on/off, emit_values on/off\r\n"
            
            if response:
                DEBUG and print(f"TX: {response.strip()}")
                uart_server.write(response.encode())
        except Exception as e:
            DEBUG and print(f"Error in BLE handler: {e}")

def handle_set_to_disk(cmd):
    """Command: save_to_disk on/off"""
    global current_save_to_disk
    if cmd.endswith('on'):
        current_save_to_disk = True
    elif cmd.endswith('off'):
        current_save_to_disk = False
    response = f"Save to disk set to {current_save_to_disk}\r\n"
    return response

def handle_save_buff():
    """ Save unsaved records to binary file before clearing """
    response = False
    try:
        if unsaved_records:
            DEBUG and print(f"Saving {len(unsaved_records)} binary records before clear...")
            save_to_disk(unsaved_records, BINARY_FILE)
            response = True
            unsaved_records.clear()
    except Exception as e:
        DEBUG and print(f"Error clearing buffer: {e}")
        response = f"Error clearing buffer: {e}\r\n"
    return response

def handle_set_freq(freq_ref):
    """ Command: set_freq [1-5] or set_freq (defaults to IMU_DEFAULT_FREQUENCY) """
    response = ""
    parts = freq_ref.split()
    if len(parts) > 1:
        try:
            level = int(parts[1])
            if 1 <= level <= 5:
                freq_levels = [IMU_FREQUENCY_LEVEL_1, IMU_FREQUENCY_LEVEL_2, IMU_FREQUENCY_LEVEL_3, IMU_FREQUENCY_LEVEL_4, IMU_FREQUENCY_LEVEL_5]
                set_imu_frequency(freq_levels[level - 1])
                response = f"IMU frequency set to level {level}\r\n"
            else:
                response = "Error: Frequency level must be 1-5\r\n"
        except ValueError:
            response = "Error: Invalid frequency level. Use: set_freq [1-5]\r\n"
    else:
        # No level specified, use default
        set_imu_frequency(IMU_DEFAULT_FREQUENCY)
        response = "IMU frequency set to default (level 4)\r\n"
    return response

def handle_set_sens(sens_ref):
    """Command: set_sens [0-6] or set_sens (defaults to IMU_DEFAULT_SENSITIVITY) """
    parts = sens_ref.split()
    if len(parts) > 1:
        try:
            level = int(parts[1])
            if 0 <= level <= 6:
                sens_levels = [IMU_SENSITIVITY_LEVEL_0, IMU_SENSITIVITY_LEVEL_1, IMU_SENSITIVITY_LEVEL_2, IMU_SENSITIVITY_LEVEL_3, IMU_SENSITIVITY_LEVEL_4, IMU_SENSITIVITY_LEVEL_5, IMU_SENSITIVITY_LEVEL_6]
                set_imu_sensitivity(sens_levels[level])
                response = f"IMU sensitivity set to level {level}\r\n"
            else:
                response = "Error: Sensitivity level must be 0-6\r\n"
        except ValueError:
            response = "Error: Invalid sensitivity level. Use: set_sens [0-6]\r\n"
    else:
        # No level specified, use default
        set_imu_sensitivity(IMU_DEFAULT_SENSITIVITY)
        response = "IMU sensitivity set to default (level 2)\r\n"
    return response

def handle_set_wakeup_sens(sens_ref):
    # Command: set_wakeup_sens [0-6] or set_wakeup_sens (defaults to WAKEUP_IMU_SENSITIVITY)
    response = ""
    global current_wakeup_sensitivity
    parts = sens_ref.split()
    if len(parts) > 1:
        try:
            level = int(parts[1])
            if 0 <= level <= 6:
                sens_levels = [IMU_SENSITIVITY_LEVEL_0, IMU_SENSITIVITY_LEVEL_1, IMU_SENSITIVITY_LEVEL_2, IMU_SENSITIVITY_LEVEL_3, IMU_SENSITIVITY_LEVEL_4, IMU_SENSITIVITY_LEVEL_5, IMU_SENSITIVITY_LEVEL_6]
                current_wakeup_sensitivity = sens_levels[level]
                response = f"Wake-up sensitivity set to level {level}\r\n"
            else:
                response = "Error: Sensitivity level must be 0-6\r\n"
        except ValueError:
            response = "Error: Invalid sensitivity level. Use: set_wakeup_sens [0-6]\r\n"
    else:
        # No level specified, use default
        current_wakeup_sensitivity = WAKEUP_IMU_SENSITIVITY
        response = "Wake-up sensitivity set to default (level 4)\r\n"
    return response

def handle_set_recording_freq(freq_ref):
    """ Command: set_recording_freq [1-10] or set_recording_freq (defaults to RECORDING_FREQUENCY) """
    response = ""
    global current_recording_frequency
    parts = freq_ref.split()
    if len(parts) > 1:
        try:
            freq = int(parts[1])
            if 1 <= freq <= 10:
                current_recording_frequency = freq
                response = f"Recording frequency set to {freq} Hz\r\n"
            else:
                response = "Error: Recording frequency must be 1-10 Hz\r\n"
        except ValueError:
            response = "Error: Invalid frequency. Use: set_recording_freq [1-10]\r\n"
    else:
        # No frequency specified, use default
        current_recording_frequency = RECORDING_FREQUENCY
        response = f"Recording frequency set to default ({RECORDING_FREQUENCY} Hz)\r\n"
    return response

def handle_set_mark(cmd):
    """ Command: mark <mark_string> - Save timestamped mark to marks.csv """
    response = ""
    parts = cmd.split(None, 1)  # Split into at most 2 parts to preserve spaces in mark value
    if len(parts) > 1:
        mark_value = parts[1]
        try:
            timestamp = get_high_resolution_timestamp()
            # Append to marks.csv with timestamp and mark value (format matches data file for synchronization)
            with open(MARKS_FILE, "a") as f:
                f.write(f"{timestamp:.6f}," + mark_value + "\r\n")
            response = "Mark saved: " + mark_value + "\r\n"
            DEBUG and print("Mark saved at " + str(timestamp) + ": " + mark_value)
        except OSError as e:
            response = "Error saving mark: " + str(e) + "\r\n"
            DEBUG and print("Error saving mark: " + str(e))
    else:
        response = "Error: mark requires a value. Use: mark <mark_string>\r\n"
    return response

def handle_set_emit_values(cmd):
    """ Command: emit_values on/off - Whether to include sensor values in BLE status updates """
    global current_emit_values
    response = "current_emit_values set to "
    if cmd.endswith('on'):
        current_emit_values = True
        response += "on\r\n"
    elif cmd.endswith('off'):
        current_emit_values = False
        response += "off\r\n"
    else:
        response = "Error: Invalid command. Use: emit_values on/off\r\n"
    return response

# --- Main Logic ---
# Simple continuous recording with BLE control
DEBUG and print("Starting...")

# Check for wake-up alarms (indicates device woke from sleep)
if alarm.wake_alarm:
    DEBUG and print(f"Woken by: {alarm.wake_alarm}")
else:
    DEBUG and print("Cold boot or power-on")

# Initialize globals
unsaved_records = []  # Tracks binary records (12 bytes each) not yet saved to file
iteration_count = 0  # Counter for BLE updates
ble_was_connected = False  # Track BLE connection state
advertisement = ProvideServicesAdvertisement(uart_server)
ble.start_advertising(advertisement)
DEBUG and print("BLE advertising started.")

# Print system status on boot
if DEBUG:
    print(get_config())
    imu_status = read_imu_status()
    print(f"--- IMU STATUS ---")
    print(f"Frequency Reg (0x10): {imu_status['frequency_bits']:#04x} (Level {imu_status['frequency_bits']})")
    print(f"General Sensitivity:  {imu_status['sensitivity_bits']:#04x} (Level {imu_status['sensitivity_scale']})")
    print(f"Wake-up Threshold:    {imu_status['wake_sensitivity']}")
    print(f"------------------\n")

# Blink LED 3 times on startup
blink_led(LED_GREEN, 1, 1)

# Initialize high-resolution timer calibration
_calibrate_timer()
DEBUG and print(f"[Timer] Using monotonic clock for high-resolution timestamps (microsecond precision)")

# Main loop: continuously record IMU data
while True:
    iteration_count += 1

    # Handle BLE connection state changes
    if ble.connected and not ble_was_connected:
        # Client just connected
        DEBUG and print("BLE client connected!")
        ble.stop_advertising()
        ble_was_connected = True
    elif not ble.connected and ble_was_connected:
        # Client just disconnected
        DEBUG and print("BLE client disconnected.")
        ble.start_advertising(advertisement)
        ble_was_connected = False
    
    # Record IMU data continuously
    try:
        imu_data = read_imu()
        elapsed_time = get_elapsed_time_since_calibration()
        accel_mag = imu_data["accel_mag"]
        gyro_mag = imu_data["gyro_mag"]
        battery_voltage = imu_data["battery_voltage"]

        # Store as binary with dynamic fields based on READ_* flags
        pack_data = [elapsed_time]
        
        if READ_ACCELEROMETER:
            pack_data.append(accel_mag)
        
        if READ_GYROSCOPE:
            pack_data.append(gyro_mag)
        
        if READ_BATTERY:
            pack_data.append(battery_voltage)
        
        # Use the pre-calculated format string
        binary_data = struct.pack(RECORD_PACK_FORMAT, *pack_data)
        unsaved_records.append(binary_data)

        # Auto-save to binary file every N records
        if len(unsaved_records) >= AUTO_SAVE_RECORDS_INTERVAL:
            DEBUG and print(f"Auto-saving {len(unsaved_records)} records to binary file...")
            if save_to_disk(unsaved_records, BINARY_FILE):
                # Successfully saved, clear buffer
                unsaved_records.clear()

        if current_emit_values and ble.connected:
            # Format text for broadcasting (dynamic based on READ_* flags, no timestamp)
            values = []
            if READ_ACCELEROMETER:
                values.append(f"{accel_mag:.2f}")
            if READ_GYROSCOPE:
                values.append(f"{gyro_mag:.2f}")
            if READ_BATTERY:
                values.append(f"{battery_voltage:.2f}")
            
            last_imu_text = ",".join(values) + "\n"
            # DEBUG and print(f"Recorded: {last_imu_text.strip()}")
            uart_server.write(last_imu_text.encode())

    except Exception as e:
        if isinstance(e, MemoryError):
            # If memory allocation error, try to save what we have and clear buffer
            DEBUG and print(f"MemoryError: Attempting to save unsaved records before clearing buffer...")
            try:
                save_to_disk(unsaved_records, BINARY_FILE)
            except Exception as save_e:
                DEBUG and print(f"Error saving to disk during MemoryError handling: {save_e}")
            # Flush the buffer no matter what
            unsaved_records.clear()
        else:
            DEBUG and print(f"Error in main loop: {e}")
    
    # Handle BLE commands (non-blocking)
    if ble.connected:
        handle_ble_commands()
    
    time.sleep(1 / current_recording_frequency)  # Sample at configured frequency

