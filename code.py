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
from adafruit_lsm6ds import Rate, AccelRange, GyroRange
from adafruit_lsm6ds.lsm6ds3trc import LSM6DS3TRC
from seeed_xiao_nrf52840 import Battery

# --- Constants ---
DEBUG = True                        # Set to True to enable print statements

RECORDING_FREQUENCY         = 5     # Hz. Rate to sample IMU data.
AUTO_SAVE_RECORDS_INTERVAL  = 10000 # Save to file every N records.

DATA_FILE             = "imu_data.csv"  # File to store recorded data.
MIN_DISK_BUFFER_BYTES = 1024 * 5        # Min free disk space to keep
SAVE_TO_FILE          = False           # Set to True to enable file saving, False to keep data in RAM only

# --- SENSITIVITY LEVELS ---

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

IMU_DEFAULT_SENSITIVITY = IMU_SENSITIVITY_LEVEL_2  # Default sensitivity for general motion detection

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

IMU_DEFAULT_FREQUENCY = IMU_FREQUENCY_LEVEL_3  # Default frequency for wake-up detection

# --- SLEEP CONFIGURATION ---
WAKEUP_REQUIRED_HITS    = 10     # 'n' values: Number of triggers needed to fully wake up
WAKEUP_WINDOW_SECONDS   = 10.0   # Period to detect those 'n' hits
WAKEUP_IMU_SENSITIVITY  = IMU_SENSITIVITY_LEVEL_4   # Sensitivity level for wake-up detection

IMU_SLEEP_FREQUENCY   = IMU_FREQUENCY_LEVEL_2  # Lower frequency during sleep to save power

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

# --- Globals ---

ble = BLERadio()
uart_server = UARTService()
current_wakeup_sensitivity = WAKEUP_IMU_SENSITIVITY  # Track current wake-up sensitivity level
current_recording_frequency = RECORDING_FREQUENCY    # Track current recording frequency in Hz

# --- Functions ---

def write_reg(register, value):
    """Helper to write to LSM6DS3TR-C registers (Default Address 0x6A)"""
    try:
        while not imu_i2c.try_lock():
            pass
        imu_i2c.writeto(0x6A, bytes([register, value]))
        imu_i2c.unlock()
    except Exception as e:
        DEBUG and print(f"Error writing register 0x{register:02X}: {e}")

def read_reg(reg):
    """Reads a single byte from a specific register."""
    result = bytearray(1)
    with imu_device as device:
        # Write register address, then read 1 byte
        device.write_then_readinto(bytes([reg]), result)
    return result[0]

def read_imu():
    """Read the IMU data and compute magnitudes."""
    accel_x, accel_y, accel_z = sensor.acceleration
    gyro_x, gyro_y, gyro_z = sensor.gyro
    temp = sensor.temperature
    
    # Compute magnitudes
    accel_mag = (accel_x**2 + accel_y**2 + accel_z**2)**0.5
    gyro_mag = (gyro_x**2 + gyro_y**2 + gyro_z**2)**0.5
    
    return {
        "accel_mag": accel_mag,
        "gyro_mag": gyro_mag,
        "temp": temp,
    }

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

def configure_wakeup_motion_detection(sensitivity_level=WAKEUP_IMU_SENSITIVITY, frequency_level=IMU_SLEEP_FREQUENCY):
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

def enter_sleep_mode():
    """Puts the device into light sleep mode and configures wake-up on motion detection."""
    configure_wakeup_motion_detection(sensitivity_level=current_wakeup_sensitivity)
    while True:
        hits = 0
        start_time = None
        
        DEBUG and print(f"Waiting for {WAKEUP_REQUIRED_HITS} motion events...")

        while hits < WAKEUP_REQUIRED_HITS:
            # 1. Clear latch so the pin can transition again
            clear_imu_interrupt()
            
            # 2. Sleep until the NEXT motion event
            motion_alarm = alarm.pin.PinAlarm(pin=board.IMU_INT1, value=True)
            alarm.light_sleep_until_alarms(motion_alarm)
            
            # 3. Handle the hit
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
        set_imu_frequency(IMU_DEFAULT_FREQUENCY)        # Restore normal frequency
        set_imu_sensitivity(IMU_DEFAULT_SENSITIVITY)    # Restore normal sensitivity

        break # Fully wake up and continue code.py
   
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

def get_status_info():
    total_ram, free_ram = get_ram_info()
    used_ram = total_ram - free_ram
    total_flash, free_flash = get_flash_info()
    used_flash = total_flash - free_flash
    
    # Get current date and time
    current_time = time.localtime()
    date_time_str = "{:04d}-{:02d}-{:02d} {:02d}:{:02d}:{:02d}".format(
        current_time[0], current_time[1], current_time[2],
        current_time[3], current_time[4], current_time[5]
    )
    
    # Get data file size safely (CircuitPython doesn't have os.path.exists)
    try:
        stat_result = os.stat(DATA_FILE)
        file_size = stat_result[6]  # Index 6 is file size in CircuitPython tuple
    except OSError:
        file_size = 0
    
    # Get battery status
    battery_info = "Battery: N/A"
    try:
        with Battery() as bat:
            voltage = bat.voltage
            charge_status = "Charged" if bat.charge_status else "Charging"
            battery_info = f"Battery: {voltage:.2f}V ({charge_status})"
    except Exception as e:
        DEBUG and print(f"Error reading battery: {e}")
    
    imu_status = read_imu_status()  # Read IMU registers for debug output

    response = (
        f"datetime: {date_time_str}\r\n"
        f"{battery_info}\r\n"
        f"SRAM: {used_ram}/{total_ram} bytes ({used_ram/1024:.1f}/{total_ram/1024:.1f} KB) ({100*used_ram//total_ram}% used)\r\n"
        f"Flash: {used_flash}/{total_flash} bytes ({used_flash/1024:.1f}/{total_flash/1024:.1f} KB) ({100*used_flash//total_flash}% used)\r\n"
        f"Unsaved records: {len(unsaved_records)}\r\n"
        f"Record file size: {file_size} bytes\r\n"
        f"IMU Frequency: {imu_status['frequency_bits']:#04x}\r\n"
        f"Sensitivity: {imu_status['sensitivity_scale']} ({imu_status['sensitivity_bits']:#04x})\r\n"
        f"Wake-up Sensitivity: {imu_status['wake_sensitivity']}\r\n"
    )
    return response

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

    if DEBUG:
        print(f"--- IMU STATUS ---")
        print(f"Frequency Reg (0x10): {reg_10:#04x} (Level {freq_val})")
        print(f"General Sensitivity:  {current_scale}")
        print(f"Wake-up Threshold:    {wake_sens} ({reg_5B:#04x})")
        print(f"------------------\n")

    return {
        "frequency_bits": freq_val,
        "sensitivity_bits": scale_bits,
        "sensitivity_scale": current_scale,
        "wake_sensitivity": wake_sens,
    }


def is_battery_safe():
    """
    Checks if voltage is high enough for safe flash writing.
    3.5V is a safe 'low battery' threshold to prevent corruption.
    """
    try:
        with Battery() as bat:
                voltage = bat.voltage
                return voltage > 3.5
    except Exception as e:
        DEBUG and print(f"Error reading battery: {e}")
        return False

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
    """Clear data file by opening in write mode (truncates automatically)."""
    try:
        with open(DATA_FILE, 'w') as f:
            f.write("# IMU Data Logger (Magnitude Mode)\n")
            f.write("timestamp,accel_magnitude,gyro_magnitude\n")
        DEBUG and print(f"Cleared and reset data file: {DATA_FILE}")
        return True
    except OSError as e:
        DEBUG and print(f"Error clearing data file: {e}")
        return False

def save_to_file(records):
    """
    Save records to CSV file. Creates file with header if needed.
    
    Args:
        records (list): List of records to be saved.
    
    Returns:
        bool: True if records were saved successfully, False otherwise.
    """
    if not records or not SAVE_TO_FILE:
        if not is_battery_safe():
            DEBUG and print("Battery voltage too low for safe file writing.")
            blink_led(LED_RED, 3, 0.1)  # Indicate low battery with red LED
        return False
    
    try:
        # Calculate actual size of records to be written
        estimated_size = sum(len(line.encode()) for line in records)
        
        # Check if we have enough space including buffer
        if not has_enough_space_for_record(estimated_size):
            DEBUG and print(f"Warning: Insufficient disk space. Flushing file to free space...")
            
            # Try to delete the file to free up space
            try:
                os.remove(DATA_FILE)
                DEBUG and print(f"Deleted {DATA_FILE} to free space")
            except OSError:
                pass  # File might not exist
            
            # Re-check available space after flush
            if not has_enough_space_for_record(estimated_size):
                DEBUG and print(f"Error: Still insufficient space even after flush. Available: {get_free_space_bytes()} bytes, "
                          f"Required: {estimated_size + MIN_DISK_BUFFER_BYTES} bytes")
                return False
        
        file_exists = False
        try:
            os.stat(DATA_FILE)
            file_exists = True
        except OSError:
            file_exists = False
        
        # Open in append mode, or create if doesn't exist
        with open(DATA_FILE, "a") as f:
            # Write header if new file
            if not file_exists:
                f.write("# IMU Data Logger (Magnitude Mode)\n")
                f.write("timestamp,accel_magnitude,gyro_magnitude\n")
            
            # Write all records
            for line in records:
                f.write(line)

            # Force data out of the buffer and into the file immediately (important for power management)
            f.flush() 
        
        DEBUG and print(f"Saved {len(records)} records to {DATA_FILE}")
        return True
    except OSError as e:
        # Check if it's a read-only filesystem error
        if e.errno == 30:  # EROFS - Read-only file system
            DEBUG and print(f"Warning: Filesystem is read-only. Data kept in RAM only. Error: {e}")
        else:
            DEBUG and print(f"Error saving to file: {e}")
        return False

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
                return True
            else:
                return "Error setting time: Invalid date/time format. Use: set_time YYYY-MM-DD HH:MM:SS\r\n"
        else:
            return "Error setting time: Invalid command format. Use: set_time YYYY-MM-DD HH:MM:SS\r\n"
    except Exception as e:
        return f"Error setting time: {e}\r\n"

def handle_ble_commands():
    """
    BLE command handler. Processes any pending UART data.
    """
    if uart_server.in_waiting:
        try:
            raw_bytes = uart_server.read(uart_server.in_waiting)
            text = raw_bytes.decode().strip().lower()
            DEBUG and print(f"RX: {text}")
            
            response = ""
            if text == 'data':
                # Send all data from file via BLE
                response = f"--- start\r\n"
                uart_server.write(response.encode())
                try:
                    with open(DATA_FILE, 'r') as f:
                        for line in f:
                            uart_server.write(line.encode())
                            time.sleep(0.01)  # Small delay between records
                    response = f"--- end\r\n"
                except OSError as e:
                    response = f"Error reading file: {e}\r\n"
            elif text == 'save_buff':
                # Save unsaved records before clearing
                try:
                    if unsaved_records:
                        print(f"Saving {len(unsaved_records)} records before clear...")
                        save_to_file(unsaved_records)
                        response = True
                        unsaved_records.clear()
                except Exception as e:
                    DEBUG and print(f"Error clearing buffer: {e}")
                    response = f"Error clearing buffer: {e}\r\n"
            elif text == 'clear_data':
                response = clear_datafile()
            elif text == 'sensors':
                d = read_imu()
                response = (
                    f"CPU Temp: {microcontroller.cpu.temperature} oC\r\n"
                    f"CPU Voltage: {round(microcontroller.cpu.voltage, 1)} volts\r\n"
                    f"Accel Magnitude: {d['accel_mag']:.2f} m/s^2\r\n"
                    f"Gyro Magnitude: {d['gyro_mag']:.2f} rad/s\r\n"
                )
            elif text == 'sleep':
                response = enter_sleep_mode()
            elif text.startswith('set_freq'):
                # Command: set_freq [1-5] or set_freq (defaults to IMU_DEFAULT_FREQUENCY)
                parts = text.split()
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
            elif text.startswith('set_sens'):
                # Command: set_sens [0-6] or set_sens (defaults to IMU_DEFAULT_SENSITIVITY)
                parts = text.split()
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
            elif text.startswith('set_wakeup_sens'):
                # Command: set_wakeup_sens [0-6] or set_wakeup_sens (defaults to WAKEUP_IMU_SENSITIVITY)
                global current_wakeup_sensitivity
                parts = text.split()
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
            elif text.startswith('set_recording_freq'):
                # Command: set_recording_freq [1-10] or set_recording_freq (defaults to RECORDING_FREQUENCY)
                global current_recording_frequency
                parts = text.split()
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
            elif text.startswith('set_time'):
                response = set_time(text)
            elif text == 'status':
                response = get_status_info()
            else:
                response = "Commands: data, clear_buff, clear_data, sensors, status, sleep, set_freq [1-5], set_sens [0-6], set_wakeup_sens [0-6], set_recording_freq [1-10], set_time\r\n"
            
            if response:
                DEBUG and print(f"TX: {response.strip()}")
                uart_server.write(response.encode())
        except Exception as e:
            DEBUG and print(f"Error in BLE handler: {e}")


# --- Main Logic ---
# Simple continuous recording with BLE control
DEBUG and print("Starting IMU Data Logger...")

# Check for wake-up alarms (indicates device woke from sleep)
if alarm.wake_alarm:
    DEBUG and print(f"Woken by: {alarm.wake_alarm}")
else:
    DEBUG and print("Cold boot or power-on")

# Initialize globals
unsaved_records = []  # Tracks records not yet saved to file
iteration_count = 0  # Counter for BLE updates
ble_was_connected = False  # Track BLE connection state
advertisement = ProvideServicesAdvertisement(uart_server)
ble.start_advertising(advertisement)
DEBUG and print("BLE advertising started.")

# Print system status on boot
DEBUG and print(get_status_info())

# Blink LED 3 times on startup
blink_led(LED_GREEN, 1, 1)

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

        line = "{:.2f},{:.2f},{:.2f}\n".format(
            time.monotonic(),
            imu_data["accel_mag"],
            imu_data["gyro_mag"]
        )
        unsaved_records.append(line)
        DEBUG and print(f"Recorded: {line.strip()}")
        
        uart_server.write(line.strip().encode())

        # Auto-save to file every N records
        if len(unsaved_records) >= AUTO_SAVE_RECORDS_INTERVAL:
            DEBUG and print(f"Auto-saving records to file...")
            if save_to_file(unsaved_records):
                # Successfully saved, clear buffer
                unsaved_records.clear()

    except Exception as e:
        if isinstance(e, MemoryError):
            # If memory allocation error, try to save what we have and clear buffer
            DEBUG and print(f"MemoryError: Attempting to save unsaved records before clearing buffer...")
            try:
                save_to_file(unsaved_records)
            except Exception as save_e:
                DEBUG and print(f"Error saving to file during MemoryError handling: {save_e}")
            # Flush the buffer no matter what
            unsaved_records.clear()
        else:
            DEBUG and print(f"Error in main loop: {e}")
    
    # try:
    #     # Send status over BLE every 5 iterations
    #     if DEBUG and iteration_count >= 5 and ble.connected:
    #         try:
    #             uart_server.write(get_status_info().encode())
    #         except Exception as e:
    #             if DEBUG:
    #                 print(f"Error sending to BLE: {e}")
    #         iteration_count = 0
    # except Exception as e:
    #     if DEBUG:
    #         print(f"Error in BLE status update: {e}")

    # Handle BLE commands (non-blocking)
    if ble.connected:
        handle_ble_commands()
    
    time.sleep(1 / current_recording_frequency)  # Sample at configured frequency
