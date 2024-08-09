import RPi.GPIO as GPIO
import smbus 
from time import sleep

# Setup GPIO pins
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
# Set the servo motor pin as output pin
GPIO.setup(4, GPIO.OUT)

pwm = GPIO.PWM(4, 50)
pwm.start(0)

# MPU9250 Registers and their Address
PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
INT_ENABLE = 0x38
GYRO_XOUT = 0x43
GYRO_YOUT = 0x45
GYRO_ZOUT = 0x47
MAG_XOUT = 0x03
MAG_YOUT = 0x05
MAG_ZOUT = 0x07
MAG_CONTROL = 0x0A

bus = smbus.SMBus(1)
Device_Address = 0x68
Magnetometer_Address = 0x0C  # Magnetometer address

def angle(Angle):
    duty = Angle / 18 + 2
    GPIO.output(4, True)
    pwm.ChangeDutyCycle(duty)
    GPIO.output(4, False)

def MPU_Init():
    # Write to sample rate register
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)

    # Write to power management register
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)

    # Write to Configuration register
    bus.write_byte_data(Device_Address, CONFIG, 0)

    # Write to Gyro configuration register
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)

    # Write to interrupt enable register
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)

    # Enable Magnetometer
    bus.write_byte_data(Magnetometer_Address, MAG_CONTROL, 0x01)

def read_raw_data(addr, device_address=Device_Address):
    high = bus.read_byte_data(device_address, addr)
    low = bus.read_byte_data(device_address, addr + 1)
    value = ((high << 8) | low)
    if value > 32768:
        value = value - 65536
    return value

MPU_Init()

while True:
    # Read Gyroscope raw value
    gyro_x = read_raw_data(GYRO_XOUT)
    gyro_y = read_raw_data(GYRO_YOUT)
    gyro_z = read_raw_data(GYRO_ZOUT)

    # Read Magnetometer raw value
    mag_x = read_raw_data(MAG_XOUT, Magnetometer_Address)
    mag_y = read_raw_data(MAG_YOUT, Magnetometer_Address)
    mag_z = read_raw_data(MAG_ZOUT, Magnetometer_Address)

    Gx = gyro_x / 131.0
    Gy = gyro_y / 131.0
    Gz = gyro_z / 131.0

    Mx = mag_x * 0.6  # Scaling factor for magnetometer data
    My = mag_y * 0.6
    Mz = mag_z * 0.6

    # Uncomment below line to see the Gyroscope and Magnetometer values
    # print ("Gx=%.2f" %Gx, u'\u00b0'+ "/s", "\tGy=%.2f" %Gy, u'\u00b0'+ "/s", "\tGz=%.2f" %Gz, u'\u00b0'+ "/s", "\tMx=%.2f uT" %Mx, "\tMy=%.2f uT" %My, "\tMz=%.2f uT" %Mz)

    # Convert magnetometer Y axis values from 0 to 180
    in_min = -1
    in_max = 1
    out_min = 0
    out_max = 180
    value = (My - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    value = int(value)

    if 0 <= value <= 180:
        angle(value)  # Rotate the servo motor using the sensor values
        sleep(0.08)
