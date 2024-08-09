import RPi.GPIO as GPIO
import smbus
from time import sleep
import math

# Setup GPIO pins
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(4, GPIO.OUT)

pwm = GPIO.PWM(4, 50)
pwm.start(0)

# MPU6050 and HMC5883L Addresses
MPU6050_ADDRESS = 0x68
HMC5883L_ADDRESS = 0x1E

# MPU6050 Registers
PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
INT_ENABLE = 0x38
GYRO_XOUT = 0x43
GYRO_YOUT = 0x45
GYRO_ZOUT = 0x47

# HMC5883L Registers
CONFIG_A = 0x00
MODE = 0x02
DATAX0 = 0x03
DATAX1 = 0x04
DATAY0 = 0x05
DATAY1 = 0x06
DATAZ0 = 0x07
DATAZ1 = 0x08

bus = smbus.SMBus(1)

def angle(Angle):
    duty = Angle / 18 + 2
    GPIO.output(4, True)
    pwm.ChangeDutyCycle(duty)
    sleep(0.1)
    GPIO.output(4, False)

def MPU6050_Init():
    bus.write_byte_data(MPU6050_ADDRESS, SMPLRT_DIV, 7)
    bus.write_byte_data(MPU6050_ADDRESS, PWR_MGMT_1, 1)
    bus.write_byte_data(MPU6050_ADDRESS, CONFIG, 0)
    bus.write_byte_data(MPU6050_ADDRESS, GYRO_CONFIG, 24)
    bus.write_byte_data(MPU6050_ADDRESS, INT_ENABLE, 1)

def HMC5883L_Init():
    bus.write_byte_data(HMC5883L_ADDRESS, CONFIG_A, 0x70)
    bus.write_byte_data(HMC5883L_ADDRESS, MODE, 0)

def read_raw_data_16bit(addr, device_addr):
    high = bus.read_byte_data(device_addr, addr)
    low = bus.read_byte_data(device_addr, addr + 1)
    value = (high << 8) | low
    if value >= 0x8000:
        return -((65535 - value) + 1)
    else:
        return value

MPU6050_Init()
HMC5883L_Init()

while True:
    # Read Gyroscope raw values
    gyro_x = read_raw_data_16bit(GYRO_XOUT, MPU6050_ADDRESS)
    gyro_y = read_raw_data_16bit(GYRO_YOUT, MPU6050_ADDRESS)
    gyro_z = read_raw_data_16bit(GYRO_ZOUT, MPU6050_ADDRESS)

    Gx = gyro_x / 131.0
    Gy = gyro_y / 131.0
    Gz = gyro_z / 131.0

    print(f"Gyroscope -> Gx: {Gx:.2f}, Gy: {Gy:.2f}, Gz: {Gz:.2f} degrees/s")

    # Read Magnetometer raw values
    x = read_raw_data_16bit(DATAX0, HMC5883L_ADDRESS)
    y = read_raw_data_16bit(DATAY0, HMC5883L_ADDRESS)
    z = read_raw_data_16bit(DATAZ0, HMC5883L_ADDRESS)

    heading = math.atan2(y, x) * (180 / math.pi)
    if heading < 0:
        heading += 360

    print(f"Compass Heading: {heading:.2f} degrees")

    # Map heading to servo motor angle (0-180 degrees)
    angle_value = int(heading / 2)
    angle(angle_value)

    sleep(0.1)  # Adjust as needed
