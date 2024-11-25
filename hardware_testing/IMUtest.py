import time
import board
import busio
import adafruit_lsm6ds

i2c = busio.I2C(board.SCL, board.SDA)
sox = adafruit_lsm6ds.LSM6DSOX(i2c)

