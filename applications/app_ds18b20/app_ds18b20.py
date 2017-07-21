import sys
sys.path.append('../../') #append the path where the drivers directory is placed
from drivers.digital_temperature.ds18b20 import *
import time
import re

print('started')
ds = DS18B20('/dev/ttyUSB0', 115200)

# rom_lists = ds.find_roms()
# print(rom_lists)

st = time.time()
while time.time() - st < 10.0:
    print('temp = {} °C'.format(ds.get_temp()))
    time.sleep(0.250)


time.sleep(0.5)

ds.poweroff()
