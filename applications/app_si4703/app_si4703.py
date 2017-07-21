import sys
sys.path.append('../../') #append the path where the drivers directory is placed
from drivers.si4703.si4703 import *
import time

si4703 = SI4703('/dev/ttyUSB0')
si4703.initI2C(True, True)
time.sleep(0.3)


si4703.enable()
print(si4703.getIdentificationString())

si4703.setVolume(volume=0x02, postponeI2CWrite=True)
si4703.setChannel()

time.sleep(30)
si4703.deinitI2C()