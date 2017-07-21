import sys
sys.path.append('../../') #append the path where the drivers directory is placed
from drivers.rtc.ds3231 import *
import time

startTime = time.time()
rtc = RTC_DS3231('/dev/ttyUSB0', )

rtc.initI2C(enablePower=True, enablePullUp=False)

time.sleep(0.05)


# if False == rtc.setCurrTime():
#     print("Couldn't set current time")
#
# if False == rtc.setCurrDate():
#     print("Couldn't set date")

time.sleep(0.05)

rcvTimeStr = rtc.readTime()
print(rcvTimeStr)

rcvTimeDate = rtc.readDate()
print(rcvTimeDate)

print('Temperature is = {}°C'.format(rtc.readTemp()));



rtc.deinitI2C()

print(time.time() - startTime);