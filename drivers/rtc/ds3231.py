import sys
sys.path.append('../../pyBusPirateLite/') #append the path where the pyBusPirateLite is placed
from pyBusPirateLite.I2C import *
import time

class RTC_DS3231:
    I2C_DS3231_ADDR_WR = 0xD0
    I2C_DS3231_ADDR_RD = 0xD1

    REG_SECONDS_ADDR   = 0x00
    REG_MINUTES_ADDR   = 0x01
    REG_HOURS_ADDR     = 0x02
    REG_WEEK_DAY_ADDR  = 0x03
    REG_MONTH_DAY_ADDR = 0x04
    REG_YEAR_ADDR      = 0x05
    REG_TEMP_UPPER_ADDR = 0x11
    REG_TEMP_LOWER_ADDR = 0x12


    def __init__(self, bpInterface='', bpBaudRate=115200):

        if '' == bpInterface or 0 == bpBaudRate:
            raise UserWarning('Invalid Bus Pirate interface parameters!')

        self.i2c = I2C(connect=False)
        self.__bpInterface = bpInterface
        self.__bpBaudRate = bpBaudRate

        # should be updated after successful calling to enable()
        self.__shadowRegs = [0x00] * 16 # at index 0 is address 0x00


    def initI2C(self, enablePower=False, enablePullUp=False):
        """
        Initializes the I2C interface
        :param enablePower:
        :param enablePullUp:
        :return:

        :raises:
                The same what I2C.speed, BBIO_base.connect() could raise, i2c.enter()
        """
        self.i2c.connect(self.__bpInterface, self.__bpBaudRate, 0.1)
        self.i2c.enter()
        self.i2c.speed = '400kHz'

        if True == enablePower or True == enablePullUp:

            self.i2c.port.flushInput()

            # bus pirate's command (over UART interface):
            cmd = 0x40 | (int(enablePower) << 3) | (int(enablePullUp) << 2)
            self.i2c.write(cmd)

            # wait for transfer and response
            self.i2c.timeout(self.i2c.minDelay * 16)

            # check response (from uart)
            if b'\x01' != self.i2c.response(1, binary=True):
                raise UserWarning("Couldn't enable power and set pullups.")


    def deinitI2C(self):
        self.i2c.port.flushInput()
        # exit to BB mode
        self.i2c.write(0x00)
        # wait for transfer and response
        self.i2c.timeout(self.i2c.minDelay * 10)

        # exit from BB mode (also resets BusPirate)
        self.i2c.write(0x0F)
        self.i2c.timeout(self.i2c.minDelay * 10)
        self.i2c.port.flushInput()

        self.i2c.disconnect()

        # reset mode state in I2C base class
        self.i2c.mode = None


    def setCurrTime(self):
        dateTime = time.localtime()

        rtcSec = dateTime.tm_sec
        u = rtcSec % 10
        d = int(rtcSec / 10)
        rtcSec = u | (d << 4)

        rtcMin = dateTime.tm_min
        u = rtcMin % 10
        d = int(rtcMin / 10)
        rtcMin = u | (d << 4)

        rtcHour = dateTime.tm_hour
        u = rtcHour % 10
        d = int(rtcHour / 10)
        rtcHour = u | (d << 4)

        return self.__i2cWrite(0x00, [rtcSec, rtcMin, rtcHour])


    def setCurrDate(self):
        dateTime = time.localtime()
        rtcWeekDay = dateTime.tm_wday + 1
        rtcDay = dateTime.tm_mday
        u =  rtcDay % 10
        d = int(rtcDay / 10)
        rtcDay = u | (d << 4)

        rtcMonth = dateTime.tm_mon
        u = rtcMonth % 10
        d = int(rtcMonth / 10)
        rtcMonth = u | (d << 4)

        rtcYear = dateTime.tm_year % 100
        u = rtcYear % 10
        d = int(rtcYear / 10)
        rtcYear = u | (d << 4)

        return self.__i2cWrite(0x03, [rtcWeekDay, rtcDay, rtcMonth, rtcYear])



    def readTime(self):
        # change rtc register pointer
        if True == self.__i2cWrite(RTC_DS3231.REG_SECONDS_ADDR, []):
            rcvBytes = self.i2c.write_then_read(1, 3, [RTC_DS3231.I2C_DS3231_ADDR_RD])
            #fixme remove assumption that time is in 24h format
            rcvStr = '{:02x}:{:02x}:{:02x}'.format(rcvBytes[2], rcvBytes[1], rcvBytes[0])
            return rcvStr, rcvBytes

        raise ProtocolError("Couldn't read time from rtc")


    def readDate(self):
        # change rtc register pointer
        if True == self.__i2cWrite(RTC_DS3231.REG_WEEK_DAY_ADDR, []):
            # reads in order: day of week, month day, month, year
            rcvBytes =  self.i2c.write_then_read(1, 4, [RTC_DS3231.I2C_DS3231_ADDR_RD])
            dayName = ['Mon', 'Tue', 'Wed', 'Thu', 'Fri', 'Sat', 'Sun']
            retStr = '{}, {:02x}.{:02x}.20{:02x}'.format(dayName[rcvBytes[0]-1], rcvBytes[1], rcvBytes[2], rcvBytes[3])
            return retStr, rcvBytes

        raise ProtocolError("Couldn't read date from rtc")


    def readTemp(self):
        # change rtc register pointer
        if True == self.__i2cWriteTest(RTC_DS3231.REG_TEMP_UPPER_ADDR, []):
            # reads in order: temp upper byte, temp lower byte
            rcvBytes = self.i2c.write_then_read(1, 2, [RTC_DS3231.I2C_DS3231_ADDR_RD])
            return float(rcvBytes[0]) + (((rcvBytes[1] >> 6) & 0xFF) * 0.25)


    def __i2cWrite(self, regAddress, dataList):
        if regAddress <= 0x12:

            self.i2c.port.flushInput()
            self.i2c.start()
            response = self.i2c.transfer([
                RTC_DS3231.I2C_DS3231_ADDR_WR,
                regAddress,
                *dataList # just insert the data (only LSB byte is relevant here!
            ])
            self.i2c.stop()

            # check response
            writeLen = len(dataList) + 2 # +2 for dev addr, and reg addr
            if len(response) == writeLen and response == [True] * writeLen:
                return True

        return False

    def __i2cWriteTest(self, regAddress, dataList):
        if regAddress <= 0x12:

            self.i2c.port.flushInput()
            self.i2c.start()
            response = self.i2c.transfer([
                RTC_DS3231.I2C_DS3231_ADDR_WR,
                regAddress,
                *dataList # just insert the data (only LSB byte is relevant here!
            ])
            # self.i2c.stop()

            # check response
            writeLen = len(dataList) + 2 # +2 for dev addr, and reg addr
            if len(response) == writeLen and response == [True] * writeLen:
                return True

        return False
