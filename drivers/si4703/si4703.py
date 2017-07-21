import sys
from _socket import timeout

sys.path.append('../../pyBusPirateLite/') #append the path where the pyBusPirateLite is placed
from pyBusPirateLite.I2C import *
from pyBusPirateLite.BitBang import BitBang
from pyBusPirateLite.BBIO_base import PIN_CS, PIN_MOSI, PIN_POWER

from pyBusPirateLite.rawwire import *
import time

class SI4703:
    SI4703_I2C_ADDR = 0x20
    SI4703_I2C_WR_ADDR = SI4703_I2C_ADDR
    SI4703_I2C_RD_ADDR = SI4703_I2C_ADDR | 0x01

    DEV_ID_REG_ADDR = 0x00
    CHIP_ID_REG_ADDR = 0x01
    PWR_CONF_REG_ADDR = 0x02
    CHANNEL_REG_ADDR = 0x03
    SYS_CONF1_REG_ADDR = 0x04
    SYS_CONF2_REG_ADDR = 0x05
    SYS_CONF3_REG_ADDR = 0x06
    TEST1_REG_ADDR = 0x07
    STATUS_RSSI_REG_ADDR = 0x0A
    READ_CHANNEL_REG_ADDR = 0x0B
    RDSA_REG_ADDR = 0x0C
    RDSB_REG_ADDR = 0x0D
    RDSC_REG_ADDR = 0x0E
    RDSD_REG_ADDR = 0x0F

    # bit definitions
    TEST1_XOSCEN_BIT = 1 << 15
    TEST1_AHIZEN_BIT = 1 << 14
    PWR_CONF_ENABLE_BIT = 1 << 0
    PWR_CONF_DISABLE_BIT = 1 << 6
    PWR_CONF_DMUTE_BIT = 1 << 14
    PWR_CONF_SDMUTE_BIT = 1 << 15
    SYS_CONF3_VOLEXT_BIT = 1 << 8
    SYS_CONF2_BAND_BIT0 = 1 << 6
    SYS_CONF2_BAND_BIT1 = 1 << 7
    SYS_CONF2_BAND_MASK = SYS_CONF2_BAND_BIT1 | SYS_CONF2_BAND_BIT0
    SYS_CONF2_CHANNEL_SPACING_BIT0 = 1 << 4
    SYS_CONF2_CHANNEL_SPACING_BIT1 = 1 << 5
    SYS_CONF2_CHANNEL_SPACING_MASK = SYS_CONF2_CHANNEL_SPACING_BIT1 | SYS_CONF2_CHANNEL_SPACING_BIT0
    SYS_CONF1_DEEMPHASIS_BIT = 1 << 11
    SYS_CONF1_RDS_EN_BIT = 1 << 12
    SYS_CONF2_VOLUME_MASK = 0x0F
    CHANNEL_MASK = 0x3FF
    CHANNEL_TUNE_BIT = 1 << 15
    STATUS_RSSI_RDSR_BIT = 1 << 15
    STATUS_RSSI_STC_BIT = 1 << 14


    def __init__(self, bpInterface='', bpBaudRate=115200):

        if '' == bpInterface or 0 == bpBaudRate:
            raise UserWarning('Invalid Bus Pirate interface parameters!')

        self.i2c = I2C(connect=False)
        self.__bpInterface = bpInterface
        self.__bpBaudRate = bpBaudRate
        self.__powerOn = False  # If set to True then power will be turned on
        self.__pullupsOn = False

        # should be updated after successful calling to enable()
        self.__shadowRegs = [0x00] * 16 # at index 0 is address 0x00

    def __conv16bitToList(self, data):
        """
        Converts 16 bits value to list of bytes, the upper byte will return at index 0
        :param data:
        :return:
        """
        return [(data >> 8) & 0xFF, data & 0xFF]


    def __conv16bitListToTransferList(self, dataList):
        if (len(dataList) > 0):
            outList = [0x00] * (len(dataList) * 2)

            idx = 0
            for item in dataList:
                outList[idx] = (item >> 8) & 0xFF #MSByte first
                outList[idx + 1] = item & 0xFF
                idx += 2

            return outList
        else:
            return []


    def __i2cWriteRegisters(self):
        """
        Note that SI4703 always starts writing from address of 0x02. After each STOP symbol the writing address counter
        is reset.
        :return:
        """
        self.i2c.port.reset_output_buffer()
        self.i2c.port.reset_input_buffer()

        self.i2c.start()

        #fixme don't save to 0x08 register
        # only addresses from 0x02 to 0x08 should be written
        # divide writing registres in part of 2 (in case when only 0x00 will be sent to pevent parse that as the enter to te scripting mode

        responses = []
        totalLength = 0

        # write the write address and the first register
        writeFirstList = [SI4703.SI4703_I2C_WR_ADDR,
                     *self.__conv16bitListToTransferList([self.__shadowRegs[0x02]])]
        responses += self.i2c.transfer(writeFirstList)
        totalLength += len(writeFirstList)

        # write the rest registers
        for regStart in range(0x03, 0x08+1, 2):
            writeList = self.__conv16bitListToTransferList(self.__shadowRegs[regStart:regStart+2])
            responses += self.i2c.transfer(writeList)
            totalLength += len(writeList)

        self.i2c.stop()

        # check response
        return responses.count(True) == totalLength

    def __i2cReadFromAddr(self, regAddress, numBytesToRead):
        """

        :param regAddress:      Register address to read from
        :param numBytesToRead:  Number bytes to read from the device
        :return:    False if couldn't read data, or bytearray if data have been read successfully
        """

        #fixme or delete, this should follow SI4703 reading procedure!
        maxAccessibleAddr = 0x0F
        if regAddress <= maxAccessibleAddr:

            self.i2c.port.flushInput()
            self.i2c.start()
            writeData = []  # intentionally empty, don't write anything rather than address -> this
                            # action should change internal register index
            response = self.i2c.transfer([
                SI4703.SI4703_I2C_WR_ADDR,
                regAddress,
                *writeData
            ])

            # check response
            writeLen = len(writeData) + 2 # +2 for dev addr, and reg addr
            if len(response) == writeLen and response == [True] * writeLen:
                # now data can be read from device
                return self.i2c.write_then_read(1, numBytesToRead, [SI4703.SI4703_I2C_RD_ADDR])

        return False

    def __readAllRegisters(self):
        """
        Note that reading always starts from the 0x0A address, automatically wraps to the 0x00. Each STOP symbol
        resets the SI4703 read address
        :return:
        """

        self.i2c.port.reset_output_buffer()
        self.i2c.port.reset_input_buffer()

        # sends the SI4703 read address, then read 32 bytes of data
        regsBytes = self.i2c.write_then_read(1, len(self.__shadowRegs)*2, [SI4703.SI4703_I2C_RD_ADDR])

        if len(regsBytes) < 32:
            raise ProtocolError("Couldn't read all registers (32 bytes) of data")

        # convert addresses from 0x0A to 0x0F - these are at first indexes of received array
        idxArray = 0
        idxShadowList = 0x0A
        while idxArray < (6 * 2) - 1:
            self.__shadowRegs[idxShadowList] = (regsBytes[idxArray] << 8) | regsBytes[idxArray + 1]
            idxArray += 2
            idxShadowList += 1

        # converts addresses from 0x00 to 0x09 - these are behind the first 12 bytes of received array
        idxArray = 6 * 2
        idxShadowList = 0
        while idxArray < (6 * 2) + (10 * 2) - 1:
            self.__shadowRegs[idxShadowList] = (regsBytes[idxArray] << 8) | regsBytes[idxArray + 1]
            idxArray += 2
            idxShadowList += 1


        # regsBytes = self.__i2cReadFromAddr(SI4703.DEV_ID_REG_ADDR, len(self.__shadowRegs) * 2)
        # if False == regsBytes:
        #     raise ProtocolError("Couldn't create shadow registers")
        # else:
        #     idxArray = 0
        #     idxShadowList = 0
        #     while idxArray < len(regsBytes) - 1:
        #         self.__shadowRegs[idxShadowList] = (regsBytes[idxArray] << 8) | regsBytes[idxArray + 1]
        #         idxArray += 2
        #         idxShadowList += 1


    def getIdentificationString(self):
        ID_PN = int((self.__shadowRegs[0] >> 12) & 0x0F)
        ID_MF = int(self.__shadowRegs[0] & 0x0FFF)
        REV = int((self.__shadowRegs[1] >> 10) & 0x3F)
        DEV = int((self.__shadowRegs[1] >> 6) & 0x0F)
        FIRM = int(self.__shadowRegs[1] & 0x3F)

        str = 'Part Number = ' + hex(ID_PN) + '\n'
        str += 'Manufacturer ID =' + hex(ID_MF) + '\n'
        str += 'Chip Ver (rev) = ' + bin(REV) + ' ' + hex(REV) + '\n'
        str += 'Device  = ' + bin(DEV) + '\n'
        str += 'FIRMWARE = ' + bin(FIRM) + '\n'
        return str

    def reset(self):
        """
        Performs reset of SI4703 by toggling RST pin and ensures set module to use I2C
        :return:
        """
        #todo

    def initI2C(self, enablePower=False, enablePullUp=False):
        """
        Goes from inactive to the powerdown mode and configures it to use 2-wire interface, then initializes the I2C interface of BusPirate
        :return:

        Raises
        -------
        The same what I2C.speed, BBIO_base.connect() could raise, i2c.enter()
        """

        ################################################################################################################
        # Initializes the BP I2C interface
        ################################################################################################################
        self.i2c.connect(self.__bpInterface, self.__bpBaudRate, 0.2)
        self.i2c.enter()
        self.i2c.speed = '100kHz'

        if True == enablePower:
            self.__powerOn = True
        if True == enablePullUp:
            self.__pullupsOn = True


        # configure peripherals -> si4703 need enter to power own mode from inactive mode
        # setting rst ine will force the si4703 to enter to powerdown mode
        peripheralsDict = {'POWER' : int(self.__powerOn),
                          'PULLUPS' : int(self.__pullupsOn),
                          'AUX' : 0, # set LOW (RST pin)
                          'CS' : 0 # pull low SDA line before setting RST pin high -> enter to double
                        }
        self.i2c.configure_peripherals(peripheralsDict)
        time.sleep(0.05)

        peripheralsDict['AUX'] = 1 # set RST line HIGH (no reset)
        self.i2c.configure_peripherals(peripheralsDict)
        time.sleep(0.1)

        peripheralsDict['CS'] = 1
        self.i2c.configure_peripherals(peripheralsDict)
        # command BP to use CS as an HiZ anyway
        self.i2c.aux(0x22)  # cs HiZ

        time.sleep(0.3) #time for entering to power down mode


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



    def enable(self):
        """
        Powerup the device.
        :return:
        """
        #note see p5 of AN230 (rev0.9) point 6,7

        # create shadow registers by reading registers from the device
        self.__readAllRegisters()


        # enable internal crystal oscillator
        self.__shadowRegs[SI4703.TEST1_REG_ADDR] |= (SI4703.TEST1_XOSCEN_BIT | 0x0100)  # hex value specified by datasheet
        if False == self.__i2cWriteRegisters():
            raise ProtocolError("Couldn't enable internal oscillator")

        # wait for stabilize the clock
        time.sleep(0.700)

        # write ENABLE = 1 and DISABLE = 0
        self.__shadowRegs[SI4703.PWR_CONF_REG_ADDR] |= SI4703.PWR_CONF_ENABLE_BIT
        self.__shadowRegs[SI4703.PWR_CONF_REG_ADDR] &= ~SI4703.PWR_CONF_DISABLE_BIT
        if False == self.__i2cWriteRegisters():
            raise ProtocolError("Couldn't set ENABLE bit")

        # wait some time after power up
        time.sleep(0.3)

        # again create shadow registers -> after power up
        self.__readAllRegisters()

        # apply settings for Europe
        if False == self.setBand(locale='US_Europe', postponeI2CWrite=True):
            raise ProtocolError("Couldn't enable module. Reason: couldn't set band.")

        if False == self.setChannelSpacing(channelSpacing='100kHz', postponeI2CWrite=True):
            raise ProtocolError("Couldn't enable module. Reason: couldn't set channel spacing.")

        if False == self.setDeemphasis('50us', postponeI2CWrite=True):
            raise ProtocolError("Couldn't enable module. Reason: couldn't set deemphasis.")

        if False == self.__i2cWriteRegisters():
            raise ProtocolError("Couldn't write Europe configuration.")


    def mute(self, doMute=False):
        """
        :param doMute:
        If is True then mute, disable mute otherwise
        :return:
        None
        """
        # todo
    def setStereMonoBlendLevel(self):
        # todo
        pass


    def setSoftMute(self):
        #todo
        pass


    def setVolExt(self, value=False):
        """
        Sets VOLEXT bit
        :param value:
        :return: True on success
        """
        # Note, that setting is available only in firmware 16 or later
        # fixme check firmware version here

        sysConf3 = self.__shadowRegs[SI4703.SYS_CONF3_REG_ADDR]\

        if (True == value):
            sysConf3 |= SI4703.SYS_CONF3_VOLEXT_BIT
        else:
            sysConf3 &= ~SI4703.SYS_CONF3_VOLEXT_BIT

        if True == self.__i2cWrite(SI4703.SYS_CONF3_REG_ADDR, [sysConf3]):
            # ok, so update shadow register
            self.__shadowRegs[SI4703.SYS_CONF3_REG_ADDR] = sysConf3
            return True

        return False


    def setSeekThreshold(self):
        pass

    def setBand(self, locale='US_Europe', postponeI2CWrite=False):
        """
        'US_Europe'
        'Japan_Wide'
        'Japan'

        :param locale:
        :param postpone:
        :return:
        """

        valuesMap = {'US_Europe'  : 0b00,
                     'Japan_Wide' : 0b01,
                     'Japan'      : 0b10}
        if locale in valuesMap:
            self.__shadowRegs[SI4703.SYS_CONF2_REG_ADDR] &= ~SI4703.SYS_CONF2_BAND_MASK
            self.__shadowRegs[SI4703.SYS_CONF2_REG_ADDR] |= (int(valuesMap[locale]) << 6)

            if False == postponeI2CWrite:
                return self.__i2cWriteRegisters()
            else:
                return True

        return False


    def setChannelSpacing(self, channelSpacing ='100kHz', postponeI2CWrite=False):
        """


        :param channelSpacing:
            200kHz - USA, Australia
            100kHz - Europe, Japan
            50kHz
        :return:
        """
        channelSpacingMap = {'200kHz' : 0b00,
                             '100kHz' : 0b01,
                             '50kHz'  : 0b10}

        if channelSpacing in channelSpacingMap:
            self.__shadowRegs[SI4703.SYS_CONF2_REG_ADDR] &= ~SI4703.SYS_CONF2_CHANNEL_SPACING_MASK
            self.__shadowRegs[SI4703.SYS_CONF2_REG_ADDR] |= (int(channelSpacingMap[channelSpacing])) << 4

            if False == postponeI2CWrite:
                return self.__i2cWriteRegisters()
            else:
                return True

        return False


    def setDeemphasis(self, timeConstant='50us', postponeI2CWrite=False):
        """
        :param time_constant:
            '50us' - Europe, Japan, Australia
            '75us' - USA
        :return:
        """
        timeConstantList = {'50us', '75us'}

        if timeConstant in timeConstantList:
            if '50us' == timeConstant:
                self.__shadowRegs[SI4703.SYS_CONF1_REG_ADDR] |= SI4703.SYS_CONF1_DEEMPHASIS_BIT
            else:
                self.__shadowRegs[SI4703.SYS_CONF1_REG_ADDR] &= ~SI4703.SYS_CONF1_DEEMPHASIS_BIT

            if False == postponeI2CWrite:
                return self.__i2cWriteRegisters()
            else:
                return True

        return False


    def enableRDS(self, enable=True):
        sysConf1 = self.__shadowRegs[SI4703.SYS_CONF1_REG_ADDR]
        if True == enable:
            sysConf1 |= SI4703.SYS_CONF1_RDS_EN_BIT
        else:
            sysConf1 &= ~SI4703.SYS_CONF1_RDS_EN_BIT

        if True == self.__i2cWrite(SI4703.SYS_CONF1_REG_ADDR, [sysConf1]):
            # update shadow reg
            self.__shadowRegs[SI4703.SYS_CONF1_REG_ADDR] = sysConf1
            return True

        return False


    def mute(self, mute=False):
        #fixme DMUTE bit
        return False

    def monoAudio(self, setMono=False):
        #fixme MONO bit
        return False


    def setVolume(self, volume=5, postponeI2CWrite=False):
        """
        range 0 (muted) - 15 (max)
        see also VOLEXT bit

        :param volume:
        :return:
        """
        if volume < 0:
            volume = 0
        elif volume > 15:
            volume = 8

        self.__shadowRegs[SI4703.SYS_CONF2_REG_ADDR] &= ~SI4703.SYS_CONF2_VOLUME_MASK
        self.__shadowRegs[SI4703.SYS_CONF2_REG_ADDR] |= volume

        if False == postponeI2CWrite:
            return self.__i2cWriteRegisters()
        else:
            return True


    def seek(self, direction='up', autoWrapAtBandLimit=False):
        #todo
        return False


    def setChannel(self):

        #fixme, this disables mute option
        self.__shadowRegs[SI4703.PWR_CONF_REG_ADDR] |= SI4703.PWR_CONF_DMUTE_BIT

        # just fixed,  set to 96MHz in europe
        self.__shadowRegs[SI4703.CHANNEL_REG_ADDR] |= SI4703.CHANNEL_TUNE_BIT
        self.__shadowRegs[SI4703.CHANNEL_REG_ADDR] &= ~SI4703.CHANNEL_MASK
        # self.__shadowRegs[SI4703.CHANNEL_REG_ADDR] |= 85 # rmffm
        self.__shadowRegs[SI4703.CHANNEL_REG_ADDR] |= 85  # eska


        if True == self.__i2cWriteRegisters():
            # wait at least 60ms
            time.sleep(0.3)

            # read registers from the SI4703 to get fresh status bits
            self.__readAllRegisters()

            # check if Tune operation is complete
            if 0 != (self.__shadowRegs[SI4703.STATUS_RSSI_REG_ADDR] & SI4703.STATUS_RSSI_STC_BIT):
                # ok, tune operation is complete, so clear tune bit
                self.__shadowRegs[SI4703.CHANNEL_REG_ADDR] &= ~SI4703.CHANNEL_TUNE_BIT
                if False == self.__i2cWriteRegisters():
                    raise ProtocolError("Couldn't clear TUNE bit")
            else:
                raise ProtocolError("One attempt to wait for STC bit set failed")


            return True
        else:
            raise ProtocolError("Couldn't save channel")
