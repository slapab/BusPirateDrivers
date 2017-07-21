import sys

sys.path.append('../../pyBusPirateLite/') #append the path where the pyBusPirateLite is placed
from pyBusPirateLite.onewire import *



class DS18B20:
    def __init__(self, bpInterface='', bpBaudRate=115200):
        if '' == bpInterface or 0 == bpBaudRate:
            raise UserWarning('Invalid Bus Pirate interface parameters!')

        self.__bpInterface = bpInterface
        self.__bpBaudRate = bpBaudRate
        self.one_wire = OneWire(self.__bpInterface, self.__bpBaudRate, True)

        # enable power and pullups
        self.one_wire.configure_peripherals({'POWER' : 1, 'PULLUPS' : 1})
        time.sleep(0.1)

    def find_roms(self):
        """

        :return: List of found ROMs
        """
        return self.one_wire.rom_search()



    def get_temp(self, romBytes=None):
        if romBytes is not None and len(romBytes) != 8:
            raise ProtocolError("ROM list must contain exactly 8 bytes")

        if romBytes is None:
            # send conversion command
            self.one_wire.write_data(skipRom=True, dataList=[0x44])
        else:
            # send match rom, rom address and conversion command
            self.one_wire.write_data(skipRom=False, romList=romBytes, dataList=[0x44])

        # wait for end of conversion
        time.sleep(0.75)

        retBytes = bytes()
        if romBytes is None:
            retBytes = self.one_wire.read_data(skipRom=True, readCommand=0xBE, bytesToRead=9)
        else:
            retBytes = self.one_wire.read_data(skipRom=False, romList=romBytes, readCommand=0xBE, bytesToRead=9)

        if len(retBytes) == 9:
            val = ((retBytes[1] & 0xFF) << 8) | (retBytes[0] & 0xFF)

            # the negative readings
            if val & 0x8000:
                val = -((val ^ 0xFFFF) + 1) #2's complement

            return val * 0.0625

        return None

    def poweroff(self):
        self.one_wire.exit()

