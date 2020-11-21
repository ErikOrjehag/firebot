import smbus
import time

class TPA81():
    TPA81_SOFTVER = 0
    TPA81_REGADDR = 0 # Same as TPA81_SOFTVER
    TPA81_AMBTEMP = 1
    TPA81_PIXEL = [ 2, 3, 4, 5, 6, 7, 8, 9] #Register for pixels from 1 to 8

    def __init__(self, address=0x68, bus_num=1):
        self.bus_num = bus_num
        self.address = address
        self.bus = smbus.SMBus(bus=bus_num)

    def read_reg(self, reg_addr):
        try:
            return self.bus.read_byte_data(self.address, reg_addr) % 128
        except IOError as e:
            print("Error Reading TPA81")
            print(e)
            return 0

    def write_reg(self, reg_addr, value):
        try:
            self.bus.write_byte_data(self.address, reg_addr, value)
            time.sleep(0.1)
        except IOError:
            print("Error Writing TPA81")

    def softwareVersion(self):
        return self.read_reg(self.TPA81_SOFTVER)

    def ambientTemperature(self):
        return self.read_reg(self.TPA81_AMBTEMP)

    def pixelTemp(self, id):
        return self.read_reg(self.TPA81_PIXEL[id])

    def allPixelTemp(self):
        TempArray = []
        lenght = len(self.TPA81_PIXEL)

        for i in range(lenght):
            TempArray.append(self.pixelTemp(i))

        return TempArray

    def highestTemp(self):
        return max(self.allPixelTemp())

    def changeAddress(self, newAddr):
        """
        ID | Address
        0	 0xD0
        1	 0xD2
        2	 0xD4
        3	 0xD6
        4	 0xD8
        5	 0xDA
        6	 0xDC
        7	 0xDE
        """

        addresses = [0xD0, 0xD2, 0xD4, 0xD6, 0xD8, 0xDA, 0xDC, 0xDE]

        self.write_reg(self.TPA81_REGADDR, 0xA0)
        self.write_reg(self.TPA81_REGADDR, 0xAA)
        self.write_reg(self.TPA81_REGADDR, 0xA5)
        self.write_reg(self.TPA81_REGADDR, addresses[newAddr])

if __name__ == '__main__':

    tpa = TPA81()

    #tpa.changeAddress(0)

    print('Version: ', tpa.softwareVersion())
    #print('Ambient Temperature: ', tpa.ambientTemperature())

    #for i in range(len(tpa.TPA81_PIXEL)):
    #    print('Pixel', (i + 1), ': ', tpa.pixelTemp(i))

    #print('Highest Temp: ', tpa.highestTemp())