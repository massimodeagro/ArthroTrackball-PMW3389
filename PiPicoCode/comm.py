import time
import register
import SROM
import struct


def read_reg(SPI, reg_addr, cs):
    cs(0)

    # send adress of the register, with MSBit = 0 to indicate it's a read
    SPI.write(bytearray([reg_addr & 0x7f]))
    time.sleep_us(100)  # tSRAD
    # read data
    data = SPI.read(1)

    time.sleep_us(1)  # tSCLK-NCS for read operation is 120ns
    cs(1)
    time.sleep_us(19)  # tSRW/tSRR (=20us) minus tSCLK-NCS

    return data


def write_reg(SPI, reg_addr, data, cs):
    cs(0)

    # send adress of the register, with MSBit = 1 to indicate it's a write
    SPI.write(bytearray([reg_addr | 0x80, data]))

    time.sleep_us(20)  # tSCLK-NCS for write operation
    cs(1)
    time.sleep_us(100)  # tSWW/tSWR (=120us) minus tSCLK-NCS. Could be shortened, but is looks like a safe lower bound


def upload_firmware(SPI, cs):
    # send the firmware to the chip, cf p.18 of the datasheet
    print("Uploading firmware...");

    # Write 0 to Rest_En bit of Config2 register to disable Rest mode.
    write_reg(SPI, register.Config2, 0x20, cs)

    # write 0x1d in SROM_enable reg for initializing
    write_reg(SPI, register.SROM_Enable, 0x1d, cs)

    # wait for more than one frame period
    time.sleep_ms(10)  # assume that the frame rate is as low as 100fps... even if it should never be that low

    # write 0x18 to SROM_enable to start SROM download
    write_reg(SPI, register.SROM_Enable, 0x18, cs)

    # write the SROM file (=firmware data)
    cs(0)
    SPI.write(bytearray([register.SROM_Load_Burst | 0x80]))  # write burst destination adress
    time.sleep_us(15)

    # send all bytes of the firmware
    for bit in SROM.PROGMEM:
        SPI.write(bytearray([bit]))
        time.sleep_us(15)

    # Read the SROM_ID register to verify the ID before any other register reads or writes.
    read_reg(SPI, register.SROM_ID, cs)

    # Write 0x00 to Config2 register for wired mouse or 0x20 for wireless mouse design.
    write_reg(SPI, register.Config2, 0x00, cs)

    # set initial CPI resolution
    # write_reg(SPI, register.Res_H, 0x15, ncs)
    # write_reg(SPI, register.Res_L, 0x15, ncs)

    cs(1)


def performStartup(SPI, cs):
    cs(1)  # ensure that the serial port is reset
    cs(0)  # ensure that the serial port is reset
    cs(1)  # ensure that the serial port is reset
    write_reg(SPI, register.Power_Up_Reset, 0x5a, cs)  # force reset
    time.sleep_ms(50)  # wait for it to reboot
    # read registers 0x02 to 0x06 (and discard the data)
    read_reg(SPI, register.Motion, cs)
    read_reg(SPI, register.Delta_X_L, cs)
    read_reg(SPI, register.Delta_X_H, cs)
    read_reg(SPI, register.Delta_Y_L, cs)
    read_reg(SPI, register.Delta_Y_H, cs)
    # upload the firmware
    upload_firmware(SPI, cs)
    time.sleep_ms(10)


def UpdatePointer(SPI, cs):
    xdat = [None, None]
    ydat = [None, None]

    # write 0x01 to Motion register and read from it to freeze the motion values and make them available
    write_reg(SPI, register.Motion, 0x01, cs)
    read_reg(SPI, register.Motion, cs)

    xdat[0] = read_reg(SPI, register.Delta_X_L, cs)
    xdat[1] = read_reg(SPI, register.Delta_X_H, cs)

    ydat[0] = read_reg(SPI, register.Delta_Y_L, cs)
    ydat[1] = read_reg(SPI, register.Delta_Y_H, cs)

    # x = xdat[1][0] << 8 | xdat[0][0]
    # y = ydat[1][0] << 8 | ydat[0][0]
    x = struct.unpack("<h", bytes([xdat[0][0], xdat[1][0]]))[0]
    y = struct.unpack("<h", bytes([ydat[0][0], ydat[1][0]]))[0]

    return x, y


def UpdatePointerBurst(SPI, cs, read_buf, write_buf):
    write_reg(SPI, register.Motion_Burst, 0x01, cs)
    write_buf[0] = register.Motion_Burst & 0x7f
    cs(0)
    SPI.write(write_buf)
    time.sleep_us(35)
    SPI.readinto(read_buf)
    cs(1)
    time.sleep_us(1)

    x = struct.unpack("<h", bytes([read_buf[2], read_buf[3]]))[0]
    y = struct.unpack("<h", bytes([read_buf[4], read_buf[5]]))[0]
    return x, y


def dispRegisters(SPI, cs):
    cs(0)
    prodID = read_reg(SPI, register.Product_ID, cs)
    revprodID = read_reg(SPI, register.Inverse_Product_ID, cs)

    print(prodID, revprodID)
