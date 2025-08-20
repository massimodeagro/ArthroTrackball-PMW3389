import time

from machine import SPI, Pin
import register
import comm
import matrixAlgebra as ma

# Sensors Definitions

S0p = [15.26, 8.8, -6.4]
S0xs = [-0.5, 0.87, 0]
S0ys = [0.29, 0.18, 0.94]

S1p = [-15.26, 8.8, -6.4]
S1xs = [-0.5, -0.87, 0]
S1ys = [-0.29, 0.18, 0.94]

res = 0.00508

read_buf = memoryview(bytearray(6))
write_buf = memoryview(bytearray(1))

# Triplets Definitions

# TRIPLET 1
Triplets = [{'p_i': None, 's_i': None,
             'p_j': None, 's_j': None,
             'p_k': None, 's_k': None,
             'cross_i': None,
             'cross_j': None,
             'cross_k': None,
             'A': None, 'Ainv': None, 'Adet': None,
             'reads': [None, None, None],
             'w1': None} for i in range(4)]

# Triplet 0
Triplets[0]['p_i'] = S0p
Triplets[0]['s_i'] = S0xs
Triplets[0]['p_j'] = S0p
Triplets[0]['s_j'] = S0ys
Triplets[0]['p_k'] = S1p
Triplets[0]['s_k'] = S1xs
Triplets[0]['reads'] = ['x0', 'y0', 'x1']

# Triplet 1
Triplets[1]['p_i'] = S0p
Triplets[1]['s_i'] = S0xs
Triplets[1]['p_j'] = S0p
Triplets[1]['s_j'] = S0ys
Triplets[1]['p_k'] = S1p
Triplets[1]['s_k'] = S1ys
Triplets[1]['reads'] = ['x0', 'y0', 'y1']

# Triplet 2
Triplets[2]['p_i'] = S0p
Triplets[2]['s_i'] = S0ys
Triplets[2]['p_j'] = S1p
Triplets[2]['s_j'] = S1xs
Triplets[2]['p_k'] = S1p
Triplets[2]['s_k'] = S1ys
Triplets[2]['reads'] = ['y0', 'x1', 'y1']

# Triplet 3
Triplets[3]['p_i'] = S0p
Triplets[3]['s_i'] = S0xs
Triplets[3]['p_j'] = S1p
Triplets[3]['s_j'] = S1xs
Triplets[3]['p_k'] = S1p
Triplets[3]['s_k'] = S1ys
Triplets[3]['reads'] = ['x0', 'x1', 'y1']

for i in range(4):
    Triplets[i]['cross_i'] = ma.crossProduct(Triplets[i]['p_i'], Triplets[i]['s_i'])
    Triplets[i]['cross_j'] = ma.crossProduct(Triplets[i]['p_j'], Triplets[i]['s_j'])
    Triplets[i]['cross_k'] = ma.crossProduct(Triplets[i]['p_k'], Triplets[i]['s_k'])

    Triplets[i]['A'] = [[0, 0, 0],
                        [0, 0, 0],
                        [0, 0, 0]]

    for n in range(3):
        Triplets[i]['A'][0][n] = Triplets[i]['cross_i'][n]
        Triplets[i]['A'][1][n] = Triplets[i]['cross_j'][n]
        Triplets[i]['A'][2][n] = Triplets[i]['cross_k'][n]

    Triplets[i]['Adet'] = ma.calculate_Adet(Triplets[i]['A'])
    Triplets[i]['w1'] = abs(Triplets[i]['Adet'])
    Triplets[i]['Ainv'] = ma.calculate_Ainv(Triplets[i]['A'])

spi0 = SPI(1, baudrate=2_000_000, polarity=1, phase=1, firstbit=SPI.MSB, sck=Pin(10), mosi=Pin(11), miso=Pin(8))
spi1 = SPI(0, baudrate=2_000_000, polarity=1, phase=1, firstbit=SPI.MSB, sck=Pin(18), mosi=Pin(19), miso=Pin(16))

cs0 = Pin(9, mode=Pin.OUT, value=1)
cs1 = Pin(17, mode=Pin.OUT, value=1)
rst0 = Pin(13, mode=Pin.OUT, value=1)
rst1 = Pin(21, mode=Pin.OUT, value=1)
mt0 = Pin(12, mode=Pin.IN)
mt1 = Pin(20, mode=Pin.IN)

# Initialization
comm.performStartup(SPI=spi0, cs=cs0)
time.sleep(1)
comm.dispRegisters(SPI=spi0, cs=cs0)
print("product ID (expected 71): ")
print(comm.read_reg(SPI=spi0, reg_addr=register.Product_ID, cs=cs0))

comm.performStartup(SPI=spi1, cs=cs1)
time.sleep(1)
comm.dispRegisters(SPI=spi1, cs=cs1)
print("product ID (expected 71): ")
print(comm.read_reg(SPI=spi1, reg_addr=register.Product_ID, cs=cs1))
time.sleep(1)

ws = [None] * 4
w1s = [None] * 4
w2s = [None] * 4
reads = {'x0': 0, 'y0': 0, 'x1': 0, 'y1': 0}

for i in range(1_000):
    pix_x0, pix_y0 = comm.UpdatePointerBurst(SPI=spi0, cs=cs0,
                                             read_buf=read_buf, write_buf=write_buf)
    pix_x1, pix_y1 = comm.UpdatePointerBurst(SPI=spi1, cs=cs1,
                                        read_buf=read_buf, write_buf=write_buf)

    reads['x0'] = pix_x0 * res
    reads['y0'] = pix_y0 * res
    reads['x1'] = pix_x1 * res
    reads['y1'] = pix_y1 * res

    for i in range(4):
        a, b, c = reads[Triplets[i]['reads'][0]], reads[Triplets[i]['reads'][1]], reads[Triplets[i]['reads'][2]]
        ws[i] = ma.calculate_w(a, b, c, Triplets[i]['Ainv'])
        w1s[i] = Triplets[i]['w1']
        w2s[i] = ma.calculate_w2_fast(a, b, c, Triplets[i]['Ainv'])

    avg_w = ma.calculate_avg_w(ws, w1s, w2s)

    print(avg_w)