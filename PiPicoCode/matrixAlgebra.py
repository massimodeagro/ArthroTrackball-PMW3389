def crossProduct(a, b):
    result = [a[1] * b[2] - a[2] * b[1],
              a[2] * b[0] - a[0] * b[2],
              a[0] * b[1] - a[1] * b[0]]
    return result


def calculate_Adet(A):
    Adet = A[0][0] * (A[1][1] * A[2][2] - A[2][1] * A[1][2]) - \
           A[0][1] * (A[1][0] * A[2][2] - A[2][0] * A[1][2]) + \
           A[0][2] * (A[1][0] * A[2][1] - A[2][0] * A[1][1])
    return Adet


def calculate_Ainv(A):
    Adet = calculate_Adet(A)
    Ainv = [[None, None, None],
            [None, None, None],
            [None, None, None]]

    if Adet == 0:
        return Ainv  # Matrix is not invertible if determinant is zero

    Ainv[0][0] = (A[1][1] * A[2][2] - A[2][1] * A[1][2]) / Adet
    Ainv[0][1] = (A[0][2] * A[2][1] - A[0][1] * A[2][2]) / Adet
    Ainv[0][2] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) / Adet

    Ainv[1][0] = (A[1][2] * A[2][0] - A[1][0] * A[2][2]) / Adet
    Ainv[1][1] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) / Adet
    Ainv[1][2] = (A[1][0] * A[0][2] - A[0][0] * A[1][2]) / Adet

    Ainv[2][0] = (A[1][0] * A[2][1] - A[2][0] * A[1][1]) / Adet
    Ainv[2][1] = (A[2][0] * A[0][1] - A[0][0] * A[2][1]) / Adet
    Ainv[2][2] = (A[0][0] * A[1][1] - A[1][0] * A[0][1]) / Adet

    return Ainv


def calculate_w(vsi, vsj, vsk, Ainv):
    w = [None, None, None]
    # calculate the w vector with matrix multiplication
    for i in range(3):
        w[i] = Ainv[i][0] * vsi + Ainv[i][1] * vsj + Ainv[i][2] * vsk

    return w


def calculate_w2(vsi, vsj, vsk, Ainv):
    w2 = [None, None, None]

    for i in range(3):
        a1, a2, a3 = Ainv[i][0], Ainv[i][1], Ainv[i][2]
        try:
            w2[i] = abs(a1 * vsi + a2 * vsj + a3 * vsk) / (abs(a1 * vsi) + abs(a2 * vsj) + abs(a2 * vsk))
        except ZeroDivisionError:
            w2[i] = 0

    return w2

def calculate_w2_fast(vsi, vsj, vsk, Ainv):
    w2 = []
    for i in range(3):
        a1, a2, a3 = Ainv[i]
        num = abs(a1 * vsi + a2 * vsj + a3 * vsk)
        den = abs(a1 * vsi) + abs(a2 * vsj) + abs(a2 * vsk) or 1  # Avoid division by zero
        w2.append(num/den)
    return w2


# Function to calculate the weighted average of angular velocity
def calculate_avg_w(ws, w1s, w2s):
    xn = []  # numerators
    xd = []  # denominators
    yn = []
    yd = []
    zn = []
    zd = []
    for w, w1, w2 in zip(ws, w1s, w2s):
        xn.append(w1 * w2[0] * w[0])
        xd.append(w1 * w2[0])
        yn.append(w1 * w2[1] * w[1])
        yd.append(w1 * w2[1])
        zn.append(w1 * w2[2] * w[2])
        zd.append(w1 * w2[2])

    if sum(xd) != 0:
        wx = sum(xn) / sum(xd)
    else:
        wx = 0
    if sum(yd) != 0:
        wy = sum(yn) / sum(yd)
    else:
        wy = 0
    if sum(zd) != 0:
        wz = sum(zn) / sum(zd)
    else:
        wz = 0

    if wx != wx:
        wx = 0
    if wy != wy:
        wy = 0
    if wz != wz:
        wz = 0

    return [wx, wy, wz]