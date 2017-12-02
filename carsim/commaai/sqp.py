import sys
import numpy as np

def squared(x, z):
    y = x * x + z
    # a = np.zeros([5, 2])
    return y, 1

if __name__ == '__main__':
    x = float(sys.argv[1])
    z = float(sys.argv[2])
    a, b = squared(x, z)
    sys.stdout.write(str(a))
    sys.stdout.write('\n*\n')
    sys.stdout.write(str(b))
    sys.stdout.write('\n*\n')
    # sys.stdout.write(str(c))
    # sys.stdout.write(str(squared(x, z)))