import numpy as np
import matplotlib.pylab as plt

filename="H:\Quantum Dots\\2018-12-14\Spectrum\\sample_4_1.txt"


data = []
with open(filename) as image:
    for line in image:
        if not line.startswith('#'):
            data.append(float(line))
        else:
            print(line)

im = np.reshape(
    np.array(data), (127, 1024))
print("Shape:", im.shape)

im=im[55:75,:]
#im=im[57:58,:]

x = list(range(1024))
y = np.sum(im, axis=0)/np.size(im,0)
y=y[::-1]

plt.plot(x,y)
plt.show()