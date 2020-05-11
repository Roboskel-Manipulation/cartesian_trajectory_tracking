import sys
import numpy as np
import matplotlib.pyplot as plt


def main():
	file = open(sys.argv[1], 'r')
	fl = file.readlines()
	x, y, z = [], [], []
	for i in range(len(fl)):
		if 'position' in fl[i]:
			x.append(float(fl[i+1][7:]))
			y.append(float(fl[i+2][7:]))
			z.append(float(fl[i+3][7:]))

	samples = np.linspace(1, len(y), len(y))
	des = [0.3 for i in range(len(y))]
	fig = plt.figure()
	ax = plt.axes()
	ax.plot(samples, des, linestyle='--', c='orange')
	ax.plot(samples, y)
	ax.grid()
	plt.show()

main()