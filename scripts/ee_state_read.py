import sys
import matplotlib.pyplot as plt
import numpy as np

def main():
	file_raw = open(sys.argv[1], 'r')
	file_state = open(sys.argv[2], 'r')

	xRaw, yRaw = [], []
	xPos, yPos = [], []
	xVel, yVel = [], []

	fl = file_raw.readlines()
	for i in xrange(len(fl)):
		if 'x' in fl[i]:
			xRaw.append(float(fl[i][3:]) + 0.6)
			yRaw.append(float(fl[i+1][3:]) + 0.4)

	fl = file_state.readlines()
	for i in xrange(len(fl)):
		if 'pose' in fl[i]:
			xPos.append(float(fl[i+2][7:]))
			yPos.append(float(fl[i+3][7:]))
			xVel.append(float(fl[i+12][7:]))
			yVel.append(float(fl[i+13][7:]))

	fig, ax = plt.subplots(1,3)
	ax[0].scatter(xRaw, yRaw, label="Input Data")
	ax[0].plot(xPos, yPos, c='orange', label="End Effector Trajectory")
	ax[0].set_xlabel("x(m)")
	ax[0].set_ylabel("y(m)")
	ax[0].grid()
	ax[0].legend()

	ax[1].scatter(np.linspace(0, len(xVel), len(xVel)), xVel, label="x-Velocity")
	ax[1].set_xlabel("n(samples)")
	ax[1].set_ylabel("x_vel")
	ax[1].grid()
	ax[1].legend()

	ax[2].scatter(np.linspace(0, len(yVel), len(yVel)), yVel, label="y-Velocity")
	ax[2].set_xlabel("n(samples)")
	ax[2].set_ylabel("y_vel")
	ax[2].grid()
	ax[2].legend()

	plt.show()


main()