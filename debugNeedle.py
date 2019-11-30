#!/usr/bin/env python
from mpl_toolkits.mplot3d import Axes3D, art3d
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import sys
from math import sin, cos


#!/usr/bin/env python
from mpl_toolkits.mplot3d import Axes3D, art3d
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import sys
from math import sin, cos


# Plot a path in R3 with a unit square obstacle centered at the origin
def plotR2(paths, environment=2):
	fig = plt.figure()
	ax = fig.gca()
	for path in paths:
		# Plotting the path
		X = [p[0] for p in path]
		Y = [p[1] for p in path]
		ax.plot(X, Y)

	if(environment == 1):
		x = 1.0
		y = 7.0
		w = 3.0
		h = 2.0
		ax.add_patch(patches.Polygon([(x, y), (x+w, y), (x+w, y+h), (x, y+h)], fill=True, color='0.20', alpha=0.8))
		x = 3.0
		y = 3.0
		w = 3.0
		h = 3.0
		ax.add_patch(patches.Polygon([(x, y), (x+w, y), (x+w, y+h), (x, y+h)], fill=True, color='0.20', alpha=0.8))
	else:
		x = 3.0
		y = 1.0
		w = 2.0
		h = 7.0
		ax.add_patch(patches.Polygon([(x, y), (x+w, y), (x+w, y+h), (x, y+h)], fill=True, color='0.20', alpha=0.8))
		x = 6.0
		y = 3.0
		w = 3.0
		h = 3.0
		ax.add_patch(patches.Polygon([(x, y), (x+w, y), (x+w, y+h), (x, y+h)], fill=True, color='0.20', alpha=0.8))

	ax.add_patch(patches.Circle((7.5, 7.5), radius = 0.5, fill=True, color='g', alpha=0.3))

	ax.set_ylim([0, 10])
	ax.set_xlim([0, 10])
	plt.gca().set_aspect('equal', adjustable='box')
	plt.show()


# Read the cspace definition and the path from filename
def readPath(filename):
	data = []
	lines = []
	for line in open(filename):
		if(len(line.rstrip()) > 0):
			lines.append(line.rstrip())
		else:
			data.append([[float(x) for x in line.split(',')] for line in lines[1:]])
			lines = []
	return data

if __name__ == '__main__':
	
	filename = 'solution_path.txt'
	paths = readPath(filename)
	plotR2(paths)


