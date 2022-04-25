import cv2
import numpy as np

def calc_error(x, y, point1, point2):
	if point2[0]-point1[0] != 0:
		slope = (point2[1]-point1[1])/(point2[0]-point1[0])
		constant = point1[1] - (slope*point1[0])
		error = y - (slope*x + constant)
	else:
		slope = (point2[0]-point1[0])/(point2[1]-point1[1])
		constant = point1[0] - (slope*point1[1])
		error = x - (slope*y + constant)
	return error

def create_graph(pad_size = 10):
	graph = np.zeros((250,400))
	org = 250
	points1 = [(65,36),(40,115),(70,80),(150,105)]
	points2 = [(org-120,165), (org-140,200), (org-120,235), (org-80,235), (org-60,200), (org-80,165)]

	for i in range(graph.shape[0]):
		for j in range(graph.shape[1]):
			e11 = calc_error(i, j, points1[0], points1[1])
			e12 = calc_error(i, j, points1[1], points1[3])
			e13 = calc_error(i, j, points1[3], points1[0])
			e14 = calc_error(i, j, points1[2], points1[1])
			e15 = calc_error(i, j, points1[1], points1[3])
			e16 = calc_error(i, j, points1[3], points1[2])


			e21 = calc_error(i, j, points2[0], points2[1])
			e22 = calc_error(i, j, points2[1], points2[2])
			e23 = calc_error(i, j, points2[2], points2[3])
			e24 = calc_error(i, j, points2[3], points2[4])
			e25 = calc_error(i, j, points2[4], points2[5])
			e26 = calc_error(i, j, points2[5], points2[0])

			ec = 40**2 - (j-300)**2 - (i-65)**2
			if ec >= 0: # Circle
				graph[i,j] = 1

			# Abstract shape
			if e11 >= 0 and e12 <= 0 and e13 >= 0:
				graph[i,j] = 1
			if e14 >= 0 and e15 <= 0 and e16 >= 0:
				graph[i,j] = 0

			# Hexagon
			if e21 >= 0 and e22 <= 0 and e24 <= 0 and e25 >= 0 and e23 <= 0 and e26 >= 0:
				graph[i,j] = 1

	return graph
