import cv2
import numpy as np
import time
from utils import *
from Cost import *
from plot import *

class Robot:
	orn = 0
	step = 1
	cl = 5
	rd = 1
	parent = 0
	cost = 0
	pos = [rd, rd, orn]
	rpm = [0, 0]
	L = 1.5

	def __init__(self, position, rpm):
		self.orn = position[2]
		self.pos = position
		self.rpm = rpm


	def move_possible(self, move, graph):
		mask = np.zeros(graph.shape)
		shape_y, shape_x = graph.shape
		# import pdb; pdb.set_trace()
		if move[0] < self.rd or move[0] > shape_y-1-self.rd or move[1] < self.rd or move[1] > shape_x-1-self.rd:
			return False
		for i in range(shape_y):
			for j in range(shape_x): # can optimize this
				ec = (self.rd + self.cl)**2 - (j-self.pos[1])**2 - (i-self.pos[0])**2
				if ec >= 0:
					mask[i,j] = 1
		# import pdb; pdb.set_trace()
		result = np.bitwise_and(graph.astype(np.uint8), mask.astype(np.uint8))
		if result.any() == 1:
			return False
		else:
			return True

	def action(self, storage, opn, visited, graph, goal):

		rpm1 = self.rpm[0]
		rpm2 = self.rpm[1]
		L = self.L
		length = len(storage)
		action_set = [[0, rpm1], [rpm1, 0], [rpm1, rpm1], [0, rpm2], [rpm2, 0], [rpm2, rpm2], [rpm1, rpm2], [rpm2, rpm1]]
		for i in action_set:
			# import pdb; pdb.set_trace()
			# new_action = new_cost(self, i)
			new_action = plot_curve(self, i, graph)
			step = new_action[3]
			new_y = int(new_action[1])
			new_x = int(new_action[0])
			orientation = int(new_action[2])
			if(orientation >= 360):
				orientation = orientation - 360
			if(orientation < 0):
				orientation = orientation + 360
			move = [new_y, new_x, orientation]
			print(move)
			

			if self.move_possible(move, graph):
				if not exist(move, visited):
					cost = self.cost + self.step
					storage[length] = [length, move, cost, self.parent]
					visited.append(move)
					opn.append([length, move, cost, self.parent])
					length += 1
					if move == goal:
						return storage, opn, visited, True
				else:
					location, location2, check = find(move, visited, opn)
					cost = self.cost + self.step
					if storage[location][2] > cost and check:
							storage[location][2] = cost
							storage[location][3] = self.parent
							opn[location2][2] = cost
							opn[location2][3] = self.parent
		

		return storage, opn, visited, False


def find(move, visited, opn):
	# import pdb; pdb.set_trace()
	ar_opn = np.array(opn)[:,1]
	ar_opn = np.array([np.array(k) for k in ar_opn])
	ar_opn = ar_opn.tolist()
	if move in ar_opn:
		return visited.index(move), ar_opn.index(move), True
	else:
		return visited.index(move), None, False

def exist(move, visited):
	if move in visited:
		return True
	else:
		return False

def check(coords1, coords2):
	if coords1 == coords2:
		return True
	else:
		return False

def get_newnode(opn, goal):
	# import pdb; pdb.set_trace()
	ar_opn = np.array(opn)[:,1]
	ar_opn = np.array([np.array(k) for k in ar_opn])
	dist = np.sum((ar_opn[:,:2]-goal[:2])**2, axis=1)
	idx = np.argmin(dist)
	curr_node = opn.pop(idx)
	return curr_node, opn

def a_star(goal, robot):
	graph = create_graph()
	shp_gr = graph.shape
	fig, ax = plt.subplots()
	if robot.pos[0] >= shp_gr[0]-robot.rd or robot.pos[1] >= shp_gr[1]-robot.rd or goal[0] >= shp_gr[0]-robot.rd or goal[1] >= shp_gr[1]-robot.rd or robot.pos[0] < robot.rd or robot.pos[1] < robot.rd or goal[0] < robot.rd or goal[1] < robot.rd:
		print("Nodes invalid.")
		return {}
	if graph[robot.pos[0], robot.pos[1]] == 1:
		print("Initial node in obstacle space.")
		return {}
	if graph[goal[0], goal[1]] == 1:
		print("Goal node in obstacle space.")
		return {}

	step_size = 3*robot.L
	# import pdb; pdb.set_trace()
	closed = []
	storage = {0 : [0, robot.pos, 0, 0]} # [num, coords, cost, parent]
	closed.append(robot.pos)
	opn = []

	while not check(robot.pos[:2], goal[:2]):
		storage, opn, closed, found = robot.action(storage, opn, closed, graph, goal)
		if found:
			break

		node, opn = get_newnode(opn, goal)
		
		if ((node[1][0]-goal[0])**2 + (node[1][1]-goal[1])**2) < (step_size)**2:
			break

		robot.pos = node[1]
		robot.parent = node[0]
		robot.cost = node[2]
		robot.orn = node[1][2]

	plt.grid()

	ax.set_aspect('equal')

	plt.xlim(0,graph.shape[1])
	plt.ylim(0,graph.shape[0])

	plt.title('How to plot a vector in matplotlib?',fontsize=10)
	plt.savefig('plot')
	plt.show()
	
	plt.close()
	return storage


def origin_shift(initial, goal):
	graph = create_graph()
	origin_shift = graph.shape[0]-1
	initial[0] = origin_shift - initial[0]
	goal[0] = origin_shift - goal[0]
	return initial, goal

def animate(storage):
	b = create_graph()*0
	g = create_graph()*255
	r = create_graph()*0
	graph = np.dstack([b,g,r]).astype(np.uint8)
	width, height, shape2 = graph.shape
	writer= cv2.VideoWriter('working.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 30, (height,width))
	# import pdb; pdb.set_trace()
	curr_key = len(storage)-1
	for i in storage.values():
		if i[0]==0:
			curr_point = np.array(i[1])[:2]
			# import pdb; pdb.set_trace()
			graph = cv2.circle(graph, tuple(curr_point[::-1]), 1, [0, 155, 255], -1)
			continue
		parent = i[3]
		par_point = np.array(storage[parent][1])[:2]
		curr_point = np.array(i[1])[:2]
		graph = cv2.arrowedLine(graph, tuple(par_point[::-1]), tuple(curr_point[::-1]), [255,255,0], 1)
		writer.write(graph)
		graph = cv2.circle(graph, tuple(curr_point[::-1]), 1, [0, 155, 255], -1)
		writer.write(graph)

	while curr_key != 0:
		parent = storage[curr_key][3]
		par_point = np.array(storage[parent][1])[:2]
		curr_point = np.array(storage[curr_key][1])[:2]
		graph = cv2.line(graph, tuple(par_point[::-1]), tuple(curr_point[::-1]), [255,255,255], 1)
		curr_key = parent
	start = time.time()
	while time.time()-start < 0.009:
		writer.write(graph)
	writer.release()
	cv2.imwrite('Graph.png', graph)

def main():
	ip_x = int(input("Enter initial x coordinate 15-389:\n"))
	ip_y = int(input("Enter initial y coordinate 15-239:\n"))
	ip_orn = int(input("Enter initial orientation:\n"))
	g_x = int(input("Enter goal x coordinate 10-389:\n"))
	g_y = int(input("Enter goal y coordinate 10-239:\n"))
	rpm1 = int(input("Enter RPM 1 0-8:\n"))
	rpm2 = int(input("Enter RPM 2 0-8:\n"))
	rpm = [rpm1, rpm2]

	start_time = time.time()

	ip_orn = ip_orn

	initial = [ip_y, ip_x, ip_orn]
	goal = [g_y, g_x]
	initial, goal = origin_shift(initial, goal)
	
	robot = Robot(initial, rpm)

	coords = a_star(goal, robot)
	if coords: 
		animate(coords)
	end_time = time.time()

if __name__ == '__main__':
	main()
