# import matplotlib.pyplot as plt
import numpy as np

class Drone:
	def __init__(self, x,y, phase, radius, yaw):
		self.pos = np.array([x,y])
		self.phase = phase
		self.radius = radius
		self.yaw = yaw
		self.target = None
	def iter(self,timestep, center):
		self.phase=(self.phase+timestep)%360
		self.updateTarget(center)
	def updateTarget(self, center):
		self.target = center + R*np.array([np.cos(self.phase*np.pi/180),np.sin(self.phase*np.pi/180)])

class Manager:
	def __init__(self,center,*drones,ts=1):
		self.drones = drones
		self.timestep = ts
		assert len({d.radius for d in self.drones}) == 1
		self.radius = self.drones[0].radius
		self.center = center
	def updateCenter(self, center):
		self.center = center
	def iter(self):
		phase = (self.drones[0].phase+90)%360
		# TODO: add target position
		virtual_target = self.center + self.radius*1.5*np.array([np.cos(phase*np.pi/180),np.sin(phase*np.pi/180)])
		# print(virtual_target)
		for d in self.drones:
			d.iter(self.timestep, self.center)
			xy = virtual_target - d.pos
			# print(d.pos, xy, virtual_target)
			d.yaw = np.arctan2(xy[1], xy[0])*180/np.pi
			# print(xy, d.yaw)
R = 5
speed = .5
maxSpeed = 1
phase = 0

drone1 = Drone(5,0,phase,R,90)
drone2 = Drone(-5,0,phase+180,R,90+phase)

manager = Manager(np.array([1,1]), drone1, drone2)
manager.iter()
print(manager.drones[0].target,manager.drones[0].yaw)
print(manager.drones[1].target,manager.drones[1].yaw)