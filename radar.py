# import matplotlib.pyplot as plt
import numpy as np

class Drone:
	def __init__(self, x,y, phase, radius, yaw):
		self.pos = np.array([x,y])
		self.phase = phase
		self.radius = radius
		self.yaw = yaw
		self.updateTarget()
	def iter(self,timestep):
		self.phase=(self.phase+timestep)%360
		self.updateTarget()
	def updateTarget(self):
		self.target = R*np.array([np.cos(self.phase*np.pi/180),np.sin(self.phase*np.pi/180)])

class Manager:
	def __init__(self,*drones, ts=1):
		self.drones = drones
		self.timestep = ts
	def iter(self):
		for d in self.drones:
			d.iter(self.timestep)
		
R = 5
speed = .5
maxSpeed = 1
phase = 0

drone1 = Drone(5,0,phase,R,90)
drone2 = Drone(-5,0,phase+180,R,90+phase)

manager = Manager(drone1, drone2)
manager.iter()
print(manager.drones[0].target)
print(manager.drones[1].target)