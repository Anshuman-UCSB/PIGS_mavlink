from pymavlink import mavutil
import vector
import numpy as np
class Drone:
	def __init__(self):
		self.phase = 0
		self.conn = mavutil.mavlink_connection('udpin:localhost:14551')
		self.conn.wait_heartbeat()
	def arm(self):
		self.conn.mav.command_long_send(self.conn.target_system, self.conn.target_component,
                                     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

		msg = self.conn.recv_match(type='COMMAND_ACK', blocking=True)
		print(msg)
	def disarm(self):
		self.conn.mav.command_long_send(self.conn.target_system, self.conn.target_component,
                                     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)

		msg = self.conn.recv_match(type='COMMAND_ACK', blocking=True)
		print(msg)
	def guided(self):
		self.conn.mav.command_long_send(self.conn.target_system, self.conn.target_component,
											mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 4, 0, 0, 0, 0, 0)

		msg = self.conn.recv_match(type='COMMAND_ACK', blocking=True)
		print(msg)
	def takeoff(self, altitude=5):
		#last param: altitude
		self.conn.mav.command_long_send(self.conn.target_system, self.conn.target_component,
											mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitude)

		msg = self.conn.recv_match(type='COMMAND_ACK', blocking=True)
		print(msg)
	def land(self):
		self.conn.mav.command_long_send(self.conn.target_system, self.conn.target_component,
											mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 9, 0, 0, 0, 0, 0)
		msg = self.conn.recv_match(type='COMMAND_ACK', blocking=True)
		print(msg)
	def getLatLon(self):
		msg = self.conn.recv_match( type='GLOBAL_POSITION_INT', blocking=True)
		return msg.lat, msg.lon
	def gps(self,lat, lon):
		self.conn.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(10, self.conn.target_system,
                        self.conn.target_component, 
                        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
						int(0b110111111000), int(lat), 
                        int(lon), 10, 0, 0, 0, 0, 0, 0, 1.57, 0.5))
		msg = self.conn.recv_match(type='LOCAL_POSITION_NED', blocking=True)
		print(msg)
	def orbit(self, center=None, radius=100):
		center=center or self.getLatLon()
		center=vector.obj(x=center[0],y=center[1])
		to=vector.obj(rho=radius,phi=self.phase)
		to.y*=1.372 # scaling factor due to lat/lon mismatch
		to+=center
		print(to.x,to.y, self.phase/(np.pi*2))
		self.gps(to.x,to.y)#
		self.phase=(self.phase+.01)%(2*np.pi)
if __name__ == "__main__":
	from time import sleep
	d = Drone()
	# d.guided()
	# d.arm()
	# d.takeoff()
	# sleep(10)
	while 1:
		d.orbit(radius=1000)
		sleep(1)
	# d.gps(15,0)
	# sleep(15)
	# d.land()