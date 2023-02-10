from pymavlink import mavutil
class Drone:
	def __init__(self):
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

if __name__ == "__main__":
	from time import sleep
	d = Drone()
	
	d.guided()
	d.arm()
	d.takeoff()
	
	sleep(15)
	d.land()