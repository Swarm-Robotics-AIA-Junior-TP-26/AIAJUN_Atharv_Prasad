import time
import math
import sys
from pymavlink import mavutil

CONNECTION_STRING = 'udp:127.0.0.1:14550'
REACH_THRESHOLD = 1.0
MAX_SPEED = 5.0
CRUISE_SPEED = 3.0

WAYPOINTS = [
    (0, 0, -10),
    (20, 0, -20),
    (20, 20, -30),
    (0, 20, -5),
    (0, 0, -10)
]

class DroneController:
    def __init__(self, connection_string):
        self.master = mavutil.mavlink_connection(connection_string)
        self.current_pos = [0, 0, 0]
        self.is_active = True
        self.velocity_mask = 0b0000111111000111

    def wait_for_heartbeat(self):
        print("Waiting for heartbeat...")
        self.master.wait_heartbeat()
        print("Heartbeat received!")

    def hover_for_seconds(self, duration):
        print(f"Hovering for {duration} seconds...")
        start_time = time.time()
        
        while time.time() - start_time < duration:
            self.send_velocity_command(0, 0, 0)
            self.get_current_position()
            time.sleep(0.1)

    def get_current_position(self):
        msg = None
        while True:
            temp = self.master.recv_match(type='LOCAL_POSITION_NED', blocking=False)
            if temp:
                msg = temp
            else:
                break
        
        if msg:
            self.current_pos = [msg.x, msg.y, msg.z]
            return True
        return False

    def send_velocity_command(self, vx, vy, vz):
        self.master.mav.set_position_target_local_ned_send(
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            self.velocity_mask,
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            0, 0
        )

    def arm_and_set_offboard(self):
        print("Setting up Offboard...")
        
        for _ in range(10):
            self.send_velocity_command(0, 0, 0)
            time.sleep(0.1)

        print("Switching to OFFBOARD...")
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            1, 6, 0, 0, 0, 0, 0
        )
        
        print("Arming...")
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0
        )
        
        time.sleep(1)

    def land(self):
        print("Landing...")
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0, 0, 0, 0, 0, 0, 0
        )
        time.sleep(5)

    def run_mission(self):
        self.wait_for_heartbeat()
        self.arm_and_set_offboard()

        print("Taking off (Velocity Control)...")
        for _ in range(50):
            self.send_velocity_command(0, 0, -1.5)
            time.sleep(0.1)
            self.get_current_position()

        print("Starting Waypoint Mission...")
        
        for i, wp in enumerate(WAYPOINTS):
            print(f"Navigating to Waypoint {i+1}: {wp}")
            
            while True:
                self.get_current_position()
                
                dx = wp[0] - self.current_pos[0]
                dy = wp[1] - self.current_pos[1]
                dz = wp[2] - self.current_pos[2]
                
                distance = math.sqrt(dx**2 + dy**2 + dz**2)
                
                if distance < REACH_THRESHOLD:
                    print(f"Reached Waypoint {i+1}")
                    self.hover_for_seconds(10)
                    break
                
                speed = min(CRUISE_SPEED, distance)
                
                if distance > 0:
                    vx = (dx / distance) * speed
                    vy = (dy / distance) * speed
                    vz = (dz / distance) * speed
                else:
                    vx, vy, vz = 0, 0, 0

                self.send_velocity_command(vx, vy, vz)
                time.sleep(0.1)

        print("Mission Complete.")
        self.land()

if __name__ == "__main__":
    drone = DroneController(CONNECTION_STRING)
    drone.run_mission()
