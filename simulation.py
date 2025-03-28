# import time
# import random
# import math

# class ESP32FlightControllerSim:
#     def __init__(self):
#         self.roll = 0.0  # degrees
#         self.pitch = 0.0  # degrees
#         self.yaw = 0.0  # degrees
#         self.altitude = 100.0  # meters
#         self.latitude = 12.9716  # dummy GPS coordinates
#         self.longitude = 77.5946
#         self.motor_thrust = 0.5  # 50% thrust

#     def simulate_imu(self):
#         """ Simulate IMU (Accelerometer + Gyro) data """
#         self.roll += random.uniform(-0.5, 0.5)  # Small variations
#         self.pitch += random.uniform(-0.5, 0.5)
#         self.yaw += random.uniform(-0.5, 0.5)

#     def simulate_barometer(self):
#         """ Simulate altitude changes """
#         climb_rate = random.uniform(-1, 1)  # m/s climb or descend
#         self.altitude += climb_rate

#     def simulate_gps(self):
#         """ Simulate GPS movement """
#         self.latitude += random.uniform(-0.0001, 0.0001)
#         self.longitude += random.uniform(-0.0001, 0.0001)

#     def pid_control(self):
#         """ Simulate simple PID control """
#         target_altitude = 120.0  # Target altitude in meters
#         error = target_altitude - self.altitude
#         self.motor_thrust = min(max(0.3 + (error * 0.01), 0.3), 1.0)  # PID-like control

#     def display_status(self):
#         """ Display formatted data in terminal """
#         print("\033[H\033[J")  # Clear screen
#         print(f"🚀 ESP32 Flight Controller Simulation")
#         print("-" * 50)
#         print(f"IMU Data: Roll: {self.roll:.2f}° | Pitch: {self.pitch:.2f}° | Yaw: {self.yaw:.2f}°")
#         print(f"GPS Position: {self.latitude:.6f}, {self.longitude:.6f}")
#         print(f"Altitude: {self.altitude:.2f} m")
#         print(f"Motor Thrust: {self.motor_thrust:.2f} (30% - 100%)")
#         print("-" * 50)

#     def run(self):
#         """ Run the simulation loop """
#         try:
#             while True:
#                 self.simulate_imu()
#                 self.simulate_barometer()
#                 self.simulate_gps()
#                 self.pid_control()
#                 self.display_status()
#                 time.sleep(1)
#         except KeyboardInterrupt:
#             print("\nSimulation stopped.")

# if __name__ == "__main__":
#     sim = ESP32FlightControllerSim()
#     sim.run()

import serial
import time
import threading
import random
import matplotlib.pyplot as plt
import numpy as np

# Detect ESP32 Connection
def detect_esp32():
    try:
        ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)  # Change port if needed
        ser.close()
        print("✅ ESP32 Detected! Starting Simulation...\n")
        return True
    except serial.SerialException:
        print("❌ ESP32 Not Found! Connect the board and restart.")
        return False

class FlightSimulator:
    def __init__(self):
        # Flight Parameters
        self.altitude = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.thrust = 0
        self.lat = 0.0001  # Initialize with a small value
        self.lon = 0.0001  # Initialize with a small value
        self.running = True

    def process_command(self, cmd):
        """ Processes user commands """
        if cmd == "takeoff":
            self.altitude = 100
            self.thrust = 0.8
        elif cmd == "land":
            self.altitude = 0
            self.thrust = 0
        elif cmd.startswith("move forward"):
            distance = int(cmd.split()[2])
            self.lat += distance * 0.0001
        elif cmd.startswith("turn left"):
            angle = int(cmd.split()[2])
            self.yaw -= angle
        elif cmd.startswith("turn right"):
            angle = int(cmd.split()[2])
            self.yaw += angle

    def generate_sensor_data(self):
        """ Simulates sensor readings """
        while self.running:
            self.roll += random.uniform(-0.5, 0.5)
            self.pitch += random.uniform(-0.5, 0.5)
            self.altitude += random.uniform(-1, 1) if self.thrust > 0 else -1
            self.altitude = max(0, self.altitude)
            time.sleep(1)

    def plot_animation(self):
        """ Runs 2D animation """
        plt.ion()
        fig, ax = plt.subplots()
        ax.set_xlim(-0.001, 0.001)
        ax.set_ylim(-0.001, 0.001)
        drone, = ax.plot([self.lat], [self.lon], "ro")  # Wrap in lists

        while self.running:
            drone.set_xdata([self.lat])  # Fix: Convert to list
            drone.set_ydata([self.lon])  # Fix: Convert to list
            ax.set_title(f"Drone Position | Alt: {self.altitude:.1f}m | Yaw: {self.yaw:.1f}°")
            plt.draw()
            plt.pause(0.1)

    def start_command_loop(self):
        """ Handles user commands in the terminal """
        print("\n✈️ Flight Command Terminal ✈️")
        print("Commands: takeoff, move forward X, turn left X, turn right X, land")

        while self.running:
            cmd = input("\nEnter Command: ").strip()
            if cmd == "exit":
                self.running = False
                break
            self.process_command(cmd)

    def run(self):
        """ Starts all simulation components """
        threading.Thread(target=self.generate_sensor_data, daemon=True).start()
        threading.Thread(target=self.start_command_loop, daemon=True).start()
        self.plot_animation()

if __name__ == "__main__":
    if detect_esp32():
        simulator = FlightSimulator()
        simulator.run()

