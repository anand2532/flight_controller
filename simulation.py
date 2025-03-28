import time
import random
import math

class ESP32FlightControllerSim:
    def __init__(self):
        self.roll = 0.0  # degrees
        self.pitch = 0.0  # degrees
        self.yaw = 0.0  # degrees
        self.altitude = 100.0  # meters
        self.latitude = 12.9716  # dummy GPS coordinates
        self.longitude = 77.5946
        self.motor_thrust = 0.5  # 50% thrust

    def simulate_imu(self):
        """ Simulate IMU (Accelerometer + Gyro) data """
        self.roll += random.uniform(-0.5, 0.5)  # Small variations
        self.pitch += random.uniform(-0.5, 0.5)
        self.yaw += random.uniform(-0.5, 0.5)

    def simulate_barometer(self):
        """ Simulate altitude changes """
        climb_rate = random.uniform(-1, 1)  # m/s climb or descend
        self.altitude += climb_rate

    def simulate_gps(self):
        """ Simulate GPS movement """
        self.latitude += random.uniform(-0.0001, 0.0001)
        self.longitude += random.uniform(-0.0001, 0.0001)

    def pid_control(self):
        """ Simulate simple PID control """
        target_altitude = 120.0  # Target altitude in meters
        error = target_altitude - self.altitude
        self.motor_thrust = min(max(0.3 + (error * 0.01), 0.3), 1.0)  # PID-like control

    def display_status(self):
        """ Display formatted data in terminal """
        print("\033[H\033[J")  # Clear screen
        print(f"🚀 ESP32 Flight Controller Simulation")
        print("-" * 50)
        print(f"IMU Data: Roll: {self.roll:.2f}° | Pitch: {self.pitch:.2f}° | Yaw: {self.yaw:.2f}°")
        print(f"GPS Position: {self.latitude:.6f}, {self.longitude:.6f}")
        print(f"Altitude: {self.altitude:.2f} m")
        print(f"Motor Thrust: {self.motor_thrust:.2f} (30% - 100%)")
        print("-" * 50)

    def run(self):
        """ Run the simulation loop """
        try:
            while True:
                self.simulate_imu()
                self.simulate_barometer()
                self.simulate_gps()
                self.pid_control()
                self.display_status()
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nSimulation stopped.")

if __name__ == "__main__":
    sim = ESP32FlightControllerSim()
    sim.run()
