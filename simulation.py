import threading
import time
import random
import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import warnings

# Suppress matplotlib warnings
warnings.filterwarnings("ignore", category=UserWarning, module="matplotlib")

class FlightControllerVisualizer:
    def __init__(self, port=None):
        self.running = True
        self.flight_state = "idle"  # idle, takeoff, move, land
        self.serial_port = port

        # Buffers for flight controller data
        self.temps = deque(maxlen=50)
        self.humids = deque(maxlen=50)
        self.aqs = deque(maxlen=50)
        self.altitudes = deque(maxlen=50)
        self.pitches = deque(maxlen=50)
        self.rolls = deque(maxlen=50)
        self.yaws = deque(maxlen=50)
        self.times = deque(maxlen=50)

        # Current values
        self.temp = 0
        self.humid = 0
        self.aq = 0
        self.altitude = 0
        self.pitch = 0
        self.roll = 0
        self.yaw = 0

    @staticmethod
    def detect_esp32():
        """Detect if an ESP32 is connected."""
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if "USB" in port.description or "UART" in port.description:
                print(f"✅ ESP32 detected on port: {port.device}")
                return port.device
        return None

    def read_simulated_data(self):
        """Simulate flight controller data."""
        while self.running:
            # Simulate environmental data (based on Bangalore's typical weather)
            self.temp = random.uniform(25, 35)  # Temperature in °C
            self.humid = random.uniform(50, 70)  # Humidity in %
            self.aq = random.uniform(50, 150)  # Air Quality in ppm

            # Simulate flight movement based on state
            if self.flight_state == "takeoff":
                self.altitude += random.uniform(5, 10)  # Increase altitude
                self.pitch = random.uniform(-5, 5)
                self.roll = random.uniform(-5, 5)
                self.yaw = random.uniform(0, 360)
            elif self.flight_state == "move":
                self.altitude += random.uniform(-2, 2)  # Slight altitude variation
                self.pitch = random.uniform(-10, 10)
                self.roll = random.uniform(-10, 10)
                self.yaw = (self.yaw + random.uniform(-15, 15)) % 360
            elif self.flight_state == "land":
                self.altitude = max(0, self.altitude - random.uniform(5, 10))  # Decrease altitude
                self.pitch = random.uniform(-5, 5)
                self.roll = random.uniform(-5, 5)
                self.yaw = random.uniform(0, 360)
                if self.altitude == 0:
                    self.flight_state = "idle"  # Stop movement after landing

            # Append data to buffers
            self.temps.append(self.temp)
            self.humids.append(self.humid)
            self.aqs.append(self.aq)
            self.altitudes.append(self.altitude)
            self.pitches.append(self.pitch)
            self.rolls.append(self.roll)
            self.yaws.append(self.yaw)
            self.times.append(time.time())

            # Debugging: Print the latest data
            print(f"State: {self.flight_state} | Temp: {self.temp:.2f}°C, Humid: {self.humid:.2f}%, AQ: {self.aq:.2f} ppm, "
                  f"Altitude: {self.altitude:.2f} m, Pitch: {self.pitch:.2f}°, Roll: {self.roll:.2f}°, Yaw: {self.yaw:.2f}°")

            time.sleep(1)  # Simulate data every second

    def update_plot(self, frame, axes):
        """Update the dashboard with live data."""
        for ax in axes:
            ax.clear()

        # Time normalization
        if self.times:
            t0 = self.times[0]
            times = [t - t0 for t in self.times]

            # Plot temperature, humidity, and air quality
            axes[0].plot(times, self.temps, label=f"Temp: {self.temp:.1f}°C", color='red')
            axes[0].plot(times, self.humids, label=f"Humidity: {self.humid:.1f}%", color='blue')
            axes[0].plot(times, self.aqs, label=f"AQ: {self.aq:.1f} ppm", color='green')
            axes[0].set_title("Environmental Data")
            axes[0].set_xlabel("Time (s)")
            axes[0].set_ylabel("Values")
            axes[0].legend(loc="upper right")
            axes[0].grid(True)

            # Plot altitude
            axes[1].plot(times, self.altitudes, label=f"Altitude: {self.altitude:.1f} m", color='purple')
            axes[1].set_title("Altitude")
            axes[1].set_xlabel("Time (s)")
            axes[1].set_ylabel("Altitude (m)")
            axes[1].legend(loc="upper right")
            axes[1].grid(True)

            # Plot pitch, roll, and yaw
            axes[2].plot(times, self.pitches, label=f"Pitch: {self.pitch:.1f}°", color='orange')
            axes[2].plot(times, self.rolls, label=f"Roll: {self.roll:.1f}°", color='cyan')
            axes[2].plot(times, self.yaws, label=f"Yaw: {self.yaw:.1f}°", color='magenta')
            axes[2].set_title("Orientation")
            axes[2].set_xlabel("Time (s)")
            axes[2].set_ylabel("Degrees")
            axes[2].legend(loc="upper right")
            axes[2].grid(True)

    def listen_for_commands(self):
        """Listen for user commands to control the flight."""
        while self.running:
            command = input("Enter command (takeoff, move, land, exit): ").strip().lower()
            if command == "takeoff":
                self.flight_state = "takeoff"
            elif command == "move":
                self.flight_state = "move"
            elif command == "land":
                self.flight_state = "land"
            elif command == "exit":
                self.running = False
                print("Exiting simulation...")
            else:
                print("Invalid command. Please enter 'takeoff', 'move', 'land', or 'exit'.")

    def run(self):
        """Run the dashboard."""
        # Start thread for reading simulated data
        threading.Thread(target=self.read_simulated_data, daemon=True).start()

        # Start thread for listening to commands
        threading.Thread(target=self.listen_for_commands, daemon=True).start()

        # Setup live plot
        fig, axes = plt.subplots(3, 1, figsize=(10, 15))
        ani = animation.FuncAnimation(fig, self.update_plot, fargs=(axes,), interval=1000)

        try:
            plt.tight_layout()
            plt.show()
        except KeyboardInterrupt:
            print("🛑 Exiting...")
            self.running = False

if __name__ == "__main__":
    try:
        port = FlightControllerVisualizer.detect_esp32()
        if port:
            visualizer = FlightControllerVisualizer(port)
            visualizer.run()
        else:
            print("❌ No ESP32 detected. Please connect an ESP32 and try again.")
    except KeyboardInterrupt:
        print("👋 Clean exit.")
