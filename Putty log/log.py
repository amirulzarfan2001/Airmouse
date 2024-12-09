import serial
import matplotlib.pyplot as plt
import time

# Set up the serial connection
arduino = serial.Serial('COM18', 115200)  # Replace 'COM3' with your Arduino's port
time.sleep(2)  # Wait for connection to establish

# Prepare the plot
plt.ion()
fig, ax = plt.subplots()
x_data, y_data = [], []

try:
    while True:
        if arduino.in_waiting > 0:
            line = arduino.readline().decode('utf-8').strip()  # Read serial data
            y = float(line)  # Convert to float (update this if sending multiple data points)
            
            # Update data
            x_data.append(len(x_data))
            y_data.append(y)
            
            # Plot data
            ax.clear()
            ax.plot(x_data, y_data, label='Arduino Data')
            ax.legend(loc='upper left')
            ax.set_xlabel("Time (samples)")
            ax.set_ylabel("Value")
            plt.pause(0.01)
except KeyboardInterrupt:
    print("Plotting stopped.")
    arduino.close()
    plt.show()