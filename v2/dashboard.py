import serial
import threading
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# --- Hardware Setup Configuration ---
SERIAL_PORT = "COM3"  # Adjust this to match your Arduino's IDE USB port properties
BAUD_RATE = 115200

# Share complete data arrays safely across parsing threads
data_store = {
    "ax": 0.0, "ay": 0.0, "az": 0.0,
    "mx": 0.0, "my": 0.0, "mz": 0.0,
    "sound": 0, "temp": 0.0, "hum": 0.0, "pres": 0.0,
    "prox": 0, "ir_red": 0, 
    "radar_p": 0, "radar_m": 0,
    "pin_fall": 0, "pin_pres": 0  # Extracted values from C01 and C02
}

# Cumulative trend lines
history_len = 50
time_history = list(range(history_len))
sound_history = [0] * history_len
temp_history = [0.0] * history_len
prox_history = [0] * history_len

current_plot_index = 1 

def serial_worker():
    """Asynchronous background data frame consumer."""
    global data_store
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2) # Allow system board to settle
        while True:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line: continue
            parts = line.split(',')
            # Verify structure alignment for all 16 values
            if len(parts) == 16:
                data_store["ax"], data_store["ay"], data_store["az"] = float(parts[0]), float(parts[1]), float(parts[2])
                data_store["mx"], data_store["my"], data_store["mz"] = float(parts[3]), float(parts[4]), float(parts[5])
                data_store["sound"] = int(parts[6])
                data_store["temp"], data_store["hum"], data_store["pres"] = float(parts[7]), float(parts[8]), float(parts[9])
                data_store["prox"], data_store["ir_red"] = int(parts[10]), int(parts[11])
                data_store["radar_p"], data_store["radar_m"] = int(parts[12]), int(parts[13])
                data_store["pin_fall"], data_store["pin_pres"] = int(parts[14]), int(parts[15])
    except Exception as e:
        print(f"Serial pipeline closed: {e}")

# Fire background receiver thread
thread = threading.Thread(target=serial_worker, daemon=True)
thread.start()

# --- Plot Display Environment Setup ---
fig = plt.figure(figsize=(11, 7))

def on_key(event):
    """Listens for numeric strokes to swap views cleanly."""
    global current_plot_index
    if event.key in ['1', '2', '3', '4', '5']:
        current_plot_index = int(event.key)
        print(f"Switched Dashboard to Plot View: {current_plot_index}")

fig.canvas.mpl_connect('key_press_event', on_key)

def update_dashboard(frame):
    global sound_history, temp_history, prox_history
    fig.clear()
    
    # Pack continuous data loops into historical trends
    sound_history = sound_history[1:] + [data_store["sound"]]
    temp_history = temp_history[1:] + [data_store["temp"]]
    prox_history = prox_history[1:] + [data_store["prox"]]

    fig.suptitle("Live Sensor Center Dashboard\nPress [1-5] on your keyboard to switch tabs", fontsize=11, color='darkblue')

    # VIEW 1: IMU Accelerometers + Live Hardware IO Pin Monitor Panel
    if current_plot_index == 1:
        # Left Panel: IMU Vectors
        ax1 = fig.add_subplot(121)
        ax1.bar(['Acc X', 'Acc Y', 'Acc Z'], [data_store["ax"], data_store["ay"], data_store["az"]], color=['crimson','seagreen','royalblue'])
        ax1.set_ylabel("Forces (G)")
        ax1.set_title("IMU Accelerometer Magnitudes")
        ax1.set_ylim([-2.0, 2.0])
        ax1.grid(True, alpha=0.3)

        # Right Panel: Radar State Readouts
        ax2 = fig.add_subplot(122)
        ax2.axis('off')
        status_box = (
            f"=== RADAR TELEMETRY STREAM ===\n\n"
            f"Software State Presence: {'YES' if data_store['radar_p'] else 'NO'}\n"
            f"Software Motion Category: {['None', 'Still', 'Active'][data_store['radar_m']]}\n\n"
            f"=== HARDWARE PIN STATES ===\n\n"
            f"Pin C01 (FALL ALERT Pin 2):  {['NORMAL', '🚨 FALL DETECTED!'][data_store['pin_fall']]}\n"
            f"Pin C02 (PRESENCE Pin 3): {['EMPTY', '👤 PERSON DETECTED'][data_store['pin_pres']]}\n"
        )
        ax2.text(0.1, 0.4, status_box, fontsize=12, family='monospace',
                 bbox=dict(facecolor='whitesmoke', alpha=0.9, boxstyle='round,pad=1'))

    # VIEW 2: 3D Magnetometer Heading Pointer
    elif current_plot_index == 2:
        ax = fig.add_subplot(111, projection='3d')
        ax.quiver(0, 0, 0, data_store["mx"], data_store["my"], data_store["mz"], color='purple', length=1.0, normalize=True)
        ax.set_xlim([-1, 1]); ax.set_ylim([-1, 1]); ax.set_zlim([-1, 1])
        ax.set_xlabel("Mag X"); ax.set_ylabel("Mag Y"); ax.set_zlabel("Mag Z")
        ax.set_title(f"3D Vector Orientation Heading Map\nX Vector: {data_store['mx']:.2f} | Y Vector: {data_store['my']:.2f}")

    # VIEW 3: Sound Amplitude Waves
    elif current_plot_index == 3:
        ax = fig.add_subplot(111)
        ax.plot(time_history, sound_history, color='darkorange', linewidth=2)
        ax.set_title(f"Microphone Audio Wave Peak Envelope Amplitude: {data_store['sound']}")
        ax.set_ylim([0, 3000])
        ax.grid(True)

    # VIEW 4: Weather Conditions Charts
    elif current_plot_index == 4:
        ax = fig.add_subplot(111)
        ax.plot(time_history, temp_history, color='deeppink', linewidth=2)
        ax.set_title(f"Onboard Climate Metrics\nTemp: {data_store['temp']:.1f}°C | Humidity: {data_store['hum']:.1f}% | Baro: {data_store['pres']:.2f} kPa")
        ax.set_ylabel("Temperature Timeline (°C)")
        ax.grid(True)

    # VIEW 5: IR Sensor / Optical Proximity
    elif current_plot_index == 5:
        ax = fig.add_subplot(111)
        ax.fill_between(time_history, prox_history, color='teal', alpha=0.3)
        ax.plot(time_history, prox_history, color='teal', linewidth=1.5)
        ax.set_title(f"Proximity Range Profile Value: {data_store['prox']} | IR Color Red Channel: {data_store['ir_red']}")
        ax.set_ylim([0, 260])
        ax.grid(True)

# Run interactive dashboard sequence loops
ani = FuncAnimation(fig, update_dashboard, interval=100, cache_frame_data=False)
plt.show()
