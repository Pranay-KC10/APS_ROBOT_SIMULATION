import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# -----------------------------
# Kalman Filter Initialization
# -----------------------------
# Assume we are tracking RSSI value that fluctuates around -60 dBm
true_value = -60.0

# Initial guesses
x_est = -60.0      # initial estimate
P = 1.0            # initial covariance
Q = 0.05           # process noise covariance
R = 2.0            # measurement noise covariance

# Arrays to store data
rssi_true = []
rssi_noisy = []
rssi_filtered = []

# -----------------------------
# Kalman Filter Update Function
# -----------------------------
def kalman_filter(z, x_est, P):
    # Prediction step
    x_pred = x_est
    P_pred = P + Q

    # Update step
    K = P_pred / (P_pred + R)         # Kalman Gain
    x_new = x_pred + K * (z - x_pred) # Updated estimate
    P_new = (1 - K) * P_pred          # Updated covariance

    return x_new, P_new

# -----------------------------
# Live Simulation Setup
# -----------------------------
fig, ax = plt.subplots()
ax.set_xlim(0, 100)
ax.set_ylim(-80, -40)
ax.set_title("Live Kalman Filter on Noisy RSSI Signal")
ax.set_xlabel("Time Step")
ax.set_ylabel("RSSI (dBm)")

line_noisy, = ax.plot([], [], 'b-', label="Noisy RSSI")
line_kalman, = ax.plot([], [], 'r-', linewidth=2, label="Kalman Filtered")
ax.legend()

# -----------------------------
# Animation Update Function
# -----------------------------
def update(frame):
    global x_est, P

    # True RSSI + small drift
    drift = np.sin(frame * 0.05) * 2
    true_rssi = true_value + drift

    # Add random measurement noise
    noisy_rssi = true_rssi + np.random.normal(0, np.sqrt(R))

    # Kalman filter update
    x_est, P = kalman_filter(noisy_rssi, x_est, P)

    # Store results
    rssi_true.append(true_rssi)
    rssi_noisy.append(noisy_rssi)
    rssi_filtered.append(x_est)

    # Limit window size for display
    window = 100
    start = max(0, len(rssi_noisy) - window)
    x_data = np.arange(start, len(rssi_noisy))

    line_noisy.set_data(x_data, rssi_noisy[start:])
    line_kalman.set_data(x_data, rssi_filtered[start:])

    ax.set_xlim(start, start + window)
    return line_noisy, line_kalman

# Run animation
ani = FuncAnimation(fig, update, frames=np.arange(0, 1000), interval=50, blit=True)
plt.show()
