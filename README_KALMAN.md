# Extended Kalman Filter + SPSA Gradient Descent Simulator

A complete implementation of **Extended Kalman Filter (EKF)** for robot state estimation combined with **SPSA (Simultaneous Perturbation Stochastic Approximation)** gradient descent for navigating toward weakest RSSI regions.

## 🎯 Overview

This simulator demonstrates:

1. **Extended Kalman Filter (EKF)** for state estimation
   - Predicts robot state using nonlinear motion model
   - Corrects predictions using noisy measurements (simulated GPS)
   - Tracks position (x, y), heading (θ), and velocity (v)

2. **SPSA Gradient Descent** for exploration
   - Estimates RSSI gradient using random perturbations
   - Navigates toward weakest signal regions (gradient descent)
   - Robust to measurement noise

3. **Visual Uncertainty Representation**
   - Real-time 95% confidence ellipse
   - Comparison of estimated vs measured paths
   - Covariance matrix visualization

## 📐 Mathematical Implementation

### A. Extended Kalman Filter Equations

#### State Vector
```
x = [x, y, θ, v]ᵀ
```
- `x, y`: Position coordinates
- `θ`: Heading angle (radians)
- `v`: Linear velocity

#### Prediction Step (Time Update)

**Nonlinear Motion Model:**
```
x_{k+1} = x_k + v_k cos(θ_k) Δt
y_{k+1} = y_k + v_k sin(θ_k) Δt
θ_{k+1} = θ_k + u_ω Δt
v_{k+1} = u_v
```

**Covariance Prediction:**
```
P⁻ = F P Fᵀ + Q
```

Where:
- `F`: Jacobian of motion model (linearization)
- `Q`: Process noise covariance

#### Update Step (Measurement Correction)

**Kalman Gain:**
```
K = P⁻ Hᵀ (H P⁻ Hᵀ + R)⁻¹
```

**State Update:**
```
x = x⁻ + K(z - Hx⁻)
```

**Covariance Update:**
```
P = (I - KH)P⁻
```

Where:
- `H`: Measurement matrix (extracts position from state)
- `R`: Measurement noise covariance
- `z`: Measurement vector [x_measured, y_measured]

### B. SPSA Gradient Descent

**SPSA Gradient Estimate:**
```
∇̂R(p) = [R(p + cΔ) - R(p - cΔ)] / (2c) · Δ
```

Where:
- `R(p)`: RSSI at position p
- `c`: Perturbation constant
- `Δ`: Random perturbation vector (±1 components)

**Position Update (Gradient Descent):**
```
p_{k+1} = p_k - η ∇̂R(p_k)
```

Where:
- `η`: Learning rate (step size)
- Negative sign → descend toward minimum (weakest RSSI)

## 🚀 Usage

### Run the Simulator

```bash
cd wifi_simulator
python kalman_filter_sim.py
```

### Controls

| Key | Action |
|-----|--------|
| **R** | Reset simulation with new RSSI map |
| **M** | Toggle mode (not implemented yet) |
| **ESC** | Quit |

## 🎨 Visualization

- **Background**: RSSI heatmap (yellow = strong, blue = weak)
- **Green line**: EKF estimated path (filtered state)
- **Blue line**: Noisy measurements (simulated GPS)
- **Yellow ellipse**: 95% confidence region (uncertainty)
- **White robot**: Current estimated position with heading indicator

## 📊 What's Happening

1. **Robot moves** based on SPSA gradient descent direction
2. **EKF predicts** next state using motion model
3. **Noisy measurements** simulate GPS readings
4. **EKF corrects** prediction using Kalman gain
5. **Uncertainty shrinks** when measurements are consistent
6. **Robot navigates** toward weakest RSSI while maintaining accurate state estimate

## ⚙️ Configuration

Edit `config.json` to adjust:

```json
{
  "learning_rate": 3.0,      // SPSA step size (η)
  "gradient_delta": 2.0,     // SPSA perturbation (c)
  "noise_std": 0.3,          // Process noise
  "map_size": 100,
  "window_size": 600,
  "fps": 60
}
```

## 🔬 EKF Parameters (in code)

### Process Noise Covariance (Q)
```python
Q = diag([0.1, 0.1, 0.05, 0.1])**2
```
Represents uncertainty in motion model:
- Position uncertainty: 0.1² per step
- Heading uncertainty: 0.05² per step
- Velocity uncertainty: 0.1² per step

### Measurement Noise Covariance (R)
```python
R = diag([0.5, 0.5])**2
```
Represents GPS measurement noise:
- x position error: ±0.5 (1σ)
- y position error: ±0.5 (1σ)

## 📈 Performance Metrics

The simulator tracks:
- **State estimate**: (x, y, θ, v)
- **Uncertainty**: σₓ, σᵧ from covariance matrix P
- **RSSI value**: Current signal strength at estimated position
- **Iteration count**: Number of filter steps

## 🧪 Experimental Features

### Uncertainty Visualization
The yellow ellipse represents the 95% confidence region (2.447σ) computed from the position covariance:

```python
eigenvalues, eigenvectors = eig(P[:2, :2])
width = 2 * 2.447 * sqrt(λ₁)
height = 2 * 2.447 * sqrt(λ₂)
```

## 🔄 Differences from Main Simulator

| Feature | Main Simulator | Kalman Filter Sim |
|---------|----------------|-------------------|
| **State Estimation** | Direct position | EKF with 4D state |
| **Navigation** | Multi-phase (lawnmower + descent) | Continuous SPSA descent |
| **Uncertainty** | None | Full covariance tracking |
| **Measurements** | Perfect | Noisy GPS simulation |
| **Path Display** | Single trail | Estimated + measured |

## 📚 Theory References

- **Extended Kalman Filter**: Nonlinear state estimation via linearization
- **SPSA**: Gradient-free optimization using simultaneous perturbations
- **Sensor Fusion**: Combining motion model with measurements for optimal estimate

## 🎓 Use Cases

1. **Educational**: Learn EKF and SPSA algorithms visually
2. **Research**: Test localization algorithms in RF environments
3. **Poster Demo**: Show real-time Kalman filtering with uncertainty
4. **Algorithm Comparison**: Compare EKF vs particle filters vs pure odometry

## 🐛 Known Limitations

- Simplified motion model (no wheel slip, perfect actuators)
- Gaussian noise assumption (may not match real sensors)
- 2D only (no elevation changes)
- Single robot (no multi-agent coordination)

## 🔮 Future Enhancements

- [ ] Add particle filter comparison
- [ ] Multi-sensor fusion (IMU + GPS + RSSI)
- [ ] Adaptive Q and R matrices
- [ ] SLAM (Simultaneous Localization and Mapping)
- [ ] Export filter performance metrics to CSV

## 📄 License

MIT License - Free for academic and research use

---

**Built for IEEE APS poster demonstration**

Combines classical control theory (EKF) with modern optimization (SPSA) for robust RF navigation.
