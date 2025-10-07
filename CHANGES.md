# Changes Made to RF Coverage Simulator

## 🔧 Key Fixes & Updates

### 1. ✅ Fixed Gradient Direction: DESCENT (not ascent)
**Problem**: Robot was moving toward strongest signal (ascent)
**Solution**: Changed to gradient **descent** to seek weakest RSSI

**File**: `simulation.py`
- Negated gradient in `compute_gradient()`: `return -grad_x, -grad_y`
- Updated comments to reflect descent behavior
- Algorithm now moves toward minimum (weakest signal)

### 2. ✅ Increased Step Size to Prevent Jittering
**Problem**: Step size was too small (0.5), causing robot to jitter in place
**Solution**: Increased learning rate from 0.5 → 3.0

**Changes**:
- `config.json`: `"learning_rate": 3.0`
- `simulation.py`: Default parameter changed to 3.0
- Reduced noise during descent phase for smoother movement

### 3. ✅ Added Multi-Phase Operation
**Problem**: No coverage mapping before descent
**Solution**: Implemented 4-phase autonomous operation

**Phases**:
1. **LAWNMOWER**: Systematic coverage pattern scanning entire field
2. **MEASURING**: Pause to "process" collected RSSI data (1 second)
3. **VISUALIZING**: Simulate heatmap generation (2 seconds)
4. **DESCENT**: Gradient descent to weakest signal point

**File**: `simulation.py`
```python
Phase flow:
├─ lawnmower (coverage pattern)
├─ measuring (data processing)
├─ visualizing (heatmap build)
└─ descent (seek minimum RSSI)
```

**Features**:
- Lawnmower path generator with configurable spacing (5.0 units)
- Progress counter showing `X/Y` waypoints completed
- Automatic phase transitions
- Measured points stored for analysis

### 4. ✅ Updated Visualization
**Problem**: UI didn't show current phase or progress
**Solution**: Enhanced info panel with phase-aware display

**File**: `utils.py`, `main.py`
- Added `get_phase_description()` method
- Info panel now shows:
  - Current phase with progress (e.g., "COVERAGE: Lawnmower scan (45/200)")
  - Phase-specific status messages
  - Updated controls display
- Wider panel to accommodate longer text (300px → 450px)

### 5. ✅ Created Kalman Filter Simulation
**Problem**: Needed separate demo with EKF + SPSA
**Solution**: Complete new simulator with uncertainty tracking

**New File**: `kalman_filter_sim.py`

**Features**:
- **Extended Kalman Filter (EKF)** implementation
  - 4D state: [x, y, θ, v]
  - Nonlinear motion model with linearization (Jacobian)
  - Prediction + Correction steps
  - Full covariance matrix tracking

- **SPSA Gradient Descent**
  - Random perturbation gradient estimation
  - Robust to noise
  - Seeking weakest RSSI

- **Visual Features**:
  - Green line: EKF estimated path
  - Blue line: Noisy measurements (simulated GPS)
  - Yellow ellipse: 95% confidence region
  - Robot with heading indicator

- **Documentation**: `README_KALMAN.md` with full theory

## 📊 Comparison

| Feature | Before | After |
|---------|--------|-------|
| **Gradient Direction** | Ascent (→ max) | Descent (→ min) ✓ |
| **Step Size** | 0.5 (jittery) | 3.0 (smooth) ✓ |
| **Operation** | Single phase | 4-phase workflow ✓ |
| **Coverage** | Random start | Lawnmower pattern ✓ |
| **Phase Display** | Generic "Mode" | Detailed phase info ✓ |
| **Kalman Filter** | None | Full EKF sim ✓ |

## 🚀 How to Run

### Main Simulator (Multi-Phase)
```bash
cd wifi_simulator
python main.py
```

**Behavior**:
1. Robot starts lawnmower coverage pattern
2. Systematically scans entire field
3. Pauses to "measure" and "visualize"
4. Begins gradient descent to weakest point
5. Press `M` to switch to manual control anytime
6. Press `R` to reset and start over

### Kalman Filter Simulator
```bash
cd wifi_simulator
python kalman_filter_sim.py
```

**Behavior**:
1. Robot navigates using SPSA gradient descent
2. EKF estimates state from noisy measurements
3. Yellow ellipse shows uncertainty
4. Green vs blue lines show filter performance
5. Press `R` to reset with new map

## 📐 Mathematical Implementations

### Gradient Descent (Main Simulator)
```python
# Compute gradient with central differences
grad_x = (R(x+δ, y) - R(x-δ, y)) / (2δ)
grad_y = (R(x, y+δ) - R(x, y-δ)) / (2δ)

# DESCENT: move opposite to gradient (toward minimum)
x_new = x - η * grad_x + noise
y_new = y - η * grad_y + noise
```

### SPSA Gradient Descent (Kalman Simulator)
```python
# Random perturbation
Δ = [±1, ±1]  # random

# Gradient estimate
grad = [(R(p+cΔ) - R(p-cΔ)) / (2c)] * Δ

# Update
p_new = p - η * grad
```

### EKF Prediction
```python
# Motion model
x_new = x + v*cos(θ)*dt
y_new = y + v*sin(θ)*dt
θ_new = θ + ω*dt

# Covariance prediction
P⁻ = F*P*Fᵀ + Q
```

### EKF Update
```python
# Kalman gain
K = P⁻*Hᵀ*(H*P⁻*Hᵀ + R)⁻¹

# State correction
x = x⁻ + K*(z - H*x⁻)

# Covariance update
P = (I - K*H)*P⁻
```

## 🎯 Use Cases

### Main Simulator
- Demonstrate complete coverage + localization workflow
- Show lawnmower scanning pattern
- Visualize gradient descent navigation
- Educational: multi-phase autonomous system

### Kalman Filter Simulator
- Demonstrate sensor fusion (motion + measurement)
- Show uncertainty quantification
- Compare noisy measurements vs filtered estimate
- Educational: EKF theory in practice

## 📁 Project Structure

```
wifi_simulator/
├── main.py                    # Multi-phase simulator
├── kalman_filter_sim.py       # EKF + SPSA simulator
├── simulation.py              # Phase logic + gradient descent
├── robot.py                   # Robot state tracking
├── map_generator.py           # RSSI field generation
├── utils.py                   # Visualization helpers
├── config.json                # Parameters
├── requirements.txt           # Dependencies
├── README.md                  # Main documentation
├── README_KALMAN.md          # Kalman filter theory
└── CHANGES.md                # This file
```

## 🔮 Future Enhancements

- [ ] Add particle filter comparison
- [ ] Multi-robot coordination
- [ ] Obstacle avoidance during lawnmower
- [ ] Export measured points to CSV with RSSI
- [ ] Real-time RSSI heatmap reconstruction
- [ ] GIF/video export of full workflow
- [ ] 3D visualization option

---

**All requested features implemented and tested!** ✓
