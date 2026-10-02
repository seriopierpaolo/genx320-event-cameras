
# GENX320 HANDSON ASSIGNMENT

This repository is also a hands-on assignment.  
The two exercises below ask you to fill in the core detection and control logic.  
Everything else — the simulator, the serial driver, the visualizer, the servo
thread, and the test suite — is already wired up and waiting for your code.

## System overview

```
GenX320 sensor (or simulator)
        │  events (t, x, y, p)
        ▼
   reader thread          ← src/client_main.cpp
        │
        ▼
   event ring buffer      ← include/event_buffer.hpp
        │
        ├──────────────────────────────────┐
        ▼                                  ▼
 line detector thread              visualizer (30 Hz)
 (every 200 ms)                    ← include/visualizer.hpp
        │
        ▼
  DetectedLine {rho, theta_rad}
        │
        ▼
   servo thread           ← src/client_main.cpp
        │  PWM pulse width [1000 µs … 1500 µs]
        ▼
   pan-tilt servo
```

The **sensor** (320 × 320 px) emits events `(t, x, y, p)` asynchronously
whenever the brightness at pixel `(x, y)` changes.  
The **goal** is to keep a vertical line centred in the field of view by
commanding the servo in the direction opposite to the line's position.

---

## Exercise 1 — Line detector

**File to edit:** `src/line_detector.cpp`  
**Function:** `LineDetector::detect(const std::vector<Event>& events)`

The function receives a snapshot of recent events and must return a
`std::vector<DetectedLine>` (sorted by confidence, highest first).

```cpp
struct DetectedLine {
    double rho;        // perpendicular distance from origin to line (pixels)
    double theta_rad;  // angle of that perpendicular in [0, π)  (radians)
};
```

The **Hough parameterisation** expresses every line as:

$$\rho = x \cos\theta + y \sin\theta$$

A vertical line at column $x_0$ corresponds to $\theta = 0$, $\rho = x_0$.

**You are free to use any algorithm.** Two approaches covered at the school:

### Option A — Perturbation tracking (event-driven)
Maintain to store events into the ring FIFO buffer.
In parallel, refer to this FIFO to estimate best line segment by hypothetical states perturbation approach.
In this case, you can only consider horizontal perturbation of the endpoints.
First, to initialize the line segment, you can use OpenCV function to fit line.
Then, you create (at least 5) hypothetical states by horizontal endpoint perturbation.
Finally, you should define cost function to evaluate which state is the best and update the line segment with the best state.

### Option B — Hough Transform (batch)
1. Build a 2-D binary frame: mark every pixel that received at least one
   event inside the time window (`window_us_`).
2. For each active pixel $(x, y)$ and each $\theta$ bin, vote:
   $\rho = x\cos\theta + y\sin\theta$
3. Find accumulator peaks (non-maximum suppression) and convert bin indices
   back to `(rho, theta_rad)`.

Member variables already available in the constructor:

| Variable | Meaning |
|---|---|
| `W_`, `H_` | Sensor width and height (320 × 320) |
| `theta_steps_` | Number of $\theta$ bins in $[0,\pi)$ — default 180 |
| `window_us_` | Keep only events within this many µs of the latest (0 = all) |


## Exercise 2 — Servo controller

**File to edit:** `include/servo_controller.hpp`  
**Function:** `compute_next_servo(const DetectedLine& line, int current_servo)`

Given the best detected line and the current servo pulse width, return the
**new pulse width** (in microseconds) that steers the camera toward the line.

The servo accepts a PWM pulse in the range:

| Constant | Value | Meaning |
|---|---|---|
| `SERVO_MIN_US` | 1000 µs | full left |
| `SERVO_NEUTRAL` | 1250 µs | straight ahead |
| `SERVO_MAX_US` | 1500 µs | full right |

The return value **must** be clamped to `[SERVO_MIN_US, SERVO_MAX_US]`.

**Direction convention:** if the line is to the **right** of centre the servo
must **decrease** (camera pans left to chase it), and vice-versa.

You are free to choose any control strategy:

### Option A — Proportional (P) controller
Convert `(rho, theta_rad)` to an x-coordinate at the sensor's centre row,
then apply a proportional correction:

$$x = \frac{\rho - \frac{H}{2}\sin\theta}{\cos\theta}$$

$$\text{error} = x - x_{\text{centre}}$$

$$\text{new servo} = \text{clamp}\!\left(\text{current servo} - K_p \times \text{error},\; \text{MIN},\; \text{MAX}\right)$$

A suggested gain `SERVO_KP = 1.5` µs/px is defined in the header.

### Option B — Bang-bang controller
- Line to the right → jump to `SERVO_MIN_US`
- Line to the left  → jump to `SERVO_MAX_US`

---


## Validation

A recording of a moving vertical line (`data/vertical_line.txt`) and its
ground truth (`data/ground_truth.txt`) are provided.  Run the test suite to
check your implementation:

```bash
cd build
./genx320_tests
```

The tests verify that:
1. Your detected line position is within **20 px** of the ground truth.
2. The servo command produced by Exercise 2 goes in the correct direction.

---


## Build

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

## Running the full system
### Terminal 1 – start the simulator (replays data/vertical_line.txt)

```bash
cd build && ./genx320_simulator

```

The simulator prints the slave PTY path on startup, e.g.:

```
=======================================================
 GenX320 Serial Port Simulator
=======================================================
 Virtual serial port : /dev/pts/3
 Connect with        : screen /dev/pts/3 115200
                       or minicom -D /dev/pts/3
 Send '??' for help.
=======================================================
```


### Terminal 2 – start the client (replace /dev/pts/N with the printed path)
```
cd build && ./genx320_client /dev/pts/N
```

An OpenCV window shows the event stream, detected lines in red, and a
**yellow arrow** at the centre of the image pointing in the direction the
servo is being commanded.

## Supported Commands

| Command | Description |
|---|---|
| `+` / `-` | Enable / disable sensor |
| `!E+` / `!E-` | Start / stop event streaming |
| `!E*` / `!E*-` | Hot-pixel detection / clear blocked pixels |
| `!EF[X\|Y\|B\|N]` | Flip axes X, Y, Both, None (HW) |
| `!ES+` / `!ES-` | Swap X/Y addresses (SW) |
| `!EF+` / `!EF-` | First-pixel-only filter on/off |
| `!EP+` / `!EP-` / `!EPA` | Polarity filter: positive / negative / all |
| `!ED=n` | Temporal down-sampling – send every n-th event |
| `!B<i>=<v>` | Set bias index `i` to value `v` (0-255) |
| `!BD` | Restore all biases to factory defaults |
| `?B[i]` | Read bias `i`; use `a` for all |
| `?BN` | Show bias index / name table |
| `!L-` / `!L+` / `!L.[=t]` / `!La[=t]` | LED off / on / blink / alarm (period in ms) |
| `R` | Reset board |
| `pRog` | Enter bootloader |
| `?V` | Firmware version |
| `?C` | Chip info |
| `??` | This help |
