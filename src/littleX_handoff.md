# littleX handoff: Desmos position calculator → PROS C++ → UKF

Context file for Claude in VS Code. Written Oct 25, 2026 from a claude.ai chat.
Read this before working on the littleX localization code.

---

## 1. Project summary

- **Robot:** VEX V5, programmed in **PROS (C++)**.
- **Goal:** localize the robot on the field using 4 V5 distance sensors + the IMU, and eventually a
  **UKF** (unscented Kalman filter) that fuses odometry, IMU, and distance readings.
- **Origin:** a Desmos graph, the "position calculator", made by Ryan Lin (finished May 1 2026, adjusted
  Oct 19 2026). It simulates the 4 sensors, computes what each one reads, then inverts those readings
  back into the robot's (x, y). It was reviewed, cleaned up, and verified in the chat (section 4).
- **Status:**
  - Desmos graph: done and verified.
  - C++ port of the graph (`wall_math.hpp`, `wall_reset.hpp/.cpp`, host test): written and verified
    against the Desmos formulas. Delivered as `littleX.zip`; may or may not be in the repo yet.
  - UKF: not started. Next step.

---

## 2. Conventions (keep these identical everywhere)

| Thing | Convention |
|---|---|
| Field | Square centered at (0, 0). Walls at x = ±b and y = ±b. Desmos uses **b = 70 in** (approximation, measure the real field). |
| Units | Inches, degrees. |
| Heading `t` / `theta` | **0° = facing +Y, positive = clockwise.** Same as PROS `pros::Imu::get_heading()` (0–360). |
| Robot frame | +x offset = **right** of robot center, +y offset = **forward**. |
| Sensor direction | Clockwise from robot forward: front 0°, right 90°, back 180°, left 270°. |
| Robot size | width w = 16 in, length l = 14 in (only used for drawing). |
| Sensor range | V5 distance sensor max **78.74 in (2000 mm)**. PROS `Distance::get()` returns mm; **9999 means nothing detected**. |

**Sensor offsets used in Desmos** (placeholders, the real robot must be measured, to each sensor's FACE):

| Sensor | x offset | y offset |
|---|---|---|
| Front | 0 | 5 |
| Back | 0 | −5 |
| Right | 5 | 2 |
| Left | −5 | 2 |

### Rotation used throughout
A robot-frame offset (ox, oy) maps to the field frame at heading t as:

```
dx = ox·cos(−t) − oy·sin(−t)  =  ox·cos t + oy·sin t
dy = ox·sin(−t) + oy·cos(−t)  = −ox·sin t + oy·cos t
```

Beam directions in the field frame (unit vectors):

```
front: ( sin t,  cos t)
right: ( cos t, −sin t)
back:  (−sin t, −cos t)
left:  (−cos t,  sin t)
```

In general: direction = (sin(t + a), cos(t + a)) where a is the sensor's mounting angle.

---

## 3. The Desmos graph, equation by equation (final version)

Desmos is in **degree mode**. Subscripts are written with underscores here.
Pose sliders: `x_1` ∈ [−60, 60], `y_1` ∈ [−60, 60], `t` (no upper limit, min −360).

### 3.1 Constants
```
b = 70                       (wall half-size)
w = 16,  l = 14              (robot width / length, for drawing)
x_frontOffset = 0,   y_frontOffset = 5
x_backOffset  = 0,   y_backOffset  = −5
x_rightOffset = 5,   y_rightOffset = 2
x_leftOffset  = −5,  y_leftOffset  = 2
```

### 3.2 Robot drawing
```
field  = polygon((−70,−70), (−70,70), (70,70), (70,−70))
center = (x_1, y_1)
c_1 = (x_1 + (l/2)cos(−t) − (w/2)sin(−t),  y_1 + (l/2)sin(−t) + (w/2)cos(−t))
c_2 = (x_1 + (l/2)cos(−t) + (w/2)sin(−t),  y_1 + (l/2)sin(−t) − (w/2)cos(−t))
c_3 = (x_1 − (l/2)cos(−t) + (w/2)sin(−t),  y_1 − (l/2)sin(−t) − (w/2)cos(−t))
c_4 = (x_1 − (l/2)cos(−t) − (w/2)sin(−t),  y_1 − (l/2)sin(−t) + (w/2)cos(−t))
robot = polygon(c_1, c_2, c_3, c_4)
```

### 3.3 Sensor positions (same pattern for all four, S ∈ {front, back, left, right})
```
x_S = x_1 + (x_SOffset·cos(−t) − y_SOffset·sin(−t))
y_S = y_1 + (x_SOffset·sin(−t) + y_SOffset·cos(−t))
```

### 3.4 Beam lines
Each beam is a line through the sensor. `F_S(y)` gives the x where the beam crosses height y
(used for top/bottom walls). `G_S(x)` gives the y where it crosses position x (used for side walls).

Front and back beams are parallel to the heading, so they use cot(t):
```
F_front(y) = (y − y_front) / cot(t) + x_front
G_front(x) = cot(t)·(x − x_front) + y_front
F_back(y)  = (y − y_back)  / cot(t) + x_back
G_back(x)  = cot(t)·(x − x_back)  + y_back
```

Left and right beams are perpendicular, so they use −tan(t):
```
F_left(y)  = (y − y_left)  / (−tan t) + x_left
G_left(x)  = −tan(t)·(x − x_left)  + y_left
F_right(y) = (y − y_right) / (−tan t) + x_right
G_right(x) = −tan(t)·(x − x_right) + y_right
```

### 3.5 Wall hit points (which wall each beam hits)
Desmos piecewise: the first condition that is true wins. `mod(t + 90, 360) ≤ 180` means the robot's
front faces upward (cos t ≥ 0). `mod(t, 360) ≤ 180` means it faces right (sin t ≥ 0).

```
a_front = {
  0 ≤ mod(t+90,360) ≤ 180  and  −b < F_front(b) < b   : (F_front(b), b),     top wall
  180 ≤ mod(t+90,360) ≤ 360 and −b < F_front(−b) < b  : (F_front(−b), −b),   bottom wall
  0 ≤ mod(t,360) ≤ 180     and  −b < G_front(b) < b   : (b, G_front(b)),     right wall
  otherwise                                            : (−b, G_front(−b))    left wall
}

a_back = {
  0 ≤ mod(t+90,360) ≤ 180  and  −b < F_back(−b) < b   : (F_back(−b), −b),
  180 ≤ mod(t+90,360) ≤ 360 and −b < F_back(b) < b    : (F_back(b), b),
  0 ≤ mod(t,360) ≤ 180     and  −b < G_back(−b) < b   : (−b, G_back(−b)),
  otherwise                                            : (b, G_back(b))
}

a_left = {
  0 ≤ mod(t+90,360) ≤ 180  and  −b < G_left(−b) < b   : (−b, G_left(−b)),
  180 ≤ mod(t+90,360) ≤ 360 and −b < G_left(b) < b    : (b, G_left(b)),
  0 ≤ mod(t,360) ≤ 180     and  −b < F_left(b) < b    : (F_left(b), b),
  otherwise                                            : (F_left(−b), −b)
}

a_right = {
  0 ≤ mod(t+90,360) ≤ 180  and  −b < G_right(b) < b   : (b, G_right(b)),
  180 ≤ mod(t+90,360) ≤ 360 and −b < G_right(−b) < b  : (−b, G_right(−b)),
  0 ≤ mod(t,360) ≤ 180     and  −b < F_right(−b) < b  : (F_right(−b), −b),
  otherwise                                            : (F_right(b), b)
}
```

### 3.6 Simulated sensor readings
```
d_f = distance((x_front, y_front), a_front)
d_b = distance((x_back,  y_back),  a_back)
d_l = distance((x_left,  y_left),  a_left)
d_r = distance((x_right, y_right), a_right)
```

### 3.7 Range limit
If a reading is over 78.74 in, the "limited" hit point collapses to the sensor itself, so later
checks like `a_limitedFront.y = b` fail and that sensor is skipped.
```
a_limitedFront = { d_f > 78.74 : (x_front, y_front),  a_front }
a_limitedBack  = { d_b > 78.74 : (x_back,  y_back),   a_back  }
a_limitedLeft  = { d_l > 78.74 : (x_left,  y_left),   a_left  }
a_limitedRight = { d_r > 78.74 : (x_right, y_right),  a_right }
beam drawings: polygon((x_S, y_S), a_limitedS)
```

### 3.8 Sensor polar offsets
`r_S` = distance from robot center to sensor. `d_angleS` = angle of that offset minus heading.
These use the **two-argument arctan** (fixed in the chat; the original used arctan(y/x), which put the
left sensor in the wrong quadrant and relied on a compensating sign flip).
```
r_S       = sqrt(x_SOffset² + y_SOffset²)
d_angleS  = arctan(y_SOffset, x_SOffset) − t
```
With this definition, the sensor's field-frame offset is exactly
`(r_S·cos(d_angleS), r_S·sin(d_angleS))`.

### 3.9 Inverting readings into position (final, cleaned-up version)
Each expression checks which sensor's limited hit is on the given wall and solves for the robot
center. First match wins; if none match, the result is undefined.

```
y_predFromFront (top wall, y = b) = {
  a_limitedFront.y = b : b − d_f·cos t − r_front·sin(d_angleFront),
  a_limitedLeft.y  = b : b − d_l·sin t − r_left ·sin(d_angleLeft),
  a_limitedRight.y = b : b + d_r·sin t − r_right·sin(d_angleRight),
  a_limitedBack.y  = b : b + d_b·cos t − r_back ·sin(d_angleBack)
}

y_predFromBack (bottom wall, y = −b) = {
  a_limitedFront.y = −b : −b − d_f·cos t − r_front·sin(d_angleFront),
  a_limitedLeft.y  = −b : −b − d_l·sin t − r_left ·sin(d_angleLeft),
  a_limitedRight.y = −b : −b + d_r·sin t − r_right·sin(d_angleRight),
  a_limitedBack.y  = −b : −b + d_b·cos t − r_back ·sin(d_angleBack)
}

x_predFromRight (right wall, x = b) = {
  a_limitedFront.x = b : b − d_f·sin t − r_front·cos(d_angleFront),
  a_limitedLeft.x  = b : b + d_l·cos t − r_left ·cos(d_angleLeft),
  a_limitedRight.x = b : b − d_r·cos t − r_right·cos(d_angleRight),
  a_limitedBack.x  = b : b + d_b·sin t − r_back ·cos(d_angleBack)
}

x_predFromLeft (left wall, x = −b) = {
  a_limitedFront.x = −b : −b − d_f·sin t − r_front·cos(d_angleFront),
  a_limitedLeft.x  = −b : −b + d_l·cos t − r_left ·cos(d_angleLeft),
  a_limitedRight.x = −b : −b − d_r·cos t − r_right·cos(d_angleRight),
  a_limitedBack.x  = −b : −b + d_b·sin t − r_back ·cos(d_angleBack)
}
```

General form behind all 16 cases: sensor position + d · beam direction = wall, solved for the center.
```
wall x = ±b:  x_1 = ±b − d·dirX − r·cos(d_angle)
wall y = ±b:  y_1 = ±b − d·dirY − r·sin(d_angle)
```

### 3.10 Final pose
```
w_final = ( { |x_predFromRight| ≤ b : x_predFromRight, x_predFromLeft },
            { |y_predFromFront| ≤ b : y_predFromFront, y_predFromBack } )
w_finalX = w_final.x
w_finalY = w_final.y
t_predicted = mod(−arctan(a_limitedFront.y − y_1, a_limitedFront.x − x_1) + 90, 360)
label: "predicted pose : (w_finalX, w_finalY, t_predicted°)"
```
Note: `t_predicted` uses the TRUE pose (x_1, y_1), so it only recovers the slider value. On the robot,
heading must come from the IMU.

### 3.11 Raw Desmos state
To get the exact current graph as JSON, open it on desmos.com, open the browser console, and run:
```js
copy(JSON.stringify(Calc.getState(), null, 1))
```
Expression ids worth knowing if editing via `Calc.setExpression({id, latex})`:
163 y_predFromFront, 164 y_predFromBack, 176 x_predFromRight, 178 x_predFromLeft,
147/152/170/171 d_angleRight/Left/Front/Back, 187 w_final, 202 t_predicted,
14 x_1, 13 y_1, 4 t, 54 b.

---

## 4. Review history (what was found and fixed)

**May 2026 version had:**
1. `a_right` used `F_left` instead of `F_right` (copy-paste bug). **Fixed in the Oct 19 version.**
2. Predictors used unlimited hit points, so out-of-range readings were used. **Fixed Oct 19.**
3. Hardcoded 70 instead of b, and a `0` "no estimate" sentinel. **Fixed Oct 19.**
4. Wall selection used `mod(abs(t), 270)`; replaced with `mod(t+90, 360)`. **Fixed Oct 19.**

**Fixed in this chat (Oct 25) via console:**
5. Single-argument `arctan(y/x)` replaced with two-argument `arctan(y, x)`; the left-sensor sign hack
   was removed so all four sensors use identical formulas. Predictors flattened into one piecewise each.
6. Removed an empty leftover expression (id 117).

**Verification run in Desmos after the fix** (console test moving the sliders):
```
true (-45, 23.5) @ 0°    → predicted (-45.00, 23.50)
true (-45, 23.5) @ 37°   → predicted (-45.00, 23.50)
true (50, -40) @ 120°    → predicted (50.00, -40.00)
true (-30, -50) @ 200°   → predicted (-30.00, -50.00)
true (40, 45) @ 300°     → predicted (40.00, 45.00)
true (0, 0) @ 40°        → predicted (NaN, NaN)   ← correct: every beam is over 78.74 in
```

**Still to do by hand in Desmos:** fix the note text "F(x) is where it intersects top line" → F(y), and
the typo "insersects".

---

## 5. The C++ port (already written)

Files (in `littleX.zip`, intended for a PROS project root):

- `include/localization/wall_math.hpp`: pure C++17, no PROS dependency.
  - `Pose {x, y, theta}`, `SensorMount {offX, offY, angle}`, `FieldConfig {halfSize=70, maxRange=78.74,
    minRange=0.8, maxIncidenceDeg=35}`.
  - `raycast(pose, mount, field)` → which wall the beam hits, distance, incidence angle. This is Desmos
    sections 3.3–3.6, done as a ray-vs-box intersection (smallest positive hit), not the piecewise.
    **This is the UKF measurement function h(x).**
  - `solveAxis(...)` → Desmos section 3.9 general form.
  - `wallReset(estimate, mounts, readings, field, gate=4)` → averages all accepted readings per axis.
- `include/localization/wall_reset.hpp`, `src/localization/wall_reset.cpp`: PROS wrapper.
  - Reads `pros::Distance` (mm → in, rejects 9999 / PROS_ERR / low confidence past 200 mm) and
    `pros::Imu::get_heading()` + `headingOffset`.
  - `correct(estimate)` updates only axes with a good reading; `globalFix()` tries four quadrant seeds
    and keeps the one whose predicted ranges best match the readings.
- `test/test_wall_math.cpp`: host test, `g++ -std=c++17 -I include test/test_wall_math.cpp -o t && ./t`
  - Compares `raycast()` against a literal port of the Desmos piecewise formulas: **25,168 rays, max
    difference 4.7e−13 in.**
  - Reproduces all six Desmos test poses above.
- `examples/main_example.cpp`: how to use it in `initialize/autonomous/opcontrol` (kept outside `src/`
  so it doesn't clash with main.cpp).

**Deliberate differences from Desmos (needed for a real robot):**
1. Heading from the IMU, not from the true pose.
2. The wall each sensor should see is chosen from the pose **estimate**, not from `a.y == b` on the
   measured hit (exact equality fails with noise).
3. Gate: a reading is used only if it's within 4 in of the expected range (rejects robots/game objects).
4. Incidence limit 35°: the beam is a cone and reads short on steep hits. Because of this, the
   (−45, 23.5) @ 37° pose gives no fix with defaults. Tune on real data.
5. Multiple sensors on the same axis are averaged (Desmos took the first match).

---

## 6. Next steps

1. **Measure the real robot/field:** field half-size, each sensor's offset to its face, ports, IMU port,
   start heading offset.
2. **Log real data** (readings, IMU heading, odometry) and tune `gate`, `maxIncidenceDeg`, `minConfidence`.
3. **Model field elements** that touch the walls if they block sensors often (the gate covers most cases).
4. **Build the UKF:**
   - State: [x, y, theta]. (Optionally add velocity later.)
   - Predict: odometry (tracking wheels or drive encoders) + IMU heading.
   - Update: for each sensor with a valid reading, measurement model h(x) = `raycast(x, mount).distance`.
     Skip sensors with no reading; reject updates whose innovation is too large (same idea as the gate).
   - Heading: either take it directly from the IMU or treat the IMU as a measurement of theta.
   - Mid-field there are dead zones with no in-range sensor (e.g. (0,0) @ 40°); the filter should coast
     on odometry there and snap back when walls come into range.
   - Write a host-side simulation first (like the Desmos graph): fake noisy odometry and readings,
     check the filter tracks the true pose, then move it onto the robot.
