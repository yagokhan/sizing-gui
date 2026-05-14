# Weapon System Motor Sizing — Theory & Usage

A single-page browser tool that sizes a brushless DC servomotor + drivetrain for
a weapon-system axis (gun elevation or turret traverse), given the mechanical
geometry, motion profile, disturbance environment, and motor candidate. The
tool computes peak/RMS torques referred to the motor shaft, the required DC bus
voltage and current, system bandwidth, suggested PI gains, and pointing
accuracy at engagement ranges — all live as you type, with a SOLVE step that
draws the full duty-cycle plots and limit envelopes.

Two equivalent files ship in this repo:

| File                  | Plotly source | When to use                                                                 |
|-----------------------|---------------|-----------------------------------------------------------------------------|
| `index.html`          | CDN           | You have internet; smaller file (~200 KB)                                   |
| `index_offline.html`  | bundled inline | Air-gapped / classified networks / no CDN access. Self-contained ~6 MB.    |

Tailwind CSS is loaded from a CDN for convenience but the page has an inline
fallback stylesheet, so it remains usable if `cdn.tailwindcss.com` is blocked.

> **Status:** v7.4x series. The tool is research-grade, not certified. Always
> validate the sizing against motor manufacturer datasheets and a thermal model
> before committing to hardware.

---

## Table of Contents

1. [Quick start](#1-quick-start)
2. [Architecture overview](#2-architecture-overview)
3. [Sign conventions & symbols](#3-sign-conventions--symbols)
4. [Theory of operation](#4-theory-of-operation)
   - 4.1 [Drivetrain models](#41-drivetrain-models)
   - 4.2 [Reflected (effective) inertia](#42-reflected-effective-inertia)
   - 4.3 [Duty-cycle profile generation](#43-duty-cycle-profile-generation)
   - 4.4 [Torque budget](#44-torque-budget)
   - 4.5 [RMS torque (thermal sizing)](#45-rms-torque-thermal-sizing)
   - 4.6 [Voltage and current at the operating point](#46-voltage-and-current-at-the-operating-point)
   - 4.7 [Power, efficiency, copper loss, regen](#47-power-efficiency-copper-loss-regen)
   - 4.8 [Stabilization disturbance](#48-stabilization-disturbance)
   - 4.9 [Closed-loop bandwidth & PI tuning](#49-closed-loop-bandwidth--pi-tuning)
   - 4.10 [Pointing accuracy](#410-pointing-accuracy)
5. [Inputs reference](#5-inputs-reference)
6. [Results reference](#6-results-reference)
7. [Compare A/B and scenarios](#7-compare-ab-and-scenarios)
8. [PDF export and printing](#8-pdf-export-and-printing)
9. [Troubleshooting](#9-troubleshooting)
10. [Glossary of symbols](#10-glossary-of-symbols)

---

## 1. Quick start

1. Open `index.html` (or `index_offline.html`) directly in any modern browser.
   No build, server, or installation is required.
2. Choose the axis in the header: **ELEVATION** (linear-actuator drive) or
   **TRAVERSE** (ring + pinion drive).
3. On the **INPUTS** view, fill in:
   - Mechanical geometry (lever arm, rollerscrew lead, ring/pinion teeth, …)
   - Load & disturbances (inertia, unbalance, friction, wind, firing)
   - Performance profile (target speed, accel, travel, dwell)
   - Stabilization (sinusoid amplitude/frequency, or imported CSV/JSON)
   - Motor candidate (K<sub>t</sub>, K<sub>e</sub>, R, J<sub>m</sub>, peak/cont
     torque, V<sub>dc</sub>, encoder bits, target bandwidth)
4. Watch the **LIVE SYSTEM OVERVIEW** strip update as you type — `i_tot`,
   `J_eff`, `ω_motor`, `T_peak est.`, `Inertia Ratio`, `V_req est.`. If those
   look sane, press **SOLVE**.
5. Inspect **RESULTS**:
   - **Overview**: badges + torque-speed diagram + duty-cycle profile + the
     three motor time-series (speed, current, power) referred to the motor shaft
   - **Power & Bandwidth**: peak mech/elec power, efficiency, time-constant,
     bandwidth, suggested PI gains, power–speed curve
   - **Precision**: encoder LSB at motor & at load, miss distance at 1/2/2.5/3/4 km,
     unit selector (mrad/μrad/mil/arcsec/rad)
   - **Analysis Charts**: torque budget breakdown, efficiency map, power overlay
   - **Visuals**: animated power-flow diagram, kinematics, margin gauges
6. Snapshot interesting points with **SNAP AS A / SNAP AS B** in the
   **COMPARE A/B** view to diff scenarios.
7. Save the inputs with **SAVE** (local-storage scenario) and export the full
   report with **PDF**.

The **EDIT INPUTS** button takes you back to the Inputs view at any time. The
status pill in the top toolbar reads `SOLVED` when results match the current
inputs and `NOT SOLVED` (stale) when they don't.

---

## 2. Architecture overview

The whole tool is a single HTML page. There is no backend.

```
+----------------------------------------------+
|  HEADER (axis toggle: elevation / traverse)  |
+----------------------------------------------+
|  VIEW NAV (Inputs / Results / Compare A/B)   |
+----------------------------------------------+
| INPUTS view                                  |
|  ├─ Live System Overview (block + kinematics)|
|  ├─ Live metrics strip (6 derived values)    |
|  └─ Parameter cards:                         |
|     ├─ Mechanical & Load                     |
|     ├─ Performance & Profile + Stabilization |
|     └─ Motor Parameters                      |
+----------------------------------------------+
| RESULTS view (sub-tabs)                      |
|  ├─ OVERVIEW   (badges, T-ω, duty, MTS×3)    |
|  ├─ POWER & BANDWIDTH                        |
|  ├─ PRECISION                                |
|  ├─ ANALYSIS CHARTS                          |
|  └─ VISUALS                                  |
+----------------------------------------------+
| COMPARE A/B view (scenario diff table)       |
+----------------------------------------------+
```

Key internal state (JavaScript):

| Variable | Meaning                                                    |
|----------|------------------------------------------------------------|
| `D`      | Defaults for elevation/traverse                            |
| `AP`     | Active parameter tree per axis (mutable from inputs)       |
| `ax`     | Current axis (`'elevation'` or `'traverse'`)               |
| `PM`     | Profile mode per axis (`'trap'`/`'sine'`/`'import'`)       |
| `SM`     | Stab mode per axis (`'sine'`/`'import'`)                   |
| `GM`     | Gear input mode per axis (`'step'`/`'single'`)             |
| `IM`     | Imported torque/speed profile data, if loaded              |
| `ISM`    | Imported stabilization disturbance φ(t), if loaded         |
| `UNITS`  | Per-input angular unit (`spd`/`acc`/`ang`: `'deg'`/`'rad'`)|
| `solved` | Are displayed results consistent with current inputs?      |

The compute pipeline is one pure function: `calc(p, axisName) → r`. `r` is then
threaded through:

```
solve() → rc() → rd()      (read DOM → AP[ax])
              → calc()     (compute r)
              → uR(r)      (update result texts + Precision/Bandwidth)
              → uC(r)      (update Plotly charts: T-ω, duty, time-series, etc.)
              → uB(r.Jrat) (update inertia-match bar)
            → swView('results')
```

All chart rendering is via [Plotly.js](https://plotly.com/javascript/) — both
versions accept `Plotly.react` updates so subsequent solves are fast.

---

## 3. Sign conventions & symbols

- Positive load speed `ω_L` corresponds to positive motor speed `ω_M = ω_L · i_tot`.
- During deceleration, the motor brakes against the rotation: `T·ω < 0` ⇒ regen.
- All torques in the trap profile (`T_a`, `T_d`, `T_h`) are **referred to the
  motor shaft** (after dividing the load torque by `η · i_tot` and adding the
  inertial contribution).
- The motor count `n` divides total torque: `T_per_motor = T_total / n`.
  Inertia adds: `J_total_motor = n · J_m`.
- Linear-actuator forces are positive when extending the rod.

Conversion constants used in the code:

| Unit          | Factor (to rad)         |
|---------------|-------------------------|
| `deg`         | `π/180`                 |
| `mrad`        | `1e-3`                  |
| `μrad`        | `1e-6`                  |
| `mil` (NATO)  | `π/3200` (full turn = 6400 mil) |
| `arcsec`      | `1/206264.806`          |

Full glossary is at the end of the document.

---

## 4. Theory of operation

### 4.1 Drivetrain models

The tool ships two physically distinct drivetrains, plus an analytical
**single-ratio** override for both axes.

#### 4.1.1 Elevation — linear actuator (rollerscrew + lever)

The actuator stroke pushes the barrel pivot. Geometric reduction comes from
three terms in series:

1. Motor → screw shaft gear ratio `i_gk`
2. Rollerscrew kinematic ratio: one revolution of the screw moves the rod by
   the lead `h` (mm/rev)
3. Lever arm `L` from the trunnion to the actuator hinge on the barrel

Per-revolution motor displacement equals lever-arm sweep, so the **total speed
reduction** from motor to barrel is:

```
i_tot  =  i_gk · (2π · L) / h
```

The **total efficiency** is the product of stage efficiencies:

```
η_tot  =  η_gk · η_s
```

Typical values: `η_gk ≈ 0.95` (single gear stage), `η_s ≈ 0.85–0.92` for a
preloaded rollerscrew.

The actuator rod force at the peak operating point is:

```
F_p  =  T_motor_total · i_gk · 2π · η_tot / h
```

The actuator-geometry visual on the Inputs view also evaluates the
transmission angle μ (angle between the actuator and the barrel at the hinge);
μ < 30° is colored red because force × sin(μ) collapses and the actuator
demands far more force than it delivers to the barrel.

#### 4.1.2 Traverse — internal ring gear + pinion + gearbox

The motor drives a pre-reduction gearbox of ratio `i_gb`, then a pinion (`z_2`
teeth) engages an internal ring gear (`z_1` teeth) attached to the turret.
Total speed reduction:

```
i_tot  =  (z_1 / z_2) · i_gb
```

The tool links the gear inputs automatically:

- `d = z · m` (pitch diameter = teeth × module)
- Edit any two of `m`, `d_1`, `z_1` (and `m`, `d_2`, `z_2`) and the third
  re-derives.

Total efficiency `η_tot` is a single user-entered combined value for the
gearbox + mesh.

#### 4.1.3 Single-ratio mode

If you only want to study the dynamics with an aggregate gearing assumption,
switch the **Gear Input Mode** to **Single ratio** and enter `i_tot`, `η_tot`,
and `J_gb` directly. Geometry-specific outputs (actuator force, transmission
angle, gear teeth visuals) are then disabled for that axis.

---

### 4.2 Reflected (effective) inertia

Inertia seen by the motor shaft is the sum of motor + drivetrain + load
inertia reflected through the squared speed ratio (and divided by drivetrain
efficiency for power balance):

```
J_eff  =  n · J_m  +  J_gb  +  J_L / (η · i_tot²)
```

The **inertia ratio** is the figure of merit for servo dynamics:

```
J_ratio  =  J_L / (η · i_tot² · n · J_m)
```

Rule of thumb:
- `≤ 5:1`  Excellent — bandwidth limited only by motor & power stage
- `5–10:1` Acceptable — control gains will need care
- `>10:1`  Poor — closed-loop bandwidth and disturbance rejection suffer

The tool color-codes the inertia bar accordingly.

---

### 4.3 Duty-cycle profile generation

Three profile modes are supported.

#### 4.3.1 Trapezoidal (default)

For target speed `ω_L`, acceleration `α_L`, total travel `θ`, dwell `t_4`:

```
t_a   = ω_L / α_L                      (accel time, motor-side equivalent ω_M, α_M)
θ_a   = ½ · α_L · t_a²                  (angle covered during accel)

If 2·θ_a ≤ θ:                          (full trapezoid)
   t_1 = t_a
   t_3 = t_a
   t_2 = (θ − 2·θ_a) / ω_L              (constant-speed phase)
Else:                                   (triangle — accel never plateaus)
   t_1 = t_3 = √(θ / α_L)
   t_2 = 0
```

Cycle time = `t_1 + t_2 + t_3 + t_4`. The status row below the inputs shows
`Slew time` and `Cycle` continuously as you type.

Per-phase **motor-shaft torque** with sign convention (decel torque can be
negative ⇒ regen):

```
Phase 1 (accel):        T_1  = +T_acc + T_d              (+ T_fire spike at the very start)
Phase 2 (constant):     T_2  = +T_d
Phase 3 (decel):        T_3  = −T_acc + T_d              (signed)
Phase 4 (dwell):        T_4  = T_h                       (hold against unbalance only)
```

where
- `T_acc = J_eff · α_M` (inertial torque, motor side)
- `T_d   = (T_w + T_fr + T_ub) / (η_tot · i_tot)` for elevation; without wind
  for traverse
- `T_h   = T_ub / (η_tot · i_tot)` (static hold against unbalance at zero speed)
- `T_fire` is the firing impulse, applied only during a short (~50 ms) spike at
  the start of Phase 1 and excluded from RMS

The peak motor-side torque is the worst-case across all four phases (the fire
spike usually wins on a gun system).

#### 4.3.2 Sinusoidal

User specifies torque amplitude (Nm at motor), frequency (Hz), duration (s).
The tool generates `T(t) = A · sin(2π f t) / n`, with `ω(t) = 0` (i.e. pure
torque excitation, suitable for stall-test sizing). RMS and peak are sampled
numerically.

#### 4.3.3 Imported (CSV / JSON)

Upload a file with one of these formats:

- CSV: `time,torque` or `time,torque,speed`
  - First column is seconds, second is Nm (referred to the **motor** shaft,
    total — the tool divides by `n` internally), optional third is rad/s.
- JSON: `[{"t": 0.0, "T": 5.2}, {"t": 0.05, "T": 5.1, "w": 0.4}, …]`

The download-template button writes a tab-delimited skeleton.

---

### 4.4 Torque budget

At the peak operating point, the per-motor torque has up to five components
(displayed in the **Analysis ▸ Torque Budget** chart as a stacked bar):

| Component         | Source                                | Always present? |
|-------------------|---------------------------------------|-----------------|
| Acceleration      | `J_eff · α_M`                         | Yes (with motion) |
| Friction          | `T_fr / (η · i_tot)`                  | Yes             |
| Unbalance         | `T_ub / (η · i_tot)`                  | Yes (gravity)   |
| Wind drag         | `T_w / (η · i_tot)` (elev only)       | If non-zero     |
| Firing impulse    | `T_fire / (η · i_tot)` (elev only)    | Briefly at recoil |

Target distribution at peak demand: **acceleration ~40–70 %** of `T_peak`. If
friction or unbalance dwarfs `J·α`, the drivetrain is oversized for the
required dynamics (consider gearing down or picking a smaller motor). A
dominant firing bar means recoil drives the sizing — check whether that's a
true design case or a rare event you should size for separately.

---

### 4.5 RMS torque (thermal sizing)

Continuous-duty thermal sizing is governed by the cycle-RMS torque, which the
tool computes excluding the firing transient (assumed << cycle time):

```
T_RMS_total  =  √( (T_1² · t_1 + T_2² · t_2 + T_3² · t_3 + T_4² · t_4) / t_cycle )

T_RMS_per_motor  =  T_RMS_total / n
I_RMS_per_motor  =  T_RMS_per_motor / K_t
```

For sizing acceptance: `T_RMS_per_motor ≤ T_cont` (the motor's S1 continuous
rating, with thermal margin for ambient / mounting). The duty-cycle chart on
the Overview tab draws a green dashed line at `T_RMS / motor` so you can see
how the profile compares to the thermal-equivalent constant torque.

---

### 4.6 Voltage and current at the operating point

For each operating point `(ω, T)` at the motor shaft:

```
I_motor  =  T / K_t                              (current per motor)
V_req    =  I · R  +  K_e · ω                    (Ohmic drop + back-EMF)
```

`V_req` must satisfy `V_req ≤ V_dc` (with margin). The **Voltage Margin** badge
on the Overview tab evaluates `V_dc − V_req` (≥ +15 % ideal, ≥ +5 %
acceptable, < +5 % risky / red). The torque-speed diagram draws the motor's
peak (red, `T_pk` saturated) and continuous (blue dashed, `T_co` saturated)
envelopes:

```
T_env(ω)  =  K_t · max(0, (V_dc − K_e·ω)) / R     (electrical-side max torque at ω)
```

clipped to the motor's rated peak or continuous limit at low speed.

The no-load speed is the intercept where the envelope meets the speed axis:

```
ω_NL  =  V_dc / K_e          (rad/s, with K_e in V/(rad/s))
```

This is rendered as a dotted reference line on the **Motor Speed vs Time**
chart.

---

### 4.7 Power, efficiency, copper loss, regen

Per-motor instantaneous quantities along the duty cycle:

```
P_mech    =  T · ω                       (mechanical output, sign tracks T·ω)
I         =  |T| / K_t
P_cu      =  I² · R                      (copper loss, always positive)
P_elec_in =  K_e · ω · I  +  P_cu        (≈ T·ω + P_cu  when motoring)
```

Sign analysis:
- `T · ω > 0`: motoring — drive provides power, `P_elec_in = P_mech + P_cu`.
- `T · ω < 0`: regenerating — energy flows back to bus, `P_elec_in = −|P_mech| + P_cu`.
  The dump resistor / bus capacitance must handle this on top of the user
  supply rails. The **Motor Power vs Time** chart draws the zero-axis line so
  regen is visually obvious below the baseline.

The numbers on the **Power & Bandwidth** sub-tab are evaluated at the **peak**
operating point:

```
P_mech_pk   =  T_pk_total · ω_M
P_elec_pk   =  n · V_req · I_pk          (per-motor V·I summed over motors)
η_system    =  P_mech_pk / P_elec_pk     (instantaneous, at peak only)
```

These differ from cycle-average values; for energy budgeting integrate the
per-sample arrays in `r.prof` over time.

---

### 4.8 Stabilization disturbance

Stabilization couples base motion (vehicle pitch/roll, hull shake) into the
weapon line of sight. The tool supports two stabilization modes:

#### Sinusoidal

User enters amplitude `φ_max` (mrad, peak, referred to the load) and frequency
`f` (Hz). The tool reflects this to the motor shaft:

```
A_motor  =  (φ_max / 1000) · i_tot                  (rad at motor)
ω_sf     =  2π · f
ω_peak_motor =  A_motor · ω_sf                      (peak speed)
α_peak_motor =  A_motor · ω_sf²                     (peak accel)

T_stab_peak  =  J_eff · α_peak_motor                (dynamic only)
T_stab_total_peak  =  T_stab_peak + |T_hold|
T_stab_RMS         =  √( (T_stab_peak/√2)² + T_hold² )
```

`T_hold = T_ub / (η · i_tot)` is the static torque the motor must hold even at
zero speed to fight unbalance.

#### Imported disturbance

Upload a φ(t) record in mrad and the tool will differentiate twice to extract
α(t), then reflect to motor torque `T(t) = J_eff · α(t) · i_tot`. If `ω(t)` is
already in the file the tool uses it directly instead of differentiating.

The Overview sub-tab renders the stabilization torque overlay on the
**Motor Speed / Current / Power vs Time** charts as a dotted amber curve.

---

### 4.9 Closed-loop bandwidth & PI tuning

The mechanical pole of the motor + reflected inertia governs how fast the
plant can respond before active control even gets involved:

```
τ_m   =  J_eff · R / (K_t · K_e)               [s]   (mechanical time constant)
ω_m   =  1 / τ_m                              [rad/s] (open-loop pole)
f_m   =  ω_m / (2π)                            [Hz]
```

A reasonable rule-of-thumb is that a **position-loop bandwidth of ~5·f_m** is
the upper bound for a well-tuned servo (this assumes a current loop ~10× the
velocity-loop bandwidth, and a velocity loop above `f_m`). Beyond that, the
motor / drive simply cannot track commands.

For a velocity loop with crossover `ω_c = 2π · BW_target`, the tool suggests:

```
K_p  =  J_eff · ω_c                      [Nm·s/rad]
K_i  =  K_p · ω_c / 5                    [Nm/rad]
```

These are starting points — refine on hardware with notch filters around
structural resonances, load compensation, and feedforward as appropriate.

---

### 4.10 Pointing accuracy

The encoder counts per revolution at the motor:

```
CPR  =  2^N            (where N = "Encoder Bits")
```

One LSB at the motor shaft:

```
Δθ_motor  =  2π / CPR        [rad]
```

Reflected to the load (the barrel/turret) through the drivetrain reduction:

```
Δθ_load  =  Δθ_motor / i_tot    [rad]
```

This is the **pointing quantum** — the smallest angle the system can command.
On the Precision sub-tab it is displayed in five selectable units:
`mrad / μrad / mil (NATO/milyem) / arcsec / rad`. The selector changes the
displayed unit on every angle-bearing field together (Δθ at Motor, Δθ at Load,
Δθ at Load alt-ref, Max slew rate, Quantization LSB).

Linear miss-distance at engagement range:

```
miss(R)  =  Δθ_load · R     [m]
```

Five rows (1 km / 2 km / 2.5 km / 3 km / 4 km) auto-scale their linear unit by
magnitude:

- ≥ 1 m   → `m`
- ≥ 1 cm  → `cm`
- ≥ 1 mm  → `mm`
- otherwise → `μm`

Decoupled from the angular selector — it's a physical distance, not an angle.

---

## 5. Inputs reference

The Inputs view is organised in three cards. Hover any label (most show a
dotted underline) to see a tooltip with the precise definition / sizing
relevance.

### 5.1 Mechanical & Load

| Field | Description |
|-------|-------------|
| **Gear Input Mode** | `Step-by-step` (geometric) or `Single ratio` (aggregate) |
| Rollerscrew Pitch `h` (elev) | Linear travel per screw revolution (mm/rev) |
| Rollerscrew Diameter `D_s` (elev) | Pitch-circle diameter; sets helix angle |
| Motor-Screw Ratio `i_gk` (elev) | Gear stage between motor and screw shaft |
| Rollerscrew Efficiency `η_s` (elev) | Typically 0.85–0.92 preloaded |
| Gear Efficiency `η_gk` (elev) | Motor-screw stage |
| Lever Arm `L` (elev) | Trunnion-to-hinge distance (m) |
| Hinge Offset `L_h` (elev) | Trunnion to actuator hinge on barrel (mm) |
| Base ΔX / ΔY (elev) | Position of actuator base pivot relative to trunnion |
| `L_min` / `L_max` (elev) | Retracted / extended actuator length (cylinder + rod) |
| Elev θ_min / θ_max | Range of barrel elevation for animation & validation |
| Module `m`, Ring `d_1`/`z_1`, Pinion `d_2`/`z_2` (trav) | Ring + pinion geometry (auto-linked: `d = z·m`) |
| Gearbox Ratio `i_gb` (trav) | Pre-reduction between motor and pinion |
| Total Efficiency `η_tot` (trav) | Combined gearbox + mesh |
| Gearbox Inertia `J_gb` | Reflected to motor shaft |
| Load Inertia `J_L` | At the load side |
| Unbalance `T_ub` | Gravity torque (CG offset) — present even at zero speed |
| Friction `T_fr` | Coulomb / viscous, opposing motion |
| Wind `T_w` (elev) | Aerodynamic drag estimate at max slew |
| Firing `T_fire` (elev) | Peak recoil torque, added to peak only |

### 5.2 Performance & Profile

Choose the **Profile Mode** (Trapezoidal / Sinusoidal / Import CSV-JSON), then:

| Field | Description |
|-------|-------------|
| Target Speed | Peak load-side angular speed (deg/s by default; click the unit label to toggle rad/s) |
| Target Accel | Acceleration during the ramp-up phase |
| Travel Angle | Total angle per laying cycle (deg or rad) |
| Dwell Time `t_4` | Pause at rest between slews |
| Torque Amplitude / Freq / Duration | Sine mode parameters |
| Import CSV / JSON | Upload an external profile |

**Stabilization** is always visible below:

| Field | Description |
|-------|-------------|
| Stab Mode | Sinusoidal or Imported disturbance |
| Stab Amplitude `φ_max` | Peak base-motion at load (mrad) |
| Stab Frequency `f` | Hz |
| Upload Disturbance | CSV: `time,phi` (mrad) or `time,phi,omega,alpha` |

Two preview plots render live below: the lay-cycle profile and the stab
disturbance.

### 5.3 Motor Parameters

| Field | Symbol | Description |
|-------|--------|-------------|
| Motor Count | `n` | Number of motors in parallel on this axis (1…8). Torque divides; inertia adds |
| Torque Constant | `K_t` | Nm/A |
| Back-EMF Constant | `K_e` | V/(rad/s) — for BLDC, `K_e ≈ K_t` in SI |
| Resistance | `R_mt` | Per-motor winding resistance, Ω |
| Rotor Inertia | `J_m` | Per motor, kg·m² |
| Peak Torque Limit | `T_pk` | Short-time, per motor |
| Cont. Torque Limit | `T_co` | S1 rating, per motor |
| DC Bus Voltage | `V_dc` | Drive supply |
| Encoder Bits | `N` | CPR = 2^N |
| Mission Duration | — | Total run time for energy estimates |
| Target Pos. BW | — | Desired closed-loop bandwidth (Hz) — must be ≤ Max achievable |

---

## 6. Results reference

The Results view has five sub-tabs.

### 6.1 Overview

- **Three status badges** (Inertia Ratio · Voltage Margin · Peak Torque /
  Motor), each colored OK / warn / danger and showing the worst headroom.
- **Gun Laying / Stabilization** cards: every line carries a tooltip
  explaining its meaning. RMS values are evaluated over the cycle excluding
  firing transients.
- **Torque–Speed Diagram**: motor envelopes (peak red filled, cont. blue
  dashed) with the laying and stabilization operating points marked. Compare
  visually to ensure both points sit comfortably inside the envelope.
- **Duty Cycle Profile**: per-motor T(t) and ω(t) over one cycle, with the
  thermal-equivalent `M_rms / motor` line.
- **Motor Speed / Current / Power vs Time**: per-motor curves with reference
  lines. Layout mirrors the Torque-Speed Diagram (same fonts, margins,
  hover-style):
  - Speed chart marks `ω_NL = V_dc / K_e`.
  - Current chart marks per-motor `I_peak = T_pk / K_t` and `I_cont = T_co / K_t`.
  - Power chart marks the zero axis so regen below is obvious.

### 6.2 Power & Bandwidth

- Three top metrics: Peak Mech. Power, Peak Elec. Power, System Efficiency.
- Dynamic Response card: `τ_m`, `ω_m`, `f_m`, Max BW, Target BW (user), and
  the suggested velocity-loop PI gains.
- Power–Speed Curve: `P_mech(ω) = T(ω)·ω` along the motor envelope, with the
  duty-cycle peak operating point.

### 6.3 Precision

- Top unit selector (mrad / μrad / mil / arcsec / rad). All angle-bearing
  fields below switch together.
- Four top cards: Encoder Resolution (counts/rev — unit-independent), Δθ at
  Motor, Δθ at Load, Δθ at Load (alt ref — same value, given for
  cross-reference).
- **Pointing Accuracy** table:
  - Total reduction `i_tot` (dimensionless)
  - Resolution @ 1 km / 2 km / 2.5 km / 3 km / 4 km (linear miss distance,
    auto-scaled μm/mm/cm/m by magnitude — independent of the angular
    selector)
  - Max slew rate (encoder limited) in `<unit>/s`
  - Quantization LSB at barrel

### 6.4 Analysis Charts

- **Torque Budget (B1)**: stacked decomposition of `T_peak / motor` into
  Accel / Friction / Unbalance / Wind / Firing. Dashed lines mark the peak
  and continuous motor limits.
- **Efficiency Map η(T, ω) (B2)**: contour plot of the motor's
  electromechanical efficiency over the operating envelope, with the
  duty-cycle peak point marked. Aim for the high-η band (typically 80–90 %).
- **Power–Speed Overlay (B3)**: P_mech and P_elec curves along the envelope
  with the operating point.

### 6.5 Visuals

- Animated power-flow block diagram.
- Linear actuator geometry (elev) or gear-train kinematics (trav), animated.
- Margin Dashboard — four gauges for Peak / Limit, Voltage Used, Inertia
  Ratio, RMS / Cont.

---

## 7. Compare A/B and scenarios

The **COMPARE A/B** view is for what-if analysis. After SOLVING, click
**SNAP AS A** to capture the current result, change inputs, SOLVE again, then
**SNAP AS B**. The comparison table shows every key metric for both snapshots
plus the delta (B − A). The snapshot is shallow but full — it serializes `r`
at capture time and does not change when inputs change.

**Scenarios** persist via `localStorage`:

- **SAVE** prompts for a name and stores the entire `AP`, axis, and profile
  modes locally.
- The **Load Scenario…** dropdown pulls a stored scenario back into the GUI
  and re-pops all the input fields.
- Scenarios are per-browser-profile; they do not leave your machine.

---

## 8. PDF export and printing

The **PDF** button (red, top toolbar) renders a multi-page A4-landscape report
that includes the axis, timestamp, input echo, results, badges, every chart
on every sub-tab, and the system visuals. The exporter steps through each
sub-tab off-screen, lets Plotly draw, then captures each `<div>` as an image —
so all charts make it into the PDF even if the user hasn't visited that tab.

The native **Print** button (printer icon) uses the browser print dialog with
a print-only stylesheet that hides the Inputs view and modebars, and forces a
white background.

---

## 9. Troubleshooting

**The page loads but nothing computes.**
- Check the DevTools console (F12). The most common cause is a CDN blocked by
  your network (look for failed loads of `plotly-...min.js` or
  `cdn.tailwindcss.com`). Switch to `index_offline.html`.
- If you see `Plotly library failed to load`, open the offline version.

**SOLVE button does nothing.**
- Verify Plotly is loaded (`typeof Plotly` in console should return `'object'`).
- Check that all required input fields have valid numeric values; the tool
  guards against zeros (`Math.max(..., .001)`) but a stray non-numeric input
  may break the chain.

**The Motor Speed / Current / Power charts show "Press SOLVE…" forever.**
- Pre-v7.41 bug — fixed. If you still see it, hard-refresh (Ctrl/Cmd+Shift+R)
  to bust the browser cache.

**Encoder bit changes don't propagate to Precision values.**
- Pre-v7.45 bug — fixed. Make sure you press SOLVE after changing values.

**Plots look the wrong size after I switch tabs.**
- Resize once with the window. The tool calls `Plotly.Plots.resize` on every
  sub-tab switch but Plotly occasionally lags on first draw into a previously
  hidden container.

**My imported CSV profile shows "No data — upload CSV/JSON file".**
- Confirm the CSV is comma-separated (not semicolon — Excel locale issue),
  starts with `time,torque` headers, and time is in seconds.

**The PDF is missing some charts.**
- The exporter waits ~250 ms per sub-tab. Slow machines may need a second run.
  Wait until the **PDF** button text returns to its default before clicking
  again.

---

## 10. Glossary of symbols

| Symbol | Meaning | Units |
|--------|---------|-------|
| `n`     | Motor count (parallel) | — |
| `K_t`   | Torque constant | Nm/A |
| `K_e`   | Back-EMF constant | V/(rad/s) |
| `R`, `R_mt` | Per-motor winding resistance | Ω |
| `J_m`   | Motor rotor inertia (per motor) | kg·m² |
| `J_gb`  | Gearbox/drivetrain inertia (motor-side) | kg·m² |
| `J_L`   | Load inertia | kg·m² |
| `J_eff` | Effective inertia at motor shaft | kg·m² |
| `J_ratio` | Reflected load inertia / motor inertia | — |
| `i_tot` | Total speed reduction motor→load | — |
| `i_gk`  | Motor → rollerscrew gear ratio | — |
| `i_gb`  | Gearbox ratio (traverse) | — |
| `z_1`, `z_2` | Ring / pinion tooth counts | — |
| `m`     | Gear module | mm |
| `d_1`, `d_2` | Pitch diameters | mm |
| `h`     | Rollerscrew lead | mm/rev |
| `L`     | Lever arm trunnion→hinge | m |
| `η_tot` | Total drivetrain efficiency | — |
| `T_ub`  | Unbalance torque (load) | Nm |
| `T_fr`  | Friction torque (load) | Nm |
| `T_w`   | Wind drag torque (load) | Nm |
| `T_fire`| Firing impulse torque (load) | Nm |
| `T_d`   | Disturbance torque at motor = (T_w+T_fr+T_ub)/(η·i_tot) | Nm |
| `T_acc` | Acceleration torque at motor = `J_eff · α_M` | Nm |
| `T_h`   | Holding torque at motor = `T_ub/(η·i_tot)` | Nm |
| `T_pk`, `T_co` | Motor peak / continuous torque rating | Nm/motor |
| `T_peak_total`, `T_peak_motor` | Peak required torque (sum / per motor) | Nm |
| `M_rms`, `M_rms_motor` | RMS torque over cycle (total / per motor) | Nm |
| `I_pk`, `I_rms` | Peak / RMS per-motor current | A |
| `V_dc`  | DC bus voltage | V |
| `V_req` | Required drive voltage at operating point | V |
| `ω_M`, `ω_L` | Motor / load angular speed | rad/s |
| `α_M`, `α_L` | Motor / load angular acceleration | rad/s² |
| `ω_NL`  | No-load motor speed `= V_dc/K_e` | rad/s |
| `τ_m`   | Mechanical time constant `= J_eff·R/(K_t·K_e)` | s |
| `ω_m`, `f_m` | Open-loop mechanical pole | rad/s, Hz |
| `BW`    | Position-loop bandwidth (Hz) | Hz |
| `K_p`, `K_i` | Suggested velocity-loop gains | Nm·s/rad, Nm/rad |
| `φ_max` | Stab disturbance amplitude (load) | mrad |
| `T_stab_peak`, `T_stab_RMS` | Stab torque metrics (total) | Nm |
| `T_hold` | Static holding torque against `T_ub` | Nm |
| `CPR`   | Encoder counts per revolution `= 2^N` | counts/rev |
| `Δθ_motor`, `Δθ_load` | Encoder LSB at motor / at load | rad (displayed in selected unit) |
| `LSB` (at barrel) | Equivalent to `Δθ_load` | rad |
| `μ` (transmission angle) | Angle between actuator and barrel at hinge (elev) | deg |

---

*Generated alongside `index.html` v7.45+. If you find a discrepancy between
this document and the code, the code is authoritative — please open an issue
on the repository so this README can be updated.*
