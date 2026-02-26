# Pneumatic Levitator — Control Engineering Project

A control engineering project developed under the supervision of **Prof. Leonardo Bermeo Clavijo**, implementing system identification and a two-degree-of-freedom PID controller to levitate a ball inside a vertical transparent tube using a fan as the actuator.

The system is an inherently **unstable third-order plant** with an integrating pole, making it a representative example of balance control systems (analogous to aircraft pitch control or DC-DC converters). The controller runs on an **ESP32** microcontroller and reads ball position using a **VL53L0X time-of-flight laser sensor**.

---

## System Overview

The plant transfer function (linearized around the operating point at the midpoint of the tube) is:

```
G(s) = b / [s(τ₁s + 1)(τ₂s + 1)]
```

For identification and controller design, the following alternative model with time delay is used:

```
Ga(s) = b / [s(τ₁s + 1)] · e^(-Ls)
```

Identified parameters from the Åström relay experiment:

| Parameter | Value |
|-----------|-------|
| b (system gain) | 53.3 |
| τ₁ (time constant) | 2.00039 s |
| L (time delay) | 0.27 s |
| Ku (ultimate gain) | 0.07 |
| ωu (ultimate frequency) | 1.33 rad/s |

---

## Repository Structure

```
pneumatic-levitator/
│
├── firmware/
│   ├── levitador_histeresis/
│   │   └── levitador_histeresis.ino    # Åström relay experiment for system identification
│   │
│   ├── levitador_control_prop/
│   │   └── levitador_control_prop.ino  # Proportional controller (used to estimate ueq)
│   │
│   └── levitador_2DOF_PID/
│       ├── levitador_control_prac3.ino # Final 2-DOF PID controller
│       └── definitions.h               # Hardware config, PWM setup, sensor driver, moving average filter
│
├── cad/
│   ├── Proyecto_control.stl            # STL file for 3D printing the base
│   └── Proyecto_control.f3d            # Fusion 360 source file
│
├── docs/
│   └── modelo_levitador.pdf            # Mathematical model and identification guide (Prof. Bermeo)
│
└── README.md
```

---

## Identification Procedure

The system is unstable, so identification is performed in **closed loop** using the Åström relay (ON-OFF with hysteresis).

**Step 1 — Estimate ueq (equilibrium voltage)**
- Open loop: apply fixed voltages with `voltsToFan()` until the ball slowly rises. This gives a rough estimate.
- Closed loop: run the proportional controller (`levitador_control_prop`) and tune `kp` until the ball stabilizes. The filtered control signal printed on the serial monitor is the exact value of `ueq`.

**Step 2 — Åström relay experiment (`levitador_histeresis`)**

Set `ueq` and reference `r = L/2` (midpoint of the tube). Choose a hysteresis value `eh` (2–5 cm) and a control deviation `ud` (0.25–2 V) so that the ball oscillates symmetrically with a total excursion of ~5 cm. Record ~120 seconds of data.

From the oscillation data, compute:

```
Ku = (4·ud) / (π·√(a² - eh²))
ωu = 2π / Tu
```

**Step 3 — Determine b, τ₁, L**

Increase `eh` so the ball traverses most of the tube. During an ascending cycle, read off y₁, y₂ and their timestamps t₁, t₂, then:

```
b  = (y₂ - y₁) / ((t₂ - t₁)·ud)
τ₁ = √((Ku²·b² - ωu²) / ωu⁴)
L  = (π/2 - arctan(τ₁·ωu)) / ωu
```

---

## Controller Design — 2-DOF PID

A proportional controller alone was insufficient to keep the ball at the reference. A **two-degree-of-freedom (2-DOF) PID** controller with derivative filter and anti-windup was designed using pole placement, targeting the closed-loop poles at:

```
ωn = 15 rad/s,  ζ = 1.5  (overdamped response)
```

The control law is:

```
P = kp·(β·r - y)
D(k) = ad·D(k-1) - bd·(y(k) - y(k-1))
U = P + D + I
I(k+1) = I(k) + bi·(r - y) + br·(usat - U)   ← anti-windup
```

Gains computed from identified parameters (`b = 53.33`, `τ = 2`, `ωn = 15`, `ζ = 1.5`, `N = 14`):

| Gain | Formula | Value |
|------|---------|-------|
| kp | τ·ωn²·(4ζ+1) / b | — |
| ki | 2·τ·ωn³ / b | — |
| kd | (τ·ωn·(2ζ+2) - 1) / b | — |

The reference weight `β = 0` eliminates proportional kick on reference changes (set-point weighting).

---

## Hardware

| Component | Detail |
|-----------|--------|
| Microcontroller | ESP32 |
| Position sensor | VL53L0X (time-of-flight laser, I²C) |
| Actuator | Fan controlled via PWM (25 kHz, 11-bit resolution) |
| PWM pin | GPIO 2 |
| Sensor SDA | GPIO 21 |
| Sensor SCL | GPIO 22 |
| Power supply | 12 V |

The base structure was designed in Fusion 360 and 3D printed — STL and source files are in `cad/`.

---

## Dependencies

- [VL53L0X Arduino library](https://github.com/pololu/vl53l0x-arduino) — Pololu
- Arduino ESP32 core

---

## Authors

- Sebastian Jaramillo Verdugo
- Rosemberth Steeven Preciga Puentes
- Jorge Santiago Camargo Guerrero
- Gabriel Felipe Ostos Iguavita

**Supervisor:** Prof. Leonardo Bermeo Clavijo

---

## References

- K.J. Åström and T. Hägglund. *Advanced PID Control*. ISA, 2006.
- K.J. Åström and R.M. Murray. *Feedback Systems: An Introduction for Scientists and Engineers*, 2nd ed. Princeton University Press, 2021.
- L. Bermeo Clavijo. "Levitador neumático: Modelo matemático y método pragmático de identificación."
