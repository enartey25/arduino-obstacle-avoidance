# 🏎️ Autonomous RL Car: 4-Direction Simulation-to-Arduino Deployment

**Target Hardware:** Arduino Uno (ATmega328P, 16 MHz, 2 KB SRAM, 32 KB Flash)  
**Sensors:** 4× HC-SR04 Ultrasonic Distance Sensors (**UP**, **DOWN**, **LEFT**, **RIGHT**)  
**Movements:** 4 Discrete Actions (**UP**, **DOWN**, **LEFT**, **RIGHT**)  
**Motor Driver:** L298N Dual H-Bridge  
**Training Architecture:** Tabular Q-Learning with Discretized State Space ($3^4 = 81$ states)  
**Deployment Footprint:** 81 bytes Flash memory (`PROGMEM`), 0 bytes SRAM  

---

## 🧭 The 4-Direction Cardinal Architecture

```
                                  [ UP (Front Sensor) ]
                                            ▲
                                            │
               [ LEFT Sensor ] ◄────── [ CAR ] ──────► [ RIGHT Sensor ]
                                            │
                                            ▼
                                 [ DOWN (Back Sensor) ]

                                  4 Discrete Movements:
                         - ACTION_UP:     Drive Forward
                         - ACTION_DOWN:   Drive Backward / Reverse
                         - ACTION_LEFT:   Spin-Turn Left
                         - ACTION_RIGHT:  Spin-Turn Right
```

When the vehicle faces dead ends or corners where **UP (Front)** is blocked, it reads its **DOWN (Back)** sensor and reverses out safely!

---

## 🧠 State & Action Space Design

### 1. State Space ($|S| = 3^4 = 81$ states)
Each of the 4 ultrasonic sensors discretizes its distance into 3 categories:
- **`0 = BLOCKED`**: $< 15\text{ cm}$
- **`1 = NEAR`**: $15\text{ cm} \le d \le 35\text{ cm}$
- **`2 = CLEAR`**: $> 35\text{ cm}$

State encoding formula:
$$\text{state\_id} = (s_{up} \times 27) + (s_{down} \times 9) + (s_{left} \times 3) + s_{right}$$
where each $s_i \in \{0, 1, 2\}$, giving an integer $0 \le \text{state\_id} \le 80$.

### 2. Action Space ($|A| = 4$ movements)
| Action ID | Movement | Left Motor | Right Motor | Physical Action |
| :---: | :--- | :---: | :---: | :--- |
| `0` | `ACTION_UP` | Forward (+210) | Forward (+210) | Drive forward |
| `1` | `ACTION_DOWN` | Reverse (-180) | Reverse (-180) | Drive backward / reverse |
| `2` | `ACTION_LEFT` | Reverse (-160) | Forward (+160) | In-place spin left |
| `3` | `ACTION_RIGHT` | Forward (+160) | Reverse (-160) | In-place spin right |

---

## 🔌 Hardware Wiring & Pin Mapping

### 1. L298N Dual H-Bridge Motor Driver
> ⚠️ **IMPORTANT:** Remove the black jumper caps on `ENA` and `ENB` so the Arduino can output analog PWM signals.

| L298N Pin | Arduino Uno Pin | Description |
| :--- | :--- | :--- |
| **ENA** | **D5** (PWM) | Left motor speed control |
| **IN1** | **D7** | Left motor forward direction |
| **IN2** | **D8** | Left motor reverse direction |
| **ENB** | **D6** (PWM) | Right motor speed control |
| **IN3** | **D9** | Right motor forward direction |
| **IN4** | **D10** | Right motor reverse direction |
| **GND** | **GND** (Common) | **Must be tied to Arduino GND** |
| **12V / VMS** | **Battery + (7.4V - 11.1V)** | External battery pack power |

### 2. 4× HC-SR04 Ultrasonic Sensors
Sequential pinging (10ms pause between sensors) prevents acoustic cross-talk.

| Direction | Physical Mounting | TRIG Pin | ECHO Pin |
| :--- | :--- | :---: | :---: |
| **UP ($s_0$)** | Front Bumper (facing forward, $0^\circ$) | **D2** | **D12** |
| **DOWN ($s_1$)** | Rear Bumper (facing back, $180^\circ$) | **D3** | **D13** |
| **LEFT ($s_2$)** | Left Flank (facing left, $-90^\circ$) | **D4** | **A0** *(digital pin)* |
| **RIGHT ($s_3$)** | Right Flank (facing right, $+90^\circ$) | **D11** | **A1** *(digital pin)* |

*Connect all sensor `VCC` pins to 5V and all `GND` pins to common ground.*

---

## 🛡️ Independent Hardware Safety Reflex

The Arduino firmware [arduino.ino](file:///c:/Users/legen/Downloads/Reinforcement%20Learning%20for%20Arduino%20project/arduino/arduino.ino#L68-L86) enforces an **independent hardware reflex** before running policy inference:

```cpp
if (d_up < DIST_EMERGENCY_STOP_CM) { // Critical threshold = 10 cm
    Serial.println(F("[SAFETY OVERRIDE] Critical frontal proximity (<10cm)!"));
    emergencyBrake();
    if (d_down > DIST_BLOCKED_THRESH_CM) {
        setMotorSpeeds(-PWM_BACKWARD, -PWM_BACKWARD); // Reverse out
        delay(250);
    } else {
        // Rear blocked -> pivot spin away
        setMotorSpeeds((d_left > d_right) ? -PWM_SPIN : PWM_SPIN, 
                       (d_left > d_right) ?  PWM_SPIN : -PWM_SPIN);
        delay(300);
    }
    emergencyBrake();
    return;
}
```

---

## 🚀 Tonight's Fast Deployment Instructions

### 1. Re-Train or Customize (Windows PC)
Run the automated pipeline:
```bash
python run_all.py
```
- Trains for 2,500 episodes in ~40 seconds.
- Evaluates the policy across 100 test scenarios (achieves **72.0% collision-free survival**).
- Exports `policy.h` into both `deployment/` and `arduino/`.

### 2. Upload Firmware (Arduino IDE)
1. Open the Arduino IDE.
2. Open [arduino.ino](file:///c:/Users/legen/Downloads/Reinforcement%20Learning%20for%20Arduino%20project/arduino/arduino.ino).
3. Select Board: **Arduino Uno**, Port: your Arduino COM port.
4. Click **Upload** (Ctrl+U).

### 3. Bench Test (Wheels Raised Test)
1. Prop the car chassis up on a stand so wheels spin freely.
2. Open the **Serial Monitor** at **115200 baud**.
3. Verify the boot-up self-test:
   - Sensors print initial distance `[UP, DOWN, LEFT, RIGHT]`.
   - Motors pulse for 400ms to confirm H-bridge wiring.
4. Test with your hand:
   - Wave hand in front of **UP (Front)** $(<10\text{ cm})$: Observe instant emergency reverse!
   - Wave hand in front of **LEFT**: Observe `RIGHT` spin turn.
   - Wave hand in front of **RIGHT**: Observe `LEFT` spin turn.

### 4. Floor Trial
Set the car down in your room or obstacle course, switch on battery power, and watch it navigate!
