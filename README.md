# Autonomous RL Car: 4-Direction Simulation-to-Arduino Deployment

**Target Hardware:** Arduino Uno (ATmega328P, 16 MHz, 2 KB SRAM, 32 KB Flash)  
**Sensors:** 4x HC-SR04 Ultrasonic Distance Sensors (UP, DOWN, LEFT, RIGHT)  
**Movements:** 4 Discrete Actions (UP, DOWN, LEFT, RIGHT)  
**Motor Driver:** L298N Dual H-Bridge  
**Training Architecture:** Tabular Q-Learning with Discretized State Space (3^4 = 81 states)  
**Deployment Footprint:** 81 bytes Flash memory (PROGMEM), 0 bytes SRAM  
**Evaluated Performance:** 99.0% Collision-Free Survival Rate  

---

## 1. System Architecture

```
                                  [ UP (Front Sensor) ]
                                            ^
                                            |
               [ LEFT Sensor ] <------ [ CAR ] ------> [ RIGHT Sensor ]
                                            |
                                            v
                                 [ DOWN (Back Sensor) ]

                                  4 Discrete Movements:
                         - ACTION_UP:     Drive Forward
                         - ACTION_DOWN:   Drive Backward / Reverse
                         - ACTION_LEFT:   Spin-Turn Left
                         - ACTION_RIGHT:  Spin-Turn Right
```

When the vehicle faces dead ends or corners where UP (Front) is blocked, it reads its DOWN (Back) sensor and reverses out safely.

---

## 2. Complete Wiring Reference

### L298N Dual H-Bridge Driver
Important: Remove the black jumper caps on ENA and ENB so Arduino can output analog PWM signals.

| L298N Pin | Arduino Uno Pin | Function |
| :--- | :--- | :--- |
| **ENA** | **D5** (PWM) | Left motor speed control |
| **IN1** | **D7** | Left motor forward direction |
| **IN2** | **D8** | Left motor reverse direction |
| **ENB** | **D6** (PWM) | Right motor speed control |
| **IN3** | **D9** | Right motor forward direction |
| **IN4** | **D10** | Right motor reverse direction |
| **GND** | **GND** (Common) | Must connect to Arduino GND |
| **12V / VMS** | **Battery + (7.4V - 11.1V)** | External battery pack power |

### 4x HC-SR04 Ultrasonic Sensors
Sequential pinging (10ms pause between sensors) prevents acoustic cross-talk reflections.

| Direction | Physical Mounting | TRIG Pin | ECHO Pin |
| :--- | :--- | :---: | :---: |
| **UP (s_0)** | Front Bumper (facing forward, 0 deg) | **D2** | **D12** |
| **DOWN (s_1)** | Rear Bumper (facing back, 180 deg) | **D3** | **D13** |
| **LEFT (s_2)** | Left Flank (facing left, -90 deg) | **D4** | **A0** (digital pin) |
| **RIGHT (s_3)** | Right Flank (facing right, +90 deg) | **D11** | **A1** (digital pin) |

Connect all sensor VCC pins to 5V and all GND pins to common ground.

---

## 3. Tonight's Step-by-Step Execution Plan

### Step 1: Hardware Diagnostic Test (5 minutes)
Before running autonomous navigation, verify that all motors spin in the correct direction and all 4 sensors report valid distances:
1. Open `arduino/Hardware_Diagnostic/Hardware_Diagnostic.ino` in Arduino IDE.
2. Select Board: **Arduino Uno**, choose your COM port, and upload.
3. Prop the wheels off the ground and open Serial Monitor at **115200 baud**.
4. Observe the motor test sequence:
   - Left motor forward pulse
   - Right motor forward pulse
   - Both motors reverse pulse
5. Wave your hand in front of each sensor (UP, DOWN, LEFT, RIGHT) and check that distance values in centimeters change accordingly.

### Step 2: Flash the Autonomous RL Policy (2 minutes)
You have two options:
- **Option A (All-In-One single sketch):** Open `arduino/RL_Car_AllInOne/RL_Car_AllInOne.ino` and upload.
- **Option B (Modular sketch):** Open `arduino/arduino.ino` (alongside `motors.cpp`, `ultrasonic.cpp`, `policy.h`) and upload.

### Step 3: Bench Test with Telemetry (3 minutes)
1. Keep the wheels raised off the ground.
2. Open Serial Monitor at **115200 baud**.
3. Verify live telemetry:
   `[RL] Dist:[U:85, D:140, L:45, R:60] -> St:80 -> Act:UP (FORWARD)`
4. Put your hand within 10 cm of the UP sensor:
   Verify `[SAFETY OVERRIDE]` triggers and the motors reverse.

### Step 4: Floor Obstacle Course Trial (10 minutes)
1. Disconnect the USB cable.
2. Place the car in an obstacle course (boxes, chair legs, walls).
3. Turn on battery power.
4. The car will navigate autonomously, steering away from walls and reversing out of dead ends.

---

## 4. Troubleshooting Guide for Tonight

1. **A motor spins in reverse during forward test:**
   Swap that motor's two wires on the L298N green terminal block (e.g. OUT1/OUT2 or OUT3/OUT4).

2. **Ultrasonic sensor reads 0 or 200 constantly:**
   Check TRIG and ECHO wire positions. Ensure the sensor's VCC is connected to regulated 5V and GND is connected to common ground.

3. **Arduino resets when motors start:**
   Motors draw high inrush current. Power motors from an external battery pack into L298N 12V/GND, NOT from the Arduino 5V pin. Ensure battery GND connects to Arduino GND.

4. **Car drifts when driving forward:**
   DC toy motors often have slight friction differences. Adjust `PWM_FORWARD` or trim individual motor speeds in `motors.cpp` / `RL_Car_AllInOne.ino`.
