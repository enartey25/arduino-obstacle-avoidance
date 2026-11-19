# Autonomous RL Car: Continuous-Like Simulation-to-Arduino Deployment

**Target Hardware:** Arduino Uno (ATmega328P, 16 MHz, 2 KB SRAM, 32 KB Flash)  
**Sensors:** 4x HC-SR04 Ultrasonic Distance Sensors (UP, DOWN, LEFT, RIGHT)  
**Movements:** 7 Smooth Continuous-Like Actions with Variable Speeds & Slew Ramping  
**Motor Driver:** L298N Dual H-Bridge  
**Training Architecture:** Tabular Q-Learning (81 states, 7 actions)  
**Deployment Footprint:** 81 bytes Flash memory (PROGMEM), 0 bytes SRAM  

---

## 1. Continuous-Like Motion Architecture

```
                                  [ UP (Front Sensor) ]
                                            ^
                                            |
               [ LEFT Sensor ] <------ [ CAR ] ------> [ RIGHT Sensor ]
                                            |
                                            v
                                 [ DOWN (Back Sensor) ]

                    7 Actions with Variable Motor Speeds:
       - ACTION_FORWARD_FAST: High-speed forward cruise (PWM ~225)
       - ACTION_FORWARD_SLOW: Cautious forward glide (PWM ~135)
       - ACTION_CURVE_LEFT:   Sweeping left arc (Outer: 210, Inner: 105)
       - ACTION_CURVE_RIGHT:  Sweeping right arc (Outer: 210, Inner: 105)
       - ACTION_SPIN_LEFT:    Tight pivot left (for close obstacles)
       - ACTION_SPIN_RIGHT:   Tight pivot right (for close obstacles)
       - ACTION_REVERSE:      Smooth backing out of dead ends
```

### Motion Smoothing Features
1. **Exponential Slew-Rate Filter:** Motor PWM smoothly ramps up and down every 10ms (`SLEW_RATE_COEFF = 0.25f`), eliminating jerky starts, stops, and wheel skid.
2. **Proportional Distance Speed Modulation:** In open space (> 70 cm), speeds boost up to 110%; as obstacles approach (18-40 cm), throttle automatically eases down to 85% before steering.
3. **Differential Curving Arcs:** Instead of always hard pivoting, the car takes sweeping curves (`CURVE_LEFT` / `CURVE_RIGHT`) where both wheels drive forward at differential speeds.

---

## 2. Hardware Wiring and Pin Mapping

### L298N Dual H-Bridge Motor Driver
Note: Remove the black jumper caps on ENA and ENB so Arduino can output analog PWM signals.

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
Sequential pinging (10ms pause between sensors) prevents acoustic cross-talk.

| Direction | Physical Mounting | TRIG Pin | ECHO Pin |
| :--- | :--- | :---: | :---: |
| **UP (s_0)** | Front Bumper (facing forward, 0 deg) | **D2** | **D12** |
| **DOWN (s_1)** | Rear Bumper (facing back, 180 deg) | **D3** | **D13** |
| **LEFT (s_2)** | Left Flank (facing left, -90 deg) | **D4** | **A0** (digital pin) |
| **RIGHT (s_3)** | Right Flank (facing right, +90 deg) | **D11** | **A1** (digital pin) |

Connect all sensor VCC pins to 5V and all GND pins to common ground.

---

## 3. Tonight's Step-by-Step Execution Plan

### Step 1: Run Hardware Diagnostic Test (5 minutes)
Verify motor polarities and sensor distances before running autonomous navigation:
1. Open `arduino/Hardware_Diagnostic/Hardware_Diagnostic.ino` in Arduino IDE.
2. Select Board: **Arduino Uno**, select COM port, and click **Upload**.
3. Prop wheels off the ground and open Serial Monitor at **115200 baud**.
4. Observe the motor test sequence:
   - Left motor forward pulse
   - Right motor forward pulse
   - Both motors reverse pulse
   *(If a wheel spins backward during the forward pulse, swap that motor's two wires on the L298N terminal block).*
5. Wave your hand in front of each sensor (UP, DOWN, LEFT, RIGHT) and confirm live distance numbers change correctly.

### Step 2: Flash the Autonomous RL Policy (2 minutes)
Choose either:
- **Option A (All-in-one single file):** Open `arduino/RL_Car_AllInOne/RL_Car_AllInOne.ino` in Arduino IDE and click **Upload**.
- **Option B (Modular project):** Open `arduino/arduino.ino` in Arduino IDE and click **Upload**.

### Step 3: Bench Test with Telemetry (3 minutes)
1. Keep the wheels raised off the ground.
2. Open Serial Monitor at **115200 baud**.
3. Confirm the live telemetry stream showing smooth actions:
   `[RL] Dist:[U:85, D:140, L:45, R:60] -> St:80 -> Act:FORWARD_FAST`
4. Put your hand within 12 cm of the UP sensor:
   Verify `[SAFETY OVERRIDE]` triggers immediately.

### Step 4: Floor Trial (10 minutes)
1. Disconnect the USB cable.
2. Place the car in your room or obstacle course.
3. Turn on external battery power.
4. The car will navigate smoothly, sweeping around obstacles and reversing out of dead ends.
