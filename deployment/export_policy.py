"""
Exports 81-State, 4-Direction Q-Learning Policy to C++ header (policy.h)
for Arduino Uno flash memory (PROGMEM).
Actions: ACTION_UP, ACTION_DOWN, ACTION_LEFT, ACTION_RIGHT
Sensors: UP (Front), DOWN (Back), LEFT, RIGHT
Total RAM footprint: 0 bytes.
Flash memory: 81 bytes.
"""

import os
import shutil
import numpy as np

ACTION_NAMES = [
    "ACTION_UP",
    "ACTION_DOWN",
    "ACTION_LEFT",
    "ACTION_RIGHT"
]

CATEGORY_NAMES = ["BLOCKED", "NEAR", "CLEAR"]

def export_policy_header(q_table_path: str = "results/q_table.npy",
                         output_header_path: str = "deployment/policy.h",
                         arduino_dest_path: str = "arduino/policy.h") -> str:
    if not os.path.exists(q_table_path):
        raise FileNotFoundError(f"Cannot find Q-table at {q_table_path}. Run training first.")

    q_table = np.load(q_table_path)
    assert q_table.shape == (81, 4), f"Unexpected Q-table shape: {q_table.shape}"

    policy = []
    for s in range(81):
        if np.all(np.isclose(q_table[s], 0.0)):
            s_up = s // 27
            s_down = (s % 27) // 9
            s_left = (s % 9) // 3
            s_right = s % 3

            if s_up == 2:
                action = 0  # UP
            elif s_left == 2 and s_left >= s_right:
                action = 2  # LEFT
            elif s_right == 2:
                action = 3  # RIGHT
            elif s_down == 2:
                action = 1  # DOWN
            else:
                action = 3  # Spin RIGHT
        else:
            action = int(np.argmax(q_table[s]))
        policy.append(action)

    header_lines = []
    header_lines.append("// =============================================================================")
    header_lines.append("// AUTONOMOUS RL CAR - 4-DIRECTION POLICY HEADER (AUTO-GENERATED)")
    header_lines.append("// Movements: UP (Forward), DOWN (Backward), LEFT, RIGHT")
    header_lines.append("// Sensors:   S0=UP (Front), S1=DOWN (Back), S2=LEFT, S3=RIGHT")
    header_lines.append("// Stored in PROGMEM flash memory (0 bytes of SRAM used)")
    header_lines.append("// =============================================================================\n")
    header_lines.append("#ifndef POLICY_H")
    header_lines.append("#define POLICY_H\n")
    header_lines.append("#include <Arduino.h>")
    header_lines.append("#include <avr/pgmspace.h>\n")

    header_lines.append("// 4 Movement Actions")
    header_lines.append("#define ACTION_UP     0  // Move UP (Forward)")
    header_lines.append("#define ACTION_DOWN   1  // Move DOWN (Backward / Reverse)")
    header_lines.append("#define ACTION_LEFT   2  // Move LEFT (Spin / Turn Left)")
    header_lines.append("#define ACTION_RIGHT  3  // Move RIGHT (Spin / Turn Right)\n")

    header_lines.append("// Sensor Distance Categories")
    header_lines.append("#define SENSOR_BLOCKED  0  // < 15 cm")
    header_lines.append("#define SENSOR_NEAR     1  // 15 - 35 cm")
    header_lines.append("#define SENSOR_CLEAR    2  // > 35 cm\n")

    header_lines.append("// Physical Sensor Thresholds (cm)")
    header_lines.append("#define DIST_BLOCKED_THRESH_CM  15")
    header_lines.append("#define DIST_NEAR_THRESH_CM     35")
    header_lines.append("#define DIST_EMERGENCY_STOP_CM  10\n")

    header_lines.append("// 81-Byte Policy Array in Flash Memory (PROGMEM)")
    header_lines.append("const uint8_t PROGMEM Q_POLICY[81] = {")

    for s in range(81):
        s_up = s // 27
        s_down = (s % 27) // 9
        s_left = (s % 9) // 3
        s_right = s % 3
        action = policy[s]
        act_name = ACTION_NAMES[action]

        comma = "," if s < 80 else " "
        comment = f"// State {s:2d}: UP={CATEGORY_NAMES[s_up]:<7} DOWN={CATEGORY_NAMES[s_down]:<7} LEFT={CATEGORY_NAMES[s_left]:<7} RIGHT={CATEGORY_NAMES[s_right]:<7} -> {act_name}"
        header_lines.append(f"    {action}{comma}  {comment}")

    header_lines.append("};\n")

    header_lines.append("// Distance Category Discretizer")
    header_lines.append("inline uint8_t discretize_distance(uint16_t dist_cm) {")
    header_lines.append("    if (dist_cm == 0 || dist_cm < DIST_BLOCKED_THRESH_CM) {")
    header_lines.append("        return SENSOR_BLOCKED;")
    header_lines.append("    } else if (dist_cm <= DIST_NEAR_THRESH_CM) {")
    header_lines.append("        return SENSOR_NEAR;")
    header_lines.append("    } else {")
    header_lines.append("        return SENSOR_CLEAR;")
    header_lines.append("    }")
    header_lines.append("}\n")

    header_lines.append("// State Index Encoder: (s_up * 27) + (s_down * 9) + (s_left * 3) + s_right")
    header_lines.append("inline uint8_t encode_state(uint8_t s_up, uint8_t s_down, uint8_t s_left, uint8_t s_right) {")
    header_lines.append("    return (s_up * 27) + (s_down * 9) + (s_left * 3) + s_right;")
    header_lines.append("}\n")

    header_lines.append("// Policy Lookup from Flash Memory")
    header_lines.append("inline uint8_t get_policy_action(uint8_t state_id) {")
    header_lines.append("    if (state_id >= 81) return ACTION_RIGHT; // Safety fallback")
    header_lines.append("    return pgm_read_byte(&(Q_POLICY[state_id]));")
    header_lines.append("}\n")

    header_lines.append("#endif // POLICY_H\n")

    content = "\n".join(header_lines)

    os.makedirs(os.path.dirname(output_header_path), exist_ok=True)
    with open(output_header_path, "w", encoding="utf-8") as f:
        f.write(content)

    os.makedirs(os.path.dirname(arduino_dest_path), exist_ok=True)
    shutil.copyfile(output_header_path, arduino_dest_path)
    print(f"Policy synced to: {output_header_path} & {arduino_dest_path}")

    return output_header_path

if __name__ == "__main__":
    export_policy_header()
