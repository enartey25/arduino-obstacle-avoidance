"""
Exports 81-State, 7-Smooth-Action Q-Learning Policy to C++ header (policy.h)
for Arduino Uno flash memory (PROGMEM).
Total RAM footprint: 0 bytes.
Flash memory: 81 bytes.
"""

import os
import shutil
import numpy as np

ACTION_NAMES = [
    "ACTION_FORWARD_FAST",
    "ACTION_FORWARD_SLOW",
    "ACTION_CURVE_LEFT",
    "ACTION_CURVE_RIGHT",
    "ACTION_SPIN_LEFT",
    "ACTION_SPIN_RIGHT",
    "ACTION_REVERSE"
]

CATEGORY_NAMES = ["BLOCKED", "NEAR", "CLEAR"]

def export_policy_header(q_table_path: str = "results/q_table.npy",
                         output_header_path: str = "deployment/policy.h",
                         arduino_dest_path: str = "arduino/policy.h") -> str:
    if not os.path.exists(q_table_path):
        raise FileNotFoundError(f"Cannot find Q-table at {q_table_path}. Run training first.")

    q_table = np.load(q_table_path)
    assert q_table.shape == (81, 7), f"Unexpected Q-table shape: {q_table.shape}"

    policy = []
    for s in range(81):
        if np.all(np.isclose(q_table[s], 0.0)):
            s_up = s // 27
            s_down = (s % 27) // 9
            s_left = (s % 9) // 3
            s_right = s % 3

            if s_up == 2:
                action = 0  # FORWARD_FAST
            elif s_up == 1:
                if s_left > s_right and s_left == 2:
                    action = 2  # CURVE_LEFT
                elif s_right > s_left and s_right == 2:
                    action = 3  # CURVE_RIGHT
                else:
                    action = 1  # FORWARD_SLOW
            else:
                if s_left > s_right:
                    action = 4  # SPIN_LEFT
                elif s_right > s_left:
                    action = 5  # SPIN_RIGHT
                elif s_down == 2:
                    action = 6  # REVERSE
                else:
                    action = 5  # SPIN_RIGHT
        else:
            action = int(np.argmax(q_table[s]))
        policy.append(action)

    header_lines = []
    header_lines.append("// =============================================================================")
    header_lines.append("// AUTONOMOUS RL CAR - 7-SMOOTH-ACTION POLICY HEADER (AUTO-GENERATED)")
    header_lines.append("// Continuous-like smooth motion with variable motor speeds")
    header_lines.append("// Stored in PROGMEM flash memory (0 bytes of SRAM used)")
    header_lines.append("// =============================================================================\n")
    header_lines.append("#ifndef POLICY_H")
    header_lines.append("#define POLICY_H\n")
    header_lines.append("#include <Arduino.h>")
    header_lines.append("#include <avr/pgmspace.h>\n")

    header_lines.append("// 7 Smooth Actions")
    header_lines.append("#define ACTION_FORWARD_FAST 0  // Fast forward sprint in open space")
    header_lines.append("#define ACTION_FORWARD_SLOW 1  // Cautious forward glide near obstacles")
    header_lines.append("#define ACTION_CURVE_LEFT   2  // Smooth sweeping left arc (continuous turn)")
    header_lines.append("#define ACTION_CURVE_RIGHT  3  // Smooth sweeping right arc (continuous turn)")
    header_lines.append("#define ACTION_SPIN_LEFT    4  // In-place pivot left (for tight obstacles)")
    header_lines.append("#define ACTION_SPIN_RIGHT   5  // In-place pivot right (for tight obstacles)")
    header_lines.append("#define ACTION_REVERSE      6  // Smooth reverse\n")

    header_lines.append("// Sensor Distance Categories")
    header_lines.append("#define SENSOR_BLOCKED  0  // < 15 cm")
    header_lines.append("#define SENSOR_NEAR     1  // 15 - 35 cm")
    header_lines.append("#define SENSOR_CLEAR    2  // > 35 cm\n")

    header_lines.append("// Physical Sensor Thresholds (cm)")
    header_lines.append("#define DIST_BLOCKED_THRESH_CM  18")
    header_lines.append("#define DIST_NEAR_THRESH_CM     40")
    header_lines.append("#define DIST_EMERGENCY_STOP_CM  12\n")

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
    header_lines.append("    if (state_id >= 81) return ACTION_SPIN_RIGHT; // Safety fallback")
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
