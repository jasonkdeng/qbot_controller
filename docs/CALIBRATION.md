# Calibrating one leg

`src/calibration.cpp` is separate firmware controlled through Serial at 115200
baud. The normal firmware still defaults to static stance. Calibration never
edits `RobotConfig.h`: measurements live in RAM until you export and copy them.
The existing front-right calibration remains unchanged.

## Build and connect

The controller is a classic ESP32-WROOM. Both environments use PlatformIO's
`esp32dev` board target; do not select an ESP32-S3 target.

```sh
pio run -e leg_calibration -t upload
pio device monitor -e leg_calibration
```

Send one command per line. Type `help` if the startup message was missed.
No PWM is attached at startup. Support the body and leg; only one joint is
powered by this tool at a time. Use external servo power with a common ground.
`off` detaches PWM; it does not physically disconnect servo power.

Select the actual wired leg (`FR`, `FL`, `RR`, or `RL`), then a joint:

```text
leg FL
joint hip
```

Set its actual GPIO with `pin <GPIO>`. The tool rejects invalid output pins and
duplicate pins within the selected leg. You must also check that the chosen pin
is available on your board and is not used by flash, PSRAM, USB or another device.
Existing pins are loaded from `RobotConfig.h`; unassigned pins cannot be armed.

### Proposed PWM wiring for ESP32-WROOM

Use GPIO labels, not physical header positions. Front-right retains its existing
wiring. The other legs still have unassigned pins in `RobotConfig.h`; enter the
following pins during calibration and save them with the exported measurements.

| Leg | Hip | Thigh | Knee |
| --- | --- | --- | --- |
| Front-right (existing) | 18 | 19 | 21 |
| Front-left | 25 | 26 | 27 |
| Rear-right | 32 | 33 | 23 |
| Rear-left | 13 | 14 | 22 |

This allocation assumes these GPIOs are exposed and unused by other peripherals.
It replaces the earlier ESP32-S3 wiring suggestion. On classic ESP32-WROOM,
GPIO6-11 are used for flash and GPIO34-39 are input-only. Connect servo signal
wires to the table's GPIOs, servo power to an external regulated supply, and
connect servo supply ground to ESP32 GND. Do not power servos from the ESP32's
USB supply or 3.3 V pin.

Reference: [Espressif ESP32 GPIO documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/gpio.html).

## Repeat for hip, thigh and knee

1. **Establish a starting position.** `arm <raw-angle>` attaches the selected
   servo and immediately commands that angle in the 0–180 degree range. Choose
   a known unobstructed starting position. If the installation is unknown,
   initially position the servo with the horn/linkage disconnected, then power
   down before installing the horn. Arming can cause a large movement.
2. **Find mathematical neutral.** Use `step 1`, `step -1`, or fractional steps
   such as `step 0.5`. Each step is limited to five raw servo degrees. Align:
   - Hip: the hip-to-thigh offset points straight downward.
   - Thigh: the thigh link points downward in the leg's pitch plane.
   - Knee: shin is straight in line with the thigh, extending away from the hip.
   Send `neutral` to record the current raw angle as mathematical zero.
3. **Determine direction.** Observe a small positive raw step from neutral.
   Positive mathematical hip roll moves the leg outward (+Y); positive thigh
   pitch rotates the thigh toward the front (+X); positive knee bend rotates
   the shin toward +X relative to the straight thigh. Set `direction 1` if
   increasing the raw angle produces that motion, otherwise `direction -1`.
   These are local conventions for every leg; do not mirror the IK equations.
4. **Measure safe limits.** Move in small steps toward each end of usable
   travel, stopping short of binding, collision or strain. Send `min` at the
   lowest mathematical angle and `max` at the highest. For a knee using the
   existing IK branch, mark `min` at straight (zero), then measure positive bend.
   Raw steps intentionally allow exploration beyond previously loaded limits;
   the tool cannot sense a mechanical stop. Never use the full 0–180 range as
   a substitute for measuring safe mechanical travel.
5. **Review.** `status` displays the calibration and which measurements are
   recorded. `zero` reports the saved raw neutral; approach it with `step`
   commands. It does not trigger a sudden return move. Send `off`, then select
   the next joint with `joint thigh` or `joint knee`.

Changing neutral or direction invalidates recorded limits. Changing the pin
invalidates that joint's measurements. Selecting another leg, even the same
leg again, reloads config values and clears all session measurements.

## Save and verify

After all three joints have a neutral, direction and two limits, send `export`.
The tool validates that the limits include zero, span a nonzero range, and map
to raw angles within 0–180 degrees. Copy the emitted `LegConfig` block into
`include/RobotConfig.h`, replacing only the corresponding leg entry.
The exported leg stays disabled until you deliberately enable it.

Resetting, uploading, or removing power loses unsaved measurements. This tool
does not automatically detect neutral, measure position, or store values in NVS.

Before enabling the new leg for stance, check that its calculated stance angles
fit the measured limits. Then test that leg with the robot supported and verify
forward/outward/downward motion using the shared IK. Measured calibration alone
does not establish four-leg stability.

Restore the normal firmware explicitly:

```sh
pio run -e esp32_wroom -t upload
```
