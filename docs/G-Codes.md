This document describes the commands that Klipper supports. These are
commands that one may enter into the OctoPrint terminal tab.

# G-Code commands

Klipper supports the following standard G-Code commands:
- Move (G0 or G1): `G1 [X<pos>] [Y<pos>] [Z<pos>] [E<pos>] [F<speed>]`
- Dwell: `G4 P<milliseconds>`
- Move to origin: `G28 [X] [Y] [Z]`
- Turn off motors: `M18` or `M84`
- Wait for current moves to finish: `M400`
- Select tool: `T<index>`
- Use absolute/relative distances for extrusion: `M82`, `M83`
- Use absolute/relative coordinates: `G90`, `G91`
- Set position: `G92 [X<pos>] [Y<pos>] [Z<pos>] [E<pos>]`
- Set speed factor override percentage: `M220 S<percent>`
- Set extrude factor override percentage: `M221 S<percent>`
- Get extruder temperature: `M105`
- Set extruder temperature: `M104 [T<index>] [S<temperature>]`
- Set extruder temperature and wait: `M109 [T<index>] S<temperature>`
- Set bed temperature: `M140 [S<temperature>]`
- Set bed temperature and wait: `M190 S<temperature>`
- Set fan speed: `M106 S<value>`
- Turn fan off: `M107`
- Emergency stop: `M112`
- Get current position: `M114`
- Get firmware version: `M115`

For further details on the above commands see the
[RepRap G-Code documentation](http://reprap.org/wiki/G-code).

Klipper's goal is to support the G-Code commands produced by common
3rd party software (eg, OctoPrint, Printrun, Slic3r, Cura, etc.) in
their standard configurations. It is not a goal to support every
possible G-Code command. Instead, Klipper prefers human readable
["extended G-Code commands"](#extended-g-code-commands).

## G-Code SD card commands

Klipper also supports the following standard G-Code commands if the
"virtual_sdcard" config section is enabled:
- List SD card: `M20`
- Initialize SD card: `M21`
- Select SD file: `M23 <filename>`
- Start/resume SD print: `M24`
- Pause SD print: `M25`
- Set SD position: `M26 S<offset>`
- Report SD print status: `M27`

# Extended G-Code Commands

Klipper uses "extended" G-Code commands for general configuration and
status.  These extended commands all follow a similar format - they
start with a command name and may be followed by one or more
parameters. For example: `SET_SERVO SERVO=myservo ANGLE=5.3`. In this
document, the commands and parameters are shown in uppercase, however
they are not case sensitive. (So, "SET_SERVO" and "set_servo" both run
the same command.)

The following standard commands are supported:
- `QUERY_ENDSTOPS`: Probe the axis endstops and report if they are
  "triggered" or in an "open" state. This command is typically used to
  verify that an endstop is working correctly.
- `GET_POSITION`: Return information on the current location of the
  toolhead.
- `SET_GCODE_OFFSET [X=<pos>|X_ADJUST=<adjust>]
  [Y=<pos>|Y_ADJUST=<adjust>] [Z=<pos>|Z_ADJUST=<adjust>]`: Set a
  positional offset to apply to future G-Code commands. This is
  commonly used to virtually change the Z bed offset or to set nozzle
  XY offsets when switching extruders. For example, if
  "SET_GCODE_OFFSET Z=0.2" is sent, then future G-Code moves will
  have 0.2mm added to their Z height. If the X_ADJUST style parameters
  are used, then the adjustment will be added to any existing offset
  (eg, "SET_GCODE_OFFSET Z=-0.2" followed by "SET_GCODE_OFFSET
  Z_ADJUST=0.3" would result in a total Z offset of 0.1).
- `PID_CALIBRATE HEATER=<config_name> TARGET=<temperature>
  [WRITE_FILE=1]`: Perform a PID calibration test. The specified
  heater will be enabled until the specified target temperature is
  reached, and then the heater will be turned off and on for several
  cycles. If the WRITE_FILE parameter is enabled, then the file
  /tmp/heattest.txt will be created with a log of all temperature
  samples taken during the test.
- `SET_VELOCITY_LIMIT [VELOCITY=<value>] [ACCEL=<value>]
  [ACCEL_TO_DECEL=<value>] [JUNCTION_DEVIATION=<value>]`: Modify the
  printer's velocity limits. Note that one may only set values less
  than or equal to the limits specified in the config file.
- `SET_PRESSURE_ADVANCE [ADVANCE=<pressure_advance>]
  [ADVANCE_LOOKAHEAD_TIME=<pressure_advance_lookahead_time>]`:
  Set pressure advance parameters.
- `RESTART`: This will cause the host software to reload its config
  and perform an internal reset. This command will not clear error
  state from the micro-controller (see FIRMWARE_RESTART) nor will it
  load new software (see
  [the FAQ](FAQ.md#how-do-i-upgrade-to-the-latest-software)).
- `FIRMWARE_RESTART`: This is similar to a RESTART command, but it
  also clears any error state from the micro-controller.
- `STATUS`: Report the Klipper host software status.
- `HELP`: Report the list of available extended G-Code commands.

## Custom Pin Commands

The following command is available when an "output_pin" config section
is enabled:
- `SET_PIN PIN=config_name VALUE=<value>`

## Servo Commands

The following commands are available when a "servo" config section is
enabled:
- `SET_SERVO SERVO=config_name WIDTH=<seconds>`
- `SET_SERVO SERVO=config_name ANGLE=<degrees>`

## Probe

The following commands are available when a "probe" config section is
enabled:
- `PROBE`: Move the nozzle downwards until the probe triggers.
- `QUERY_PROBE`: Report the current status of the probe ("triggered"
  or "open").

## Delta Calibration

The following commands are available when the "delta_calibrate" config
section is enabled:
- `DELTA_CALIBRATE`: This command will probe seven points on the bed
  and recommend updated endstop positions, tower angles, and radius.
  - `NEXT`: If manual bed probing is enabled, then one can use this
    command to move to the next probing point during a DELTA_CALIBRATE
    operation.

## Bed Tilt

The following commands are available when the "bed_tilt" config
section is enabled:
- `BED_TILT_CALIBRATE`: This command will probe the points specified
  in the config and then recommend updated x and y tilt adjustments.
  - `NEXT`: If manual bed probing is enabled, then one can use this
    command to move to the next probing point during a
    BED_TILT_CALIBRATE operation.

## Dual Carriages

The following command is available when the "dual_carriage" config
section is enabled:
- `SET_DUAL_CARRIAGE CARRIAGE=[0|1]`: This command will set the active
  carriage. It is typically invoked from the activate_gcode and
  deactivate_gcode fields in a multiple extruder configuration.
