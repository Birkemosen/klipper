Frequently asked questions
==========================

1. [How can I donate to the project?](#how-can-i-donate-to-the-project)
2. [How do I calculate the step_distance parameter in the printer config file?](#how-do-i-calculate-the-step_distance-parameter-in-the-printer-config-file)
3. [Where's my serial port?](#wheres-my-serial-port)
4. [When the micro-controller restarts the device changes to /dev/ttyUSB1](#when-the-micro-controller-restarts-the-device-changes-to-devttyusb1)
5. [The "make flash" command doesn't work](#the-make-flash-command-doesnt-work)
6. [How do I change the serial baud rate?](#how-do-i-change-the-serial-baud-rate)
7. [Can I run Klipper on something other than a Raspberry Pi 3?](#can-i-run-klipper-on-something-other-than-a-raspberry-pi-3)
8. [Why can't I move the stepper before homing the printer?](#why-cant-i-move-the-stepper-before-homing-the-printer)
9. [Why is the Z position_endstop set to 0.5 in the default configs?](#why-is-the-z-position_endstop-set-to-05-in-the-default-configs)
10. [I converted my config from Marlin and the X/Y axes work fine, but I just get a screeching noise when homing the Z axis](#i-converted-my-config-from-marlin-and-the-xy-axes-work-fine-but-i-just-get-a-screeching-noise-when-homing-the-z-axis)
11. [My TMC motor driver turns off in the middle of a print](#my-tmc-motor-driver-turns-off-in-the-middle-of-a-print)
12. [When I set "restart_method=command" my AVR device just hangs on a restart](#when-i-set-restart_methodcommand-my-avr-device-just-hangs-on-a-restart)
13. [Will the heaters be left on if the Raspberry Pi crashes?](#will-the-heaters-be-left-on-if-the-raspberry-pi-crashes)
14. [How do I upgrade to the latest software?](#how-do-i-upgrade-to-the-latest-software)

### How can I donate to the project?

Thanks. Kevin has a Patreon page at: https://www.patreon.com/koconnor

### How do I calculate the step_distance parameter in the printer config file?

If you know the steps per millimeter for the axis then use a
calculator to divide 1.0 by steps_per_mm. Then round this number to
six decimal places and place it in the config (six decimal places is
nano-meter precision).

The step_distance defines the distance that the axis will travel on
each motor driver pulse. It can also be calculated from the axis
pitch, motor step angle, and driver microstepping. If unsure, do a web
search for "calculate steps per mm" to find an online calculator.

### Where's my serial port?

The general way to find a USB serial port is to run `ls -l
/dev/serial/by-id/` from an ssh terminal on the host machine. It will
likely produce output similar to the following:
```
lrwxrwxrwx 1 root root 13 Jun  1 21:12 usb-1a86_USB2.0-Serial-if00-port0 -> ../../ttyUSB0
```

The name found in the above command is stable and it is possible to
use it in the config file and while flashing the micro-controller
code. For example, a flash command might look similar to:
```
sudo service klipper stop
make flash FLASH_DEVICE=/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0
sudo service klipper start
```
and the updated config might look like:
```
[mcu]
serial: /dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0
```

Be sure to copy-and-paste the name from the "ls" command that you ran
above as the name will be different for each printer.

If you are using multiple micro-controllers and they do not have
unique ids (common on boards with a CH340 USB chip) then follow the
directions above using the directory `/dev/serial/by-path/` instead.

### When the micro-controller restarts the device changes to /dev/ttyUSB1

Follow the directions in the
"[Where's my serial port?](#wheres-my-serial-port)" section to prevent
this from occurring.

### The "make flash" command doesn't work

The code attempts to flash the device using the most common method for
each platform. Unfortunately, there is a lot of variance in flashing
methods, so the "make flash" command may not work on all boards.

If you're having an intermittent failure or you do have a standard
setup, then double check that Klipper isn't running when flashing
(sudo service klipper stop), make sure OctoPrint isn't trying to
connect directly to the device (open the Connection tab in the web
page and click Disconnect if the Serial Port is set to the device),
and make sure FLASH_DEVICE is set correctly for your board (see the
[question above](#wheres-my-serial-port)).

However, if "make flash" just doesn't work for your board, then you
will need to manually flash. See if there is a config file in the
[config directory](../config) with specific instructions for flashing
the device. Also, check the board manufacturer's documentation to see
if it describes how to flash the device. Finally, on AVR devices, it
may be possible to manually flash the device using
[avrdude](http://www.nongnu.org/avrdude/) with custom command-line
parameters - see the avrdude documentation for further information.

### How do I change the serial baud rate?

The default baud rate is 250000 in both the Klipper micro-controller
configuration and in the Klipper host software. This works on almost
all micro-controllers and it is the recommended setting. (Most online
guides that refer to a baud rate of 115200 are outdated.)

If you need to change the baud rate, then the new rate will need to be
configured in the micro-controller (during **make menuconfig**) and
that updated code will need to be flashed to the micro-controller. The
Klipper printer.cfg file will also need to be updated to match that
baud rate (see the example.cfg file for details).  For example:
```
[mcu]
baud: 250000
```

The baud rate shown on the OctoPrint web page has no impact on the
internal Klipper micro-controller baud rate. Always set the OctoPrint
baud rate to 250000 when using Klipper.

### Can I run Klipper on something other than a Raspberry Pi 3?

The recommended hardware is a Raspberry Pi 2 or a Raspberry
Pi 3.

Klipper will run on a Raspberry Pi 1 and on the Raspberry Pi Zero, but
these boards don't have enough processing power to run OctoPrint
well. It's not uncommon for print stalls to occur on these slower
machines (the printer may move faster than OctoPrint can send movement
commands) when printing directly from OctoPrint. If you wish to run on
one one of these slower boards anyway, consider using the
"virtual_sdcard" feature (see
[config/example-extras.cfg](../config/example-extras.cfg) for details)
when printing.

For running on the Beaglebone, see the
[Beaglebone specific installation instructions](beaglebone.md).

Klipper has been run on other machines.  The Klipper host software
only requires Python running on a Linux (or similar)
computer. However, if you wish to run it on a different machine you
will need Linux admin knowledge to install the system prerequisites
for that particular machine. See the
[install-octopi.sh](../scripts/install-octopi.sh) script for further
information on the necessary Linux admin steps.

### Why can't I move the stepper before homing the printer?

The code does this to reduce the chance of accidentally commanding the
head into the bed or a wall. Once the printer is homed the software
attempts to verify each move is within the position_min/max defined in
the config file. If the motors are disabled (via an M84 or M18
command) then the motors will need to be homed again prior to
movement.

If you want to move the head after canceling a print via OctoPrint,
consider changing the OctoPrint cancel sequence to do that for
you. It's configured in OctoPrint via a web browser under:
Settings->GCODE Scripts

If you want to move the head after a print finishes, consider adding
the desired movement to the "custom g-code" section of your slicer.

If the printer requires some additional movement as part of the homing
process itself (or fundamentally does not have a homing process) then
consider using a homing_override section in the config file. If you
need to move a stepper for diagnostic or debugging purposes then
consider adding a force_move section to the config file. See
[example-extras.cfg](../config/example-extras.cfg) for further details
on these options.

### Why is the Z position_endstop set to 0.5 in the default configs?

For cartesian style printers the Z position_endstop specifies how far
the nozzle is from the bed when the endstop triggers. If possible, it
is recommended to use a Z-max endstop and home away from the bed (as
this reduces the potential for bed collisions). However, if one must
home towards the bed then it is recommended to position the endstop so
it triggers when the nozzle is still a small distance away from the
bed. This way, when homing the axis, it will stop before the nozzle
touches the bed.

Almost all mechanical switches can still move a small distance
(eg, 0.5mm) after they are triggered. So, for example, if the
position_endstop is set to 0.5mm then one may still command the
printer to move to Z0.2. The position_min config setting (which
defaults to 0) is used to specify the minimum Z position one may
command the printer to move to.

Note, the Z position_endstop specifies the distance from the nozzle to
the bed when the nozzle and bed (if applicable) are hot. It is typical
for thermal expansion to cause nozzle expansion of around .1mm, which
is also the typical thickness of a sheet of printer paper. Thus, it is
common to use the "paper test" to confirm calibration of the Z
height - check that the bed and nozzle are at room temperature, check
that there is no plastic on the head or bed, home the printer, place a
piece of paper between the nozzle and bed, and repeatedly command the
head to move closer to the bed checking each time if you feel a small
amount of friction when sliding the paper between bed and nozzle - if
all is calibrated well a small amount of friction would be felt when
the height is at Z0.

### I converted my config from Marlin and the X/Y axes work fine, but I just get a screeching noise when homing the Z axis

Short answer: Try reducing the max_z_velocity setting in the printer
config. Also, if the Z stepper is moving in the wrong direction, try
inverting the dir_pin setting in the config (eg, "dir_pin: !xyz"
instead of "dir_pin: xyz").

Long answer: In practice Marlin can typically only step at a rate of
around 10000 steps per second. If it is requested to move at a speed
that would require a higher step rate then Marlin will generally just
step as fast as it can. Klipper is able to achieve much higher step
rates, but the stepper motor may not have sufficient torque to move at
a higher speed. So, for a Z axis with a very precise step_distance the
actual obtainable max_z_velocity may be smaller than what is
configured in Marlin.

### My TMC motor driver turns off in the middle of a print

There have been reports of some TMC drivers being disabled in the
middle of a print. (In particular, with the TMC2208 driver.) When this
issue occurs, the stepper associated with the driver moves freely,
while the print continues.

It is believed this may be due to "over current" detection within the
TMC driver. Trinamic has indicated that this could occur if the driver
is in "stealthChop mode" and an abrupt velocity change occurs. If you
experience this problem during homing, consider using a slower homing
speed. If you experience this problem in the middle of a print,
consider using a lower square_corner_velocity setting.

### When I set "restart_method=command" my AVR device just hangs on a restart

Some old versions of the AVR bootloader have a known bug in watchdog
event handling. This typically manifests when the printer.cfg file has
restart_method set to "command". When the bug occurs, the AVR device
will be unresponsive until power is removed and reapplied to the
device (the power or status LEDs may also blink repeatedly until the
power is removed).

The workaround is to use a restart_method other than "command" or to
flash an updated bootloader to the AVR device. Flashing a new
bootloader is a one time step that typically requires an external
programmer - search the web to find the instructions for your
particular device.

### Will the heaters be left on if the Raspberry Pi crashes?

The software has been designed to prevent that. Once the host enables
a heater, the host software needs to confirm that enablement every 5
seconds. If the micro-controller does not receive a confirmation every
5 seconds it goes into a "shutdown" state which is designed to turn
off all heaters and stepper motors.

See the "config_digital_out" command in the
[MCU commands](MCU_Commands.md) document for further details.

In addition, the micro-controller software is configured with a
minimum and maximum temperature range for each heater at startup (see
the min_temp and max_temp parameters in the
[example.cfg](../config/example.cfg) file for details). If the
micro-controller detects that the temperature is outside of that range
then it will also enter a "shutdown" state.

Separately, the host software also implements code to check that
heaters and temperature sensors are functioning correctly. See the
"verify_heater" section of the
[example-extras.cfg](../config/example-extras.cfg) for further
details.

### How do I upgrade to the latest software?

The general way to upgrade is to ssh into the Raspberry Pi and run:

```
cd ~/klipper
git pull
~/klipper/scripts/install-octopi.sh
```

Then one can recompile and flash the micro-controller code. For
example:

```
make menuconfig
make clean
make

sudo service klipper stop
make flash FLASH_DEVICE=/dev/ttyACM0
sudo service klipper start
```

However, it's often the case that only the host software changes. In
this case, one can update and restart just the host software with:

```
cd ~/klipper
git pull
sudo service klipper restart
```

If after using this shortcut the software warns about needing to
reflash the micro-controller or some other unusual error occurs, then
follow the full upgrade steps outlined above. Note that the RESTART
and FIRMWARE_RESTART g-code commands do not load new software - the
above "sudo service klipper restart" and "make flash" commands are
needed for a software change to take effect.
