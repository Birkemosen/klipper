# Support for servos
#
# Copyright (C) 2017,2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

SERVO_SIGNAL_PERIOD = 0.020
PIN_MIN_TIME = 0.100

class PrinterServo:
    def __init__(self, config):
        self.printer = config.get_printer()
        ppins = self.printer.lookup_object('pins')
        self.mcu_servo = ppins.setup_pin('pwm', config.get('pin'))
        self.mcu_servo.setup_max_duration(0.)
        self.mcu_servo.setup_cycle_time(SERVO_SIGNAL_PERIOD)
        self.min_width = config.getfloat(
            'minimum_pulse_width', .001, above=0., below=SERVO_SIGNAL_PERIOD)
        self.max_width = config.getfloat(
            'maximum_pulse_width', .002
            , above=self.min_width, below=SERVO_SIGNAL_PERIOD)
        self.max_angle = config.getfloat('maximum_servo_angle', 180.)
        self.angle_to_width = (self.max_width - self.min_width) / self.max_angle
        self.width_to_value = 1. / SERVO_SIGNAL_PERIOD
        self.last_value = self.last_value_time = 0.
        servo_name = config.get_name().split()[1]
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_mux_command("SET_SERVO", "SERVO", servo_name,
                                        self.cmd_SET_SERVO,
                                        desc=self.cmd_SET_SERVO_help)
    def set_pwm(self, print_time, value):
        if value == self.last_value:
            return
        print_time = max(print_time, self.last_value_time + PIN_MIN_TIME)
        self.mcu_servo.set_pwm(print_time, value)
        self.last_value = value
        self.last_value_time = print_time
    def set_angle(self, print_time, angle):
        angle = max(0., min(self.max_angle, angle))
        width = self.min_width + angle * self.angle_to_width
        self.set_pwm(print_time, width * self.width_to_value)
    def set_pulse_width(self, print_time, width):
        width = max(self.min_width, min(self.max_width, width))
        self.set_pwm(print_time, width * self.width_to_value)
    cmd_SET_SERVO_help = "Set servo angle"
    def cmd_SET_SERVO(self, params):
        print_time = self.printer.lookup_object('toolhead').get_last_move_time()
        if 'WIDTH' in params:
            self.set_pulse_width(print_time,
                                 self.gcode.get_float('WIDTH', params))
        else:
            self.set_angle(print_time, self.gcode.get_float('ANGLE', params))

def load_config_prefix(config):
    return PrinterServo(config)
