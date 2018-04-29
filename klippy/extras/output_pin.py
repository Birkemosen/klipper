# Code to configure miscellaneous chips
#
# Copyright (C) 2017,2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

PIN_MIN_TIME = 0.100

class PrinterOutputPin:
    def __init__(self, config):
        self.printer = config.get_printer()
        ppins = self.printer.lookup_object('pins')
        self.is_pwm = config.getboolean('pwm', False)
        if self.is_pwm:
            self.mcu_pin = ppins.setup_pin('pwm', config.get('pin'))
            cycle_time = config.getfloat('cycle_time', 0.100, above=0.)
            hardware_pwm = config.getboolean('hardware_pwm', False)
            self.mcu_pin.setup_cycle_time(cycle_time, hardware_pwm)
            self.scale = config.getfloat('scale', 1., above=0.)
        else:
            self.mcu_pin = ppins.setup_pin('digital_out', config.get('pin'))
            self.scale = 1.
        self.mcu_pin.setup_max_duration(0.)
        self.last_value_time = 0.
        static_value = config.getfloat('static_value', None,
                                       minval=0., maxval=self.scale)
        if static_value is not None:
            self.is_static = True
            self.last_value = static_value / self.scale
            self.mcu_pin.setup_start_value(
                self.last_value, self.last_value, True)
        else:
            self.is_static = False
            self.last_value = config.getfloat(
                'value', 0., minval=0., maxval=self.scale) / self.scale
            shutdown_value = config.getfloat(
                'shutdown_value', 0., minval=0., maxval=self.scale) / self.scale
            self.mcu_pin.setup_start_value(self.last_value, shutdown_value)
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command("SET_PIN", self.cmd_SET_PIN,
                                    desc=self.cmd_SET_PIN_help)
    cmd_SET_PIN_help = "Set the value of an output pin"
    def cmd_SET_PIN(self, params):
        pin_name = self.gcode.get_str('PIN', params)
        pin = self.printer.lookup_object('output_pin ' + pin_name, None)
        if pin is not self:
            if pin is None:
                raise self.gcode.error("Pin not configured")
            return pin.cmd_SET_PIN(params)
        if self.is_static:
            raise self.gcode.error("Static pin can not be changed at run-time")
        value = self.gcode.get_float('VALUE', params, minval=0., maxval=1.)
        value /= self.scale
        if value == self.last_value:
            return
        print_time = self.printer.lookup_object('toolhead').get_last_move_time()
        print_time = max(print_time, self.last_value_time + PIN_MIN_TIME)
        if self.is_pwm:
            self.mcu_pin.set_pwm(print_time, value)
        else:
            if value not in [0., 1.]:
                raise self.gcode.error("Invalid pin value")
            self.mcu_pin.set_digital(print_time, value)
        self.last_value = value
        self.last_value_time = print_time

def load_config_prefix(config):
    return PrinterOutputPin(config)
