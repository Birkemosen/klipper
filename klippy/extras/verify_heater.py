# Heater/sensor verification code
#
# Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

HINT_THERMAL = """
See the 'verify_heater' section in config/example-extras.cfg
for the parameters that control this check.
"""

class HeaterCheck:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.heater_name = config.get_name().split()[1]
        self.heater = None
        self.hysteresis = config.getfloat('hysteresis', 5., minval=0.)
        self.max_error = config.getfloat('max_error', 120., minval=0.)
        self.heating_gain = config.getfloat('heating_gain', 2., above=0.)
        default_gain_time = 20.
        if self.heater_name == 'heater_bed':
            default_gain_time = 60.
        self.check_gain_time = config.getfloat(
            'check_gain_time', default_gain_time, minval=1.)
        self.met_target = False
        self.last_target = self.goal_temp = self.error = 0.
        self.fault_systime = self.printer.get_reactor().NEVER
    def printer_state(self, state):
        if state == 'connect':
            if self.printer.get_start_args().get('debugoutput') is not None:
                # Disable verify_heater if outputting to a debug file
                return
            pheater = self.printer.lookup_object('heater')
            self.heater = pheater.lookup_heater(self.heater_name)
            logging.info("Starting heater checks for %s", self.heater_name)
            reactor = self.printer.get_reactor()
            reactor.register_timer(self.check_event, reactor.NOW)
    def check_event(self, eventtime):
        temp, target = self.heater.get_temp(eventtime)
        if temp >= target - self.hysteresis:
            # Temperature near target - reset checks
            if not self.met_target and target:
                logging.info("Heater %s within range of %.3f",
                             self.heater_name, target)
            self.met_target = True
            self.error = 0.
        elif self.met_target:
            self.error += (target - self.hysteresis) - temp
            if target != self.last_target:
                # Target changed - reset checks
                logging.info("Heater %s approaching new target of %.3f",
                             self.heater_name, target)
                self.met_target = False
                self.goal_temp = temp + self.heating_gain
                self.fault_systime = eventtime + self.check_gain_time
            elif self.error >= self.max_error:
                # Failure due to inability to maintain target temperature
                return self.heater_fault()
        elif temp >= self.goal_temp:
            # Temperature approaching target - reset checks
            self.goal_temp = temp + self.heating_gain
            self.fault_systime = eventtime + self.check_gain_time
        elif eventtime >= self.fault_systime:
            # Failure due to inability to approach target temperature
            return self.heater_fault()
        self.last_target = target
        return eventtime + 1.
    def heater_fault(self):
        msg = "Heater %s not heating at expected rate" % (self.heater_name,)
        logging.error(msg)
        self.printer.invoke_shutdown(msg + HINT_THERMAL)
        return self.printer.get_reactor().NEVER

def load_config_prefix(config):
    return HeaterCheck(config)
