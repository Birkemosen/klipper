# Code for handling the kinematics of cartesian robots
#
# Copyright (C) 2016-2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import stepper, homing

class CartKinematics:
    def __init__(self, toolhead, config):
        self.printer = config.get_printer()
        # Setup axis rails
        self.rails = [stepper.LookupMultiRail(config.getsection('stepper_' + n))
                      for n in ['x', 'y', 'z']]
        for rail, axis in zip(self.rails, 'xyz'):
            rail.setup_itersolve('cartesian_stepper_alloc', axis)
        # Setup boundary checks
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat(
            'max_z_velocity', max_velocity, above=0., maxval=max_velocity)
        self.max_z_accel = config.getfloat(
            'max_z_accel', max_accel, above=0., maxval=max_accel)
        self.need_motor_enable = True
        self.limits = [(1.0, -1.0)] * 3
        # Setup stepper max halt velocity
        max_halt_velocity = toolhead.get_max_axis_halt()
        self.rails[0].set_max_jerk(max_halt_velocity, max_accel)
        self.rails[1].set_max_jerk(max_halt_velocity, max_accel)
        self.rails[2].set_max_jerk(
            min(max_halt_velocity, self.max_z_velocity), max_accel)
        # Check for dual carriage support
        self.dual_carriage_axis = None
        self.dual_carriage_rails = []
        if config.has_section('dual_carriage'):
            dc_config = config.getsection('dual_carriage')
            dc_axis = dc_config.getchoice('axis', {'x': 'x', 'y': 'y'})
            self.dual_carriage_axis = {'x': 0, 'y': 1}[dc_axis]
            dc_rail = stepper.LookupMultiRail(dc_config)
            dc_rail.setup_itersolve('cartesian_stepper_alloc', dc_axis)
            dc_rail.set_max_jerk(max_halt_velocity, max_accel)
            self.dual_carriage_rails = [
                self.rails[self.dual_carriage_axis], dc_rail]
            self.printer.lookup_object('gcode').register_command(
                'SET_DUAL_CARRIAGE', self.cmd_SET_DUAL_CARRIAGE,
                desc=self.cmd_SET_DUAL_CARRIAGE_help)
    def get_steppers(self, flags=""):
        if flags == "Z":
            return self.rails[2].get_steppers()
        return [s for rail in self.rails for s in rail.get_steppers()]
    def calc_position(self):
        return [rail.get_commanded_position() for rail in self.rails]
    def set_position(self, newpos, homing_axes):
        for i, rail in enumerate(self.rails):
            rail.set_position(newpos)
            if i in homing_axes:
                self.limits[i] = rail.get_range()
    def _home_axis(self, homing_state, axis, rail):
        # Determine moves
        position_min, position_max = rail.get_range()
        hi = rail.get_homing_info()
        if hi.positive_dir:
            pos = hi.position_endstop - 1.5*(hi.position_endstop - position_min)
            rpos = hi.position_endstop - hi.retract_dist
            r2pos = rpos - hi.retract_dist
        else:
            pos = hi.position_endstop + 1.5*(position_max - hi.position_endstop)
            rpos = hi.position_endstop + hi.retract_dist
            r2pos = rpos + hi.retract_dist
        # Initial homing
        homing_speed = hi.speed
        if axis == 2:
            homing_speed = min(homing_speed, self.max_z_velocity)
        homepos = [None, None, None, None]
        homepos[axis] = hi.position_endstop
        coord = [None, None, None, None]
        coord[axis] = pos
        homing_state.home(coord, homepos, rail.get_endstops(), homing_speed)
        # Retract
        coord[axis] = rpos
        homing_state.retract(coord, homing_speed)
        # Home again
        coord[axis] = r2pos
        homing_state.home(coord, homepos, rail.get_endstops(),
                          homing_speed/2.0, second_home=True)
        # Set final homed position
        coord[axis] = hi.position_endstop + rail.get_homed_offset()
        homing_state.set_homed_position(coord)
    def home(self, homing_state):
        # Each axis is homed independently and in order
        for axis in homing_state.get_axes():
            if axis == self.dual_carriage_axis:
                dc1, dc2 = self.dual_carriage_rails
                altc = self.rails[axis] == dc2
                self._activate_carriage(0)
                self._home_axis(homing_state, axis, dc1)
                self._activate_carriage(1)
                self._home_axis(homing_state, axis, dc2)
                self._activate_carriage(altc)
            else:
                self._home_axis(homing_state, axis, self.rails[axis])
    def motor_off(self, print_time):
        self.limits = [(1.0, -1.0)] * 3
        for rail in self.rails:
            rail.motor_enable(print_time, 0)
        for rail in self.dual_carriage_rails:
            rail.motor_enable(print_time, 0)
        self.need_motor_enable = True
    def _check_motor_enable(self, print_time, move):
        need_motor_enable = False
        for i, rail in enumerate(self.rails):
            if move.axes_d[i]:
                rail.motor_enable(print_time, 1)
            need_motor_enable |= not rail.is_motor_enabled()
        self.need_motor_enable = need_motor_enable
    def _check_endstops(self, move):
        end_pos = move.end_pos
        for i in (0, 1, 2):
            if (move.axes_d[i]
                and (end_pos[i] < self.limits[i][0]
                     or end_pos[i] > self.limits[i][1])):
                if self.limits[i][0] > self.limits[i][1]:
                    raise homing.EndstopMoveError(
                        end_pos, "Must home axis first")
                raise homing.EndstopMoveError(end_pos)
    def check_move(self, move):
        limits = self.limits
        xpos, ypos = move.end_pos[:2]
        if (xpos < limits[0][0] or xpos > limits[0][1]
            or ypos < limits[1][0] or ypos > limits[1][1]):
            self._check_endstops(move)
        if not move.axes_d[2]:
            # Normal XY move - use defaults
            return
        # Move with Z - update velocity and accel for slower Z axis
        self._check_endstops(move)
        z_ratio = move.move_d / abs(move.axes_d[2])
        move.limit_speed(
            self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio)
    def move(self, print_time, move):
        if self.need_motor_enable:
            self._check_motor_enable(print_time, move)
        for i, rail in enumerate(self.rails):
            if move.axes_d[i]:
                rail.step_itersolve(move.cmove)
    # Dual carriage support
    def _activate_carriage(self, carriage):
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.get_last_move_time()
        dc_rail = self.dual_carriage_rails[carriage]
        dc_axis = self.dual_carriage_axis
        self.rails[dc_axis] = dc_rail
        extruder_pos = toolhead.get_position()[3]
        toolhead.set_position(self.calc_position() + [extruder_pos])
        if self.limits[dc_axis][0] <= self.limits[dc_axis][1]:
            self.limits[dc_axis] = dc_rail.get_range()
        self.need_motor_enable = True
    cmd_SET_DUAL_CARRIAGE_help = "Set which carriage is active"
    def cmd_SET_DUAL_CARRIAGE(self, params):
        gcode = self.printer.lookup_object('gcode')
        carriage = gcode.get_int('CARRIAGE', params, minval=0, maxval=1)
        self._activate_carriage(carriage)
        gcode.reset_last_position()

def load_kinematics(toolhead, config):
    return CartKinematics(toolhead, config)
