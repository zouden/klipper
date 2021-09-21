# Five Bar linkage kinematics
 
# Pontus Borg <glpontus@gmail.com>
# Eirinn Mackay <eirinn.mackay@gmail.com>

# This file may be distributed under the terms of the GNU GPLv3 license.
#
# TODO:
#   Add support for xoffset, yoffset and flipping of x and y
#   Check other angle limits? such as head angle and angles by the body
#


import math, logging
import stepper, mathutil, chelper

class FiveBar:
    def __init__(self, toolhead, config):

        stepper_configs = [config.getsection('stepper_left'),
                           config.getsection('stepper_right')]

        rail_arm_left = stepper.PrinterRail(stepper_configs[0],
                                            units_in_radians=True)
        rail_arm_right = stepper.PrinterRail(stepper_configs[1],
                                             units_in_radians=True)
        rail_arm_left.safe_home_angle = stepper_configs[0].getfloat('safe_home_angle')
        rail_arm_right.safe_home_angle = stepper_configs[1].getfloat('safe_home_angle')
        rail_z = stepper.LookupMultiRail(config.getsection('stepper_z'))
        rail_z.setup_itersolve('cartesian_stepper_alloc', 'z')

        self.inner_distance = config.getfloat('inner_distance', above=0.)

        self.left_inner_arm = stepper_configs[0].getfloat('inner_arm_length', above=0.)
        self.left_outer_arm = stepper_configs[0].getfloat('outer_arm_length', above=0.)
        rail_arm_left.setup_itersolve('fivebar_stepper_alloc', 'l',
                                      self.left_inner_arm, self.left_outer_arm,
                                      self.inner_distance)

        self.right_inner_arm = stepper_configs[1].getfloat('inner_arm_length', above=0.)
        self.right_outer_arm = stepper_configs[1].getfloat('outer_arm_length', above=0.)
        rail_arm_right.setup_itersolve(
            'fivebar_stepper_alloc', 'r',
             self.right_inner_arm, self.right_outer_arm,
             self.inner_distance)

        self.rails = [rail_arm_left, rail_arm_right, rail_z]

        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)
        config.get_printer().register_event_handler(
            "stepper_enable:motor_off",  self._motor_off)

        # Setup boundary checks
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat(
            'max_z_velocity', max_velocity, above=0., maxval=max_velocity)
        self.max_z_accel = config.getfloat(
            'max_z_accel', max_accel, above=0., maxval=max_accel)
        self.limit_z = (1.0, -1.0)
        self.homedXY = False
        self.ishoming = False

        # Homing trickery fake cartesial kinematic
        self.printer = config.get_printer()
        ffi_main, ffi_lib = chelper.get_ffi()
        self.cartesian_kinematics_L = ffi_main.gc(
            ffi_lib.cartesian_stepper_alloc('x'), ffi_lib.free)
        self.cartesian_kinematics_R = ffi_main.gc(
            ffi_lib.cartesian_stepper_alloc('y'), ffi_lib.free)

        logging.info("Fivebar setup %.2f %.2f %.2f %.2f %.2f",
                     self.left_inner_arm, self.left_outer_arm,
                     self.right_inner_arm, self.right_outer_arm,
                     self.inner_distance)

    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]

    def get_arm_steppers(self):
        return [s for rail in self.rails[:2] for s in rail.get_steppers()]
   
    # Geometry functions
    # def _distance(self, x0, y0, x1, y1):
    #     dx = x1-x0
    #     dy = y1-y0
    #     return math.sqrt(dx*dx + dy*dy)

    def _calculate_line(self, x0, y0, theta, length):
        # Find the endpoints of a line given its length and angle
        x1 = x0 + math.cos(theta) * length
        y1 = y0 + math.sin(theta) * length
        return x1, y1

    # def _triangle_side(self, a, b, C):
    #     # Finds the side of a triangle given 2 sides and the angle between them
    #     return math.sqrt((a*a) + (b*b) - (2*a*b*math.cos(C)))

    # def _triangle_angle(self, a, b, c):
    #     # Find the angle of the corner opposite to side c
    #     # (cosine rule)
    #     cosC = (a*a + b*b - c*c) / ( 2*a*b)
    #     return math.acos(cosC)

    def _get_intersections(self, circle0, r0, circle1, r1):
        # https://stackoverflow.com/questions/55816902/finding-the-intersection-of-two-circles
        # circle 1: (x0, y0), radius r0
        # circle 2: (x1, y1), radius r1
        x0, y0 = circle0
        x1, y1 = circle1

        d=math.sqrt((x1-x0)**2 + (y1-y0)**2)
        # non intersecting
        if d > r0 + r1 :
            return None
        # One circle within other
        if d < abs(r0-r1):
            return None
        # coincident circles
        if d == 0 and r0 == r1:
            return None
        else:
            a=(r0**2-r1**2+d**2)/(2*d)
            h=math.sqrt(r0**2-a**2)
            x2=x0+a*(x1-x0)/d   
            y2=y0+a*(y1-y0)/d   
            x3=x2+h*(y1-y0)/d     
            y3=y2-h*(x1-x0)/d 

            x4=x2-h*(y1-y0)/d
            y4=y2+h*(x1-x0)/d
            
        return ((x3, y3), (x4, y4))

    def _angles_to_position(self, left_angle, right_angle):
        # Forward kinematics.
        # Assume the left stepper is at the origin, and both steppers are at y=0
        x_left, y_left = 0,0
        x_right, y_right = self.inner_distance,0

        # First find the coords of the elbow joints 
        left_elbow_pos = self._calculate_line(x_left, y_left, left_angle, self.left_inner_arm)
        right_elbow_pos = self._calculate_line(x_right, y_right, right_angle, self.right_inner_arm)
        # now find the intersection of two circles centred around these elbows. There will be 2 candidates.
        candidates = self._get_intersections(left_elbow_pos, self.left_outer_arm, right_elbow_pos, self.right_outer_arm)
        # in this working mode (working mode 2) the endpoint is the one with the most positive Y position
        endpoint = candidates[0] if candidates[0][1] > candidates[1][1] else candidates[1]
        return endpoint

    def _position_to_angles(self, x, y):
        # Inverse kinematics. Returns None if no valid solution.
        # Assume the left stepper is at the origin, and both steppers are at y=0
        x_left, y_left = 0,0
        x_right, y_right = self.inner_distance,0
        # find the position of the left elbow. It is at an intersection of 2 circles, 
        # the endpoint and the left stepper
        left_candidates = self._get_intersections((x_left, y_left), self.left_inner_arm, 
                                                  (x,y), self.left_outer_arm)
        if not left_candidates:
            return None #no valid angle!
        # otherwise the valid candidate is the left-most one (lowest X)
        left_elbow = left_candidates[0] if left_candidates[0][0] < left_candidates[1][0] else left_candidates[1]
        left_angle = math.atan2(left_elbow[1]-y_left, left_elbow[0]-x_left)
        # we don't want negative thetas on the left stepper.
        if left_angle<0: left_angle+=math.pi * 2
        # now do the right side
        right_candidates = self._get_intersections((x_right, y_right), self.right_inner_arm, 
                                                  (x,y), self.right_outer_arm)
        if not right_candidates:
            return None #no valid angle!
        # otherwise the valid candidate is the right-most one (highest X)
        right_elbow = right_candidates[0] if right_candidates[0][0] > right_candidates[1][0] else right_candidates[1]
        right_angle = math.atan2(right_elbow[1]-y_right, right_elbow[0]-x_right)

        return left_angle, right_angle

    def calc_position(self, stepper_positions):
        # Motor pos -> caretesian coordinates (forward kinematics)
        pos = [stepper_positions[rail.get_name()] for rail in self.rails]
        left_angle  = pos[0]
        right_angle = pos[1]
        x, y = self._angles_to_position(left_angle, right_angle)
        return [x, y, pos[2]]

    def set_position(self, newpos, homing_axes):
        logging.info("Fivebar setting position to %s %s", newpos, homing_axes)
        for i, rail in enumerate(self.rails):
            rail.set_position(newpos)
        if 0 in homing_axes and 1 in homing_axes:
            self.homedXY = True
            logging.info("Set xy %f %f", newpos[0], newpos[1])
        if 2 in homing_axes:
            self.limit_z = self.rails[2].get_range()
            logging.info("Set z limit %f %f", self.limit_z[0], self.limit_z[1])

    def note_z_not_homed(self):
        # Helper for Safe Z Home
        self.limit_z = (1.0, -1.0)
    def home(self, homing_state):
        axes = homing_state.get_axes()
        logging.info("Fivebar home %s", axes)
        if 0 in axes or 1 in axes: #  XY
            # Home left then right!
            # disable steppers - important
            stepper_enable = self.printer.lookup_object('stepper_enable')
            stepper_enable.motor_off() # disables all steppers. They will be enabled by the homing process.

            self.homedXY = False
            homing_state.set_axes([0, 1])
            rails = [self.rails[0], self.rails[1]]
            l_endstop = rails[0].get_homing_info().position_endstop
            l_min, l_max = rails[0].get_range()
            l_safe = rails[0].safe_home_angle

            r_endstop = rails[1].get_homing_info().position_endstop
            r_min, r_max = rails[1].get_range()
            r_safe = rails[1].safe_home_angle

            # Swap to linear kinematics
            toolhead = self.printer.lookup_object('toolhead')
            toolhead.flush_step_generation()

            steppers = self.get_arm_steppers()
            kinematics = [self.cartesian_kinematics_L,
                          self.cartesian_kinematics_R]
            # pop in the cartesian kinematics and hold the old one in this variable
            prev_sks    = [stepper.set_stepper_kinematics(kinematic)
                            for stepper, kinematic in zip(steppers, kinematics)]

            try:
                self.ishoming = True
                # homepos  = [l_endstop, r_endstop, None, None]
                homepos = [l_endstop, None, None]
                hi0 = rails[0].get_homing_info()
                # is the endstop within 25% of the position_max, or 25% of the position_min?
                if hi0.positive_dir:
                    forcepos = [l_min, None, None]
                else:
                    forcepos = [l_max, None, None] 
                
                # home first rail (left)
                # the homing function alters kinematics class to think rail is at forcepos.
                # the stepper then moves towards homepos until endstop is triggered
                logging.info("Fivebar home L %s %s", forcepos, homepos)
                homing_state.home_rails([rails[0]], forcepos, homepos)
                # this then sets the toolhead position to homepos 
                # The stepper now thinks its at forcepos (eg 0)
                # We need to move stepper L towards safe_homing_angle
                logging.info("Fivebar moving L to safe home %s", rails[0].safe_home_angle)
                toolhead.manual_move([rails[0].safe_home_angle, None, None], hi0.speed)
                

                # now home R
                logging.info("Fivebar homing R now")
                homepos = [None, r_endstop, None]
                hi1 = rails[1].get_homing_info()
                if hi1.positive_dir: #should be false
                    forcepos = [None, r_min, None]
                else: 
                    forcepos = [None, r_max, None]
                # we should be moving from positive to negative
                homing_state.home_rails([rails[1]], forcepos, homepos)
                logging.info("Fivebar moving R to safe home %s", rails[1].safe_home_angle)
                toolhead.manual_move([None, rails[1].safe_home_angle, None], hi0.speed)
                toolhead.flush_step_generation()

                self.ishoming = False
                # restore kinematics
                logging.info("Fivebar restoring kinematics")
                for stepper, prev_sk in zip(steppers, prev_sks):
                    stepper.set_stepper_kinematics(prev_sk)

                [x,y] = self._angles_to_position(
                    rails[0].safe_home_angle,
                    rails[1].safe_home_angle)
                
                toolhead.set_position( [x, y, 0, 0], (0, 1))
                toolhead.flush_step_generation()
                self.homedXY = True
                logging.info("Homed LR done")

            except Exception as e:
                for stepper, prev_sk in zip(steppers, prev_sks):
                    stepper.set_stepper_kinematics(prev_sk)
                toolhead.flush_step_generation()
                raise

        if 2 in axes: # Z
            rail = self.rails[2]
            position_min, position_max = rail.get_range()
            hi = rail.get_homing_info()
            homepos = [None, None, hi.position_endstop]
            forcepos = list(homepos)
            if hi.positive_dir:
                forcepos[2] -= 1.5 * (hi.position_endstop - position_min)
            else:
                forcepos[2] += 1.5 * (position_max - hi.position_endstop)
            # Perform homing
            logging.info("Fivebar homing Z %s %s", forcepos, homepos);
            homing_state.home_rails([rail], forcepos, homepos)

    def _motor_off(self, print_time):
        self.homedXY = False
        self.limit_z = (1.0, -1.0)

    def check_move(self, move):
        #during homing state, the kinematics are not valid anyway
        if self.ishoming: 
            return True 
        end_pos = move.end_pos
        # XY moves
        if move.axes_d[0] or move.axes_d[1]:
            xpos, ypos = end_pos[:2]
            logging.info("Checking move to %s %s", xpos, ypos)
            if not (self.ishoming or self.homedXY):
                raise move.move_error("Must home axis first")

            # Check that coordinate is in front
            if ypos < 0:
                raise move.move_error("Attempted move behind printer")

            # Make sure distance from left and right attachement point is
            # no further away that inner+outer
            target_angles = self._position_to_angles(xpos, ypos)
            if not target_angles:
                raise move.move_error(
                    "Attempted move outside reachable area (arm length)")

            left_min_a, left_max_a = self.rails[0].get_range()
            if not (left_min_a <= target_angles[0] <= left_max_a):
                raise move.move_error(
                    "Attempted move left arm outside angle limits: %.2f (%.2f %.2f)" %
                    (target_angles[0], left_min_a, left_max_a))

            right_min_a, right_max_a = self.rails[1].get_range()
            if not (right_min_a <= target_angles[1] <= right_max_a):
                raise move.move_error(
                    "Attempted move right arm outside angle limits: %.2f (%.2f %.2f)" %
                    (target_angles[1], right_min_a, right_max_a))

            # TODO: Speed limits

        # Check Z
        if move.axes_d[2]:
            if end_pos[2] < self.limit_z[0] or end_pos[2] > self.limit_z[1]:
                if self.limit_z[0] > self.limit_z[1]:
                    raise move.move_error("Must home axis first")
                raise move.move_error("Z out of range")
            # Move with Z - update velocity and accel for slower Z axis
            z_ratio = move.move_d / abs(move.axes_d[2])
            move.limit_speed(
                self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio)

    def get_status(self, eventtime):
        xy_home = "xy" if self.homedXY else ""
        z_home = "z" if self.limit_z[0] <= self.limit_z[1] else ""
        return {'homed_axes': xy_home + z_home}

def load_kinematics(toolhead, config):
    return FiveBar(toolhead, config)
