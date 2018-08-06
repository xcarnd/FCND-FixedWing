# -*- coding: utf-8 -*-
import numpy as np


PI = 3.14159


class Params(object):
    def __init__(self):
        # monitor file changes if watchdog is installed
        # first, check if watchdog is installed
        self.params = {
            "Kp_roll": 14.0,
            "Kp_p": 0.15,
            "Kp_pitch": 2.4,
            "Kp_q": 0.3,
            "Kp_yaw": 2.6,
            "Ki_yaw": 0.2,
            "Kp_alt": 0.03,
            "Ki_alt": 0.02,
            "Kp_speed": 0.15,
            "Ki_speed": 0.05,
            "Kp_speed2": -0.12,
            "Ki_speed2": -0.08,
            "T_ff": 0.667,
            "Trans_Hold_Climb_dalt": 30,
            "Kp_sideslip": -1.8,
            "Ki_sideslip": -0.15,
            "K_track": 0.008,
            "Kp_orbit": 0.066
        }

    def get(self, parma_name):
        return self.params[parma_name]

params = Params()


class LongitudinalAutoPilot(object):
    def __init__(self):
        self.max_throttle_rpm = 2500
        self.max_elevator = 30.0*PI/180.0
        
        self.min_throttle = 0.0
        self.max_throttle = 1.0
        self.max_pitch_cmd = 30.0*np.pi/180.0
        self.max_pitch_cmd2 = 45.0*np.pi/180.0
        
        self.speed_int = 0.0
        self.alt_int = 0.0
        self.climb_speed_int = 0.0
    
    """Used to calculate the elevator command required to acheive the target
    pitch
    
        Args:
            pitch: in radians
            pitch_rate: in radians/sec
            pitch_cmd: in radians
        
        Returns:
            elevator_cmd: in percentage elevator [-1,1]
    """
    def pitch_loop(self, pitch, pitch_rate, pitch_cmd):
        elevator_cmd = 0.0
        # STUDENT CODE HERE
        kp = params.get("Kp_pitch")
        kd = params.get("Kp_q")
        e_pitch = pitch_cmd - pitch
        elevator_cmd = kp * e_pitch - kd * pitch_rate
        elevator_cmd = np.clip(elevator_cmd, -self.max_elevator, self.max_elevator) / self.max_elevator
        return elevator_cmd
    
    """Used to calculate the pitch command required to maintain the commanded
    altitude
    
        Args:
            altitude: in meters (positive up)
            altitude_cmd: in meters (positive up)
            dt: timestep in seconds
        
        Returns:
            pitch_cmd: in radians
    """
    def altitude_loop(self, altitude, altitude_cmd, dt):
        pitch_cmd = 0.0
        # STUDENT CODE HERE
        kp = params.get("Kp_alt")
        ki = params.get("Ki_alt")

        e_alt = altitude_cmd - altitude
        pitch_cmd = kp * e_alt
        # anti-windup. include the i term only when e_alt is small (10 meters?)
        if abs(e_alt) < 10:
            self.alt_int += e_alt * dt
            pitch_cmd += ki * self.alt_int
        pitch_cmd = np.clip(pitch_cmd, -self.max_pitch_cmd, self.max_pitch_cmd)
                
        return pitch_cmd
    

    """Used to calculate the throttle command required command the target 
    airspeed
        
        Args:
            airspeed: in meters/sec
            airspeed_cmd: in meters/sec
            dt: timestep in seconds
        
        Returns:
            throttle_command: in percent throttle [0,1]
    """
    def airspeed_loop(self, airspeed, airspeed_cmd, dt):        
        throttle_cmd = 0.0
        # STUDENT CODE HERE
        kp = params.get("Kp_speed")
        ki = params.get("Ki_speed")
        t_ff = params.get("T_ff")

        e_speed = airspeed_cmd - airspeed
        throttle_cmd = kp * e_speed + t_ff
        if abs(e_speed) < 3:
            self.speed_int += e_speed * dt
            throttle_cmd += ki * self.speed_int
        
        return throttle_cmd
    """Used to calculate the pitch command required to maintain the commanded
    airspeed
    
        Args:
            airspeed: in meters/sec
            airspeed_cmd: in meters/sec
            dt: timestep in seconds
        
        Returns:
            pitch_cmd: in radians
    """
    def airspeed_pitch_loop(self, airspeed, airspeed_cmd, dt):
        pitch_cmd = 0.0
        # STUDENT CODE HERE
        kp = params.get("Kp_speed2")
        ki = params.get("Ki_speed2")

        e_speed = airspeed_cmd - airspeed
        pitch_cmd = kp * e_speed
        if abs(e_speed) < 5:
            self.climb_speed_int += e_speed * dt
            pitch_cmd += ki * self.climb_speed_int

        pitch_cmd = np.clip(pitch_cmd, -self.max_pitch_cmd2, self.max_pitch_cmd2)
        return pitch_cmd
    
    """Used to calculate the pitch command and throttle command based on the
    aicraft altitude error
    
        Args:
            airspeed: in meter/sec
            altitude: in meters (positive up)
            airspeed_cmd: in meters/sec
            altitude_cmd: in meters/sec (positive up)
            dt: timestep in seconds
            
        Returns:
            pitch_cmd: in radians
            throttle_cmd: in in percent throttle [0,1]
    """
    def longitudinal_loop(self, airspeed, altitude, airspeed_cmd, altitude_cmd,
                          dt):
        pitch_cmd = 0.0
        throttle_cmd = 0.0
        # STUDENT CODE HERE
        dalt = params.get("Trans_Hold_Climb_dalt")
        e_alt = altitude_cmd - altitude
        if abs(e_alt) < dalt:
            # into altitude hold mode
            pitch_cmd = self.altitude_loop(altitude, altitude_cmd, dt)
            throttle_cmd = self.airspeed_loop(airspeed, airspeed_cmd, dt)
        else:
            if e_alt > 0:
                # into steady climb mode
                throttle_cmd = self.max_throttle
            else:
                # into steady descent mode
                throttle_cmd = self.min_throttle
            pitch_cmd = self.airspeed_pitch_loop(airspeed, airspeed_cmd, dt)

        return[pitch_cmd, throttle_cmd]


 
class LateralAutoPilot:
    
    def __init__(self):
        self.g = 9.81
        self.integrator_yaw = 0.0 
        self.integrator_beta = 0.0
        self.gate = 1
        self.max_roll = 60*np.pi/180.0
        self.state = 1
        self.gates = np.array([[500, 20],
                               [900, -380],
                               [600, -680],
                               [-450, -680]])

    """Used to calculate the commanded aileron based on the roll error
    
        Args:
            phi_cmd: commanded roll in radians
            phi: roll angle in radians
            roll_rate: in radians/sec
            T_s: timestep in sec
            
        Returns:
            aileron: in percent full aileron [-1,1]
    """
    def roll_attitude_hold_loop(self,
                                phi_cmd,  # commanded roll
                                phi,    # actual roll 
                                roll_rate, 
                                T_s = 0.0):
        aileron = 0
        # STUDENT CODE HERE
        kp = params.get("Kp_roll")
        kd = params.get("Kp_p")
        e_phi = phi_cmd - phi
        cmd = kp * e_phi - kd * roll_rate
        aileron = np.clip(cmd / self.max_roll, -1.0, 1.0)
        return aileron

    """Used to calculate the commanded roll angle from the course/yaw angle
    
        Args:
            yaw_cmd: commanded yaw in radians
            yaw: roll angle in radians
            roll_rate: in radians/sec
            T_s: timestep in sec
            
        Returns:
            roll_cmd: commanded roll in radians
    """
    def yaw_hold_loop(self,
                         yaw_cmd,  # desired heading
                         yaw,     # actual heading 
                         T_s,
                         roll_ff=0):
        roll_cmd = 0
        
        # STUDENT CODE HERE
        kp = params.get("Kp_yaw")
        ki = params.get("Ki_yaw")
        e_yaw = yaw_cmd - yaw
        while e_yaw > np.pi:
            e_yaw -= 2 * np.pi
        while e_yaw < -np.pi:
            e_yaw += 2 * np.pi
        roll_cmd = kp * e_yaw
        if abs(e_yaw) < 0.1:
            self.integrator_yaw += e_yaw * T_s
            roll_cmd += ki * self.integrator_yaw
        roll_cmd = np.clip(roll_cmd, -self.max_roll, self.max_roll)
        
        return roll_cmd


    """Used to calculate the commanded rudder based on the sideslip
    
        Args:
            beta: sideslip angle in radians
            T_s: timestep in sec
            
        Returns:
            rudder: in percent full rudder [-1,1]
    """
    def sideslip_hold_loop(self,
                           beta, # sideslip angle 
                           T_s):
        rudder = 0
        # STUDENT CODE HERE
        kp = params.get("Kp_sideslip")
        ki = params.get("Ki_sideslip")
        e = -beta
        cmd = kp * e
        if abs(beta) < 0.2:
            self.integrator_beta += e * T_s
            cmd += ki * self.integrator_beta
        rudder = np.clip(cmd, -1, 1)
        return rudder
    
    """Used to calculate the desired course angle based on cross-track error
    from a desired line
    
        Args:
            line_origin: point on the desired line in meters [N, E, D]
            line_course: heading of the line in radians
            local_position: vehicle position in meters [N, E, D]
            
        Returns:
            course_cmd: course/yaw cmd for the vehicle in radians
    """
    def straight_line_guidance(self, line_origin, line_course, 
                               local_position):
        course_cmd = 0
        # STUDENT CODE HERE
        kp = params.get("K_track")
        po = np.array(line_origin)[:2]
        pp = np.array(local_position)[:2]
        pd = po + np.array([np.cos(line_course), np.sin(line_course)])

        error = np.cross(pp - po, pd - po) / np.linalg.norm(pd - po)
        course_cmd = kp * error + line_course
        
        return course_cmd
    
    """Used to calculate the desired course angle based on radius error from
    a specified orbit center
    
        Args:
            orbit_center: in meters [N, E, D]
            orbit_radius: desired radius in meters
            local_position: vehicle position in meters [N, E, D]
            yaw: vehicle heading in radians
            clockwise: specifies whether to fly clockwise (increasing yaw)
            
        Returns:
            course_cmd: course/yaw cmd for the vehicle in radians
    """
    def orbit_guidance(self, orbit_center, orbit_radius, local_position, yaw,
                       clockwise = True):
        course_cmd = 0
        kp = params.get("Kp_orbit")
        # STUDENT CODE HERE
        po = np.array(orbit_center)[:2]
        pp = np.array(local_position)[:2]
        op = pp - po
        actual_radius = np.linalg.norm(op)
        e_radius = orbit_radius - actual_radius
        if clockwise:
            course_cmd = -kp * e_radius
        else:
            course_cmd = kp * e_radius

        heading = np.arctan2(op[1], op[0])
        if clockwise:
            course_ff = heading + (np.pi / 2)
        else:
            course_ff = heading - (np.pi / 2)

        course_cmd += course_ff
        while course_cmd > np.pi:
            course_cmd -= 2 * np.pi
        while course_cmd < -np.pi:
            course_cmd += 2 * np.pi
        return course_cmd

    """Used to calculate the feedforward roll angle for a constant radius
    coordinated turn
    
        Args:
            speed: the aircraft speed during the turn in meters/sec
            radius: turning radius in meters
            cw: true=clockwise turn, false = counter-clockwise turn
            
        Returns:
            roll_ff: feed-forward roll in radians
    """
    def coordinated_turn_ff(self, speed, radius, cw):
        
        roll_ff = 0
        # STUDENT CODE HERE
        # equations for the math:
        # F: lift force
        # theta: roll angle
        # r: turning radis
        # m: mass
        # g: gravity constant
        # v: aircraft speed (linear velocity)
        #
        # for level flight, F * cos(theta) = m * g
        # when performing cyclic movement, centripetal force Fr = m * v**2 / R
        # and Fr = F * sin(theta)
        # so:
        # F * cos(theta) = Fr / sin(theta) = m * v** 2 * cos(theta) / (R * sin(theta)) = m * g
        # g * R = v ** 2 * cos(theta) / sin(theta)
        # tan(theta) = v ** 2 / (g * R)
        # theta = arctan(v ** 2 / (g * R))
        roll_ff = np.arctan(speed ** 2 / (self.g * radius)) * (1 if cw else -1)
        if not cw:
            roll_ff = -roll_ff
        return roll_ff

    """Used to calculate the desired course angle and feed-forward roll
    depending on which phase of lateral flight (orbit or line following) the 
    aicraft is in
    
        Args:
            local_position: vehicle position in meters [N, E, D]
            yaw: vehicle heading in radians
            airspeed_cmd: in meters/sec
            
        Returns:
            roll_ff: feed-forward roll in radians
            yaw_cmd: commanded yaw/course in radians
    """
    def path_manager(self, local_position, yaw, airspeed_cmd):
        roll_ff = 0
        yaw_cmd = 0
        # STUDENT CODE HERE
        if self.gate > 4:
            return [0, 0]
        dist_to_gate = np.linalg.norm(np.array(local_position)[:2] - self.gates[self.gate - 1])
        if dist_to_gate < 10:
            print(dist_to_gate)
            self.gate += 1
            print("Gate #{}: {}".format(self.gate, self.gates[self.gate - 1]))
        if self.gate == 1:
            line_origin = np.array([0, 20])
            line_course = 0.0
            yaw_cmd = self.straight_line_guidance(line_origin, line_course, local_position)
        elif self.gate == 2:
            orbit_center = np.array([500, -380])
            orbit_radius = 400.0
            cw = False
            yaw_cmd = self.orbit_guidance(orbit_center, orbit_radius, local_position, yaw, cw)
            roll_ff = self.coordinated_turn_ff(airspeed_cmd, orbit_radius, cw)
        elif self.gate == 3:
            orbit_center = np.array([600, -380])
            orbit_radius = 300.0
            cw = False
            yaw_cmd = self.orbit_guidance(orbit_center, orbit_radius, local_position, yaw, cw)
            roll_ff = self.coordinated_turn_ff(airspeed_cmd, orbit_radius, cw)
        else: # self.gate == 4
            line_origin = np.array([600, -680])
            line_course = np.pi
            yaw_cmd = self.straight_line_guidance(line_origin, line_course, local_position)
        
        return(roll_ff,yaw_cmd)
    
    
    """Used to calculate the desired course angle and feed-forward roll
    depending on which phase of lateral flight (orbit or line following) the 
    aicraft is in
    
        Args:
            waypoint_tuple: 3 waypoints, (prev_waypoint, curr_waypoint, next_waypoint), waypoints are in meters [N, E, D]
            local_position: vehicle position in meters [N, E, D]
            yaw: vehicle heading in radians
            airspeed_cmd: in meters/sec
            
        Returns:
            roll_ff: feed-forward roll in radians
            yaw_cmd: commanded yaw/course in radians
            cycle: True=cycle waypoints (at the end of orbit segment)
    """
    def waypoint_follower(self, waypoint_tuple, local_position, yaw, airspeed_cmd):
        roll_ff = 0.0
        yaw_cmd = 0.0
        cycle = False
        
        # STUDENT CODE HERE
        
        
        
        return(roll_ff, yaw_cmd, cycle)



def euler2RM(roll,pitch,yaw):
    R = np.array([[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]])
    cr = np.cos(roll)
    sr = np.sin(roll)
    
    cp = np.cos(pitch)
    sp = np.sin(pitch)
    
    cy = np.cos(yaw)
    sy = np.sin(yaw)
    
    R[0,0] = cp*cy
    R[1,0] = -cr*sy+sr*sp*cy
    R[2,0] = sr*sy+cr*sp*cy
    
    R[0,1] = cp*sy
    R[1,1] = cr*cy+sr*sp*sy
    R[2,1] = -sr*cy+cr*sp*sy
    
    R[0,2] = -sp
    R[1,2] = sr*cp
    R[2,2] = cr*cp
    
    return R.transpose()