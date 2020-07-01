#!/usr/bin/python
import rospy, re, threading
from aerowake_flight.srv import MissionCommand, MissionCommandResponse
from aerowake_flight.srv import TetherLength, TetherLengthResponse
from aerowake_flight.srv import UavAltitude, UavAltitudeResponse
from ublox_msgs.msg import NavVELNED
from geometry_msgs.msg import WrenchStamped
from nav_msgs.msg import Odometry
from rosflight_msgs.msg import Command
from math import atan, sin, pi
from numpy import sign

class TrajectoryPlanner(object):
    def __init__(self):
        self.length = rospy.get_param('~tether_takeoff_length', 0.2)
        self.takeoff_alt = rospy.get_param('~takeoff_altitude', 1.0)
        self.comm_alt_buf = rospy.get_param('~commanded_altitude_buffer', 0.1)
        self.tension_threshold = rospy.get_param('~tension_threshold', 2.0)
        self.tensioner_pitch = rospy.get_param('~tensioner_pitch', 0.1) # make a max value?
        self.kp_lateral = rospy.get_param('~kp_lateral', 0.5)
        self.v_max = rospy.get_param('~v_max', 1.2)
        self.commanded_rel_pos_buffer = rospy.get_param('~commanded_rel_pos_buffer', 0.1)

        self.rel_yaw_home = pi * rospy.get_param('~rel_yaw_home', 0.0) / 180.0
        self.rel_yaw_buff = pi * rospy.get_param('~commanded_rel_yaw_buffer', 10.0) / 180.0
        self.sweep_angle  = pi * rospy.get_param('~sweep_angle', 45.0) / 180.0
        self.max_rel_yaw  = pi * rospy.get_param('~max_rel_yaw', 90) / 180.0

        self.sweep_rel_yaw_lower = max(-self.max_rel_yaw, self.rel_yaw_home - self.sweep_angle) + self.rel_yaw_buff
        self.sweep_rel_yaw_upper = min( self.max_rel_yaw, self.rel_yaw_home + self.sweep_angle) - self.rel_yaw_buff

        self.mcS  = rospy.Service('mission_command', MissionCommand, self.MCCallback)
        self.tlS  = rospy.Service('tether_length', TetherLength, self.TLCallback)
        self.tltS = rospy.ServiceProxy('tether_length_tether', TetherLength)
        self.uaS  = rospy.Service('uav_altitude', UavAltitude, self.UACallback)

        self.abs_pose  = None
        self.abs_twist = None
        self.rel_pose  = None
        self.rel_yaw   = 0.0
        self.rel_twist = None
        self.Fext      = None
        self.shipVel   = rospy.get_param('~ship_vel_guess', 1.0)

        self.command = Command()
        self.command.mode = Command.MODE_ROLL_PITCH_YAWRATE_THROTTLE
        self.command.ignore = 0
        self.command.x = 0.0
        self.command.y = 0.0
        self.command.z = 0.0
        self.command.F = 0.0

        self.abs_ekf_sub = rospy.Subscriber("odometry", Odometry, self.abs_ekf_callback)
        self.rel_ekf_sub = rospy.Subscriber("rel_odometry", Odometry, self.rel_ekf_callback)
        self.Fext_sub    = rospy.Subscriber("Fext", WrenchStamped, self.fext_callback)
        self.ship_v_sub  = rospy.Subscriber("base/navvelned", NavVELNED, self.ship_v_callback)
        self.hlc_com_pub = rospy.Publisher("high_level_command", Command, queue_size=1)

        self.locked = False
        self.checkRate = rospy.Rate(2.0)
        self.flying = False
        self.inTension = False

        self.tensionThread = threading.Thread(target=self.establishTension)
        self.command_timer = rospy.Timer(rospy.Duration(1.0/20.0), self.update)

    def update(self, event):
        self.hlc_com_pub.publish(self.command)

    def MCCallback(self, req): # NEED AN OVERRIDING "HOLD AEROWAKE" COMMAND
        mcstring = req.command.lower()
        if not self.locked:
            if mcstring == "takeoff":
                rospy.loginfo('[TRAJECTORY PLANNER] Processing command: %s' % mcstring)
                self.takeoff()
            elif mcstring == "rth":
                rospy.loginfo('[TRAJECTORY PLANNER] Processing command: %s' % mcstring)
                self.emergency_rth()
            elif mcstring == "land":
                rospy.loginfo('[TRAJECTORY PLANNER] Processing command: %s' % mcstring)
                self.land()
            elif mcstring == "sweep":
                rospy.loginfo('[TRAJECTORY PLANNER] Processing command: %s' % mcstring)
                self.sweep()
            elif mcstring == "center0":
                rospy.loginfo('[TRAJECTORY PLANNER] Processing command: %s' % mcstring)
                self.center()
            elif mcstring == "center":
                rospy.loginfo('[TRAJECTORY PLANNER] Processing command: %s' % mcstring)
                self.center(self.rel_yaw_home)
            else:
                rospy.logerr('[TRAJECTORY PLANNER] Unrecognized command: %s' % mcstring)
                return MissionCommandResponse(False)
            return MissionCommandResponse(True)
        else:
            rospy.logwarn('[TRAJECTORY PLANNER] Cannot process command; mutex locked.')

    def UACallback(self, req):
        if self.abs_pose is None:
            rospy.logerr('NO STATE RECEIVED; NOT GOING TO CHANGE ALTITUDE')
        else:
            matchstr = re.findall(r'(\+|-)?((\d+(\.\d+))|(\d+))?',req.altitude_string)[0][0:2]
            if matchstr[0] == '+':
                comm_alt = -self.abs_pose.position.z + float(matchstr[1])
            elif matchstr[0] == '-':
                comm_alt = -self.abs_pose.position.z - float(matchstr[1])
            else:
                comm_alt = float(matchstr[1])
            self.command.mode = Command.MODE_PITCH_YVEL_YAW_ALTITUDE
            self.command.F = comm_alt

        return UavAltitudeResponse(True)

    def abs_ekf_callback(self, msg):
        self.abs_pose  = msg.pose.pose
        self.abs_twist = msg.twist.twist

    def rel_ekf_callback(self, msg):
        self.rel_pose  = msg.pose.pose
        if not self.rel_pose.position.x == 0.0:
            self.rel_yaw = atan(self.rel_pose.position.y / self.rel_pose.position.x)
        else:
            self.rel_yaw = -sign(self.rel_pose.position.y) * pi / 2.0

        self.rel_twist = msg.twist.twist

    def fext_callback(self, msg):
        self.Fext = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z]

    def ship_v_callback(self, msg):
        self.shipVel = 1.0e-2*float(msg.gSpeed)

    def TLCallback(self, req):
        rospy.loginfo("Attempting to set tether length...")
        try:
            resp = self.tltS(req.tether_length_string)
            if not resp.success:
                rospy.logwarn("WARNING: couldn't set tether length!")
                return TetherLengthResponse(False)
        except rospy.ServiceException as exc:
            rospy.logwarn("WARNING: tether length couldn't be set: %s" % str(exc))
            return TetherLengthResponse(False)

        matchstr = re.findall(r'(\+|-)?((\d+(\.\d+))|(\d+))?',req.tether_length_string)[0][0:2]
        if matchstr[0] == '+':
            self.length += float(matchstr[1])
        elif matchstr[0] == '-':
            self.length -= float(matchstr[1])
        else:
            self.length = float(matchstr[1])

        # Establish tension with new tether length
        if not self.tensionThread.isAlive():
            # self.establishTension()
            self.tensionThread.start()

        return TetherLengthResponse(True)

    # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    def takeoff(self):
        if not self.flying:
            self.locked = True

            self.command.mode = Command.MODE_PITCH_YVEL_YAW_ALTITUDE
            self.command.x = 0.0
            self.command.y = 0.0
            self.command.z = 0.0
            self.command.F = self.takeoff_alt

            while abs(-self.abs_pose.position.z - self.takeoff_alt) > self.comm_alt_buf:
                rospy.logwarn_throttle(2, "Still trying to reach target takeoff altitude [{} -> {}]".format(-self.abs_pose.position.z, self.takeoff_alt))
                self.checkRate.sleep()

            self.locked = False
            self.flying = True
            rospy.loginfo('[TRAJECTORY PLANNER] Takeoff operation complete.')
        else:
            rospy.logwarn('[TRAJECTORY PLANNER] Cannot takeoff when already flying.')

    def land(self):
        if self.flying:
            self.locked = True

            self.command.mode = Command.MODE_PITCH_YVEL_YAW_ALTITUDE
            self.command.x = 0.0
            self.command.y = 0.0
            self.command.z = 0.0
            self.command.F = 0.0

            while abs(-self.abs_pose.position.z) > self.comm_alt_buf:
                self.checkRate.sleep()

            self.locked = False
            self.flying = False
            rospy.loginfo('[TRAJECTORY PLANNER] Land operation complete.')
        else:
            rospy.logwarn('[TRAJECTORY PLANNER] Cannot land when not flying.')

    def establishTension(self):
        # Alter the UAV's pitch so that tension is created (or maintained)
        # in the tether line
        # self.locked = True

        self.command.mode = Command.MODE_PITCH_YVEL_YAW_ALTITUDE
        self.command.x = self.tensioner_pitch

        while self.command.x == self.tensioner_pitch:
            # tension thresholding
            self.inTension = (self.Fext[0] >= self.tension_threshold)
            # simple attempt to keep UAV pointed at the ship
            self.command.z = self.rel_yaw
            # report
            rospy.logwarn_throttle(5.0, 'MEASURED TENSION STATUS: ' + str(self.inTension) + '\nCOMMANDED RELATIVE YAW: ' + str(180.0*self.rel_yaw/pi) + ' deg')
            self.checkRate.sleep()

        # self.locked = False
        rospy.loginfo('[TRAJECTORY PLANNER] Tension seeking ceased.')

    def center(self, val=0.0):
        if self.inTension:
            self.locked = True

            self.command.mode = Command.MODE_PITCH_YVEL_YAW_ALTITUDE

            des_y = -self.length * sin(val)

            while abs(self.rel_pose.position.y - des_y) > 0.01:
                self.command.y = self.P(self.rel_pose.position.y, des_y, self.kp_lateral, self.v_max, -self.v_max) - self.shipVel * sin(self.rel_yaw)
                # print(self.command.y)
                self.checkRate.sleep()

            self.locked = False
            rospy.loginfo('[TRAJECTORY PLANNER] Center operation complete.')
        else:
            rospy.logwarn('[TRAJECTORY PLANNER] Cannot center without established tether tension.')

    def sweep(self):
        if self.inTension:
            rospy.loginfo('[TRAJECTORY PLANNER] Beginning sweep...')
            sweep_state = 0 # 0 for LEFT, 1 for RIGHT
            while not self.locked:

                if sweep_state == 0:
                    rospy.loginfo('[TRAJECTORY PLANNER] Sweeping left...')
                    while not self.locked and self.rel_yaw > self.sweep_rel_yaw_lower:
                        self.command.y = self.v_max - self.shipVel * sin(self.rel_yaw)
                        self.checkRate.sleep()
                    # while not self.locked and self.rel_pose.position.x < -2.0: # self.rel_pose.position.y < -0.75*self.rel_pose.position.x:
                    #     self.command.y = self.v_max
                    #     self.checkRate.sleep()
                    # self.command.y = 0.0
                    # for i in range(4):
                    #     self.checkRate.sleep()
                    sweep_state = 1

                else:
                    rospy.loginfo('[TRAJECTORY PLANNER] Sweeping right...')
                    while not self.locked and self.rel_yaw < self.sweep_rel_yaw_upper:
                        self.command.y = -self.v_max - self.shipVel * sin(self.rel_yaw)
                        self.checkRate.sleep()
                    # while not self.locked and self.rel_pose.position.x < -2.0: # self.rel_pose.position.y > 0.75*self.rel_pose.position.x:
                    #     self.command.y = -self.v_max
                    #     self.checkRate.sleep()
                    # self.command.y = 0.0
                    # for i in range(4):
                    #     self.checkRate.sleep()
                    sweep_state = 0

        else:
            rospy.logwarn('[TRAJECTORY PLANNER] Cannot sweep without established tether tension.')

    def emergency_rth(self):
        rospy.loginfo('[TRAJECTORY PLANNER] Establishing tension and centering...')
        if not self.tensionThread.isAlive():
            # self.establishTension()
            self.tensionThread.start()
        self.center()

        rospy.loginfo('[TRAJECTORY PLANNER] Getting to takeoff altitude...')
        self.command.mode = Command.MODE_PITCH_YVEL_YAW_ALTITUDE
        self.command.F = self.takeoff_alt

        while abs(-self.abs_pose.position.z - self.takeoff_alt) > self.comm_alt_buf:
            self.checkRate.sleep()

        rospy.logwarn('[TRAJECTORY PLANNER] RC TAKEOVER REQUIRED TO RETURN UAV')

    # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    def P(self, val, des, gain, maxv=9999.0, minv=-9999.0):
        return self.sat(gain * (des - val), maxv, minv)

    def sat(self, val, max_val, min_val):
        if val > max_val:
            return max_val
        if val < min_val:
            return min_val
        return val

if __name__ == "__main__":
    rospy.init_node("trajectory_planner", anonymous=True)
    TP = TrajectoryPlanner()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down trajectory planner Node."
