import rospy
from std_msgs.msg import Float64, Int8, Bool
from hand_arduino.msg import grasp

class hand_driver:
    def __init__(self):
        self.grasp_set_pos_pub = rospy.Publisher('hand/grasp/set_position', Float64, queue_size=10)
        self.start_grasp_pub = rospy.Publisher('hand/grasp/start_grasp', grasp, queue_size=10)
        self.stop_grasp_pub = rospy.Publisher('hand/grasp/stop', Bool, queue_size=10)

        self.spread_set_pos_pub = rospy.Publisher('hand/spread/set_position', Float64, queue_size=10)
        self.start_spread_pub = rospy.Publisher('hand/spread/start_spread', Bool, queue_size=10)
        self.start_unspread_pub = rospy.Publisher('hand/spread/start_unspread', Bool, queue_size=10)
        self.stop_spread_pub = rospy.Publisher('hand/spread/stop', Bool, queue_size=10)

        self.grasp_load_sub = rospy.Subscriber('hand/grasp/load', Float64, self.grasp_load_cb)
        self.grasp_state_sub = rospy.Subscriber('hand/grasp/state', Int8, self.grasp_state_cb)

        self.current_load = None
        self.grasp_state = None
        self.spread_pos = 525.0
        self.unspread_pos = 708.0

# Functions for grasping
    def grasp(self, speed=20, force=30, force_threshold=90, pos_max=4800):
        grasp_msg = grasp(speed=speed, force=force, force_threshold=force_threshold, pos_max=pos_max)
        self.start_grasp_pub.publish(grasp_msg)

    def ungrasp(self, position=1154):
        self.set_grasp_position(position)

    def set_grasp_position(self, position=1690):
        grasp_pos_setpoint = Float64(data=position)
        self.grasp_set_pos_pub.publish(grasp_pos_setpoint)

    def stop_grasp(self):
        self.stop_grasp_pub.publish(Bool(True))

# Functions for spreading and unspreading
    def set_spread_position(self, position):
        spread_pos_setpoint = Float64(data=position)
        self.spread_set_pos_pub.publish(spread_pos_setpoint)

    def start_spread(self):
        self.start_spread_pub.publish(Bool(True))

    def start_unspread(self):
        self.start_unspread_pub.publish(Bool(True))

    def stop_spread(self):
        self.stop_spread_pub.publish(Bool(True))

# -------------------------------------------------------
    def grasp_load_cb(self, load_msg):
        self.current_load = load_msg.data

    def grasp_state_cb(self, state_msg):
        self.grasp_state = state_msg.data






