#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
import time
import math

###############################################################################
'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

'''

###############################################################################

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number

###############################################################################

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
	rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.current_pose = None
        self.base_waypoints = []
        self.next_waypoint = None
        self._waypoint_redlight = None
	self.reset_velocity_waypoint = [] 
	self.msg_seq = 0

	self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        
	# Parameters
        self.target_acceleration_value = -0.9
        self.halt_at_red_light = True
        self.minimum_stop_margin = 5.0
        self.end_waypoint_halt_override = True
        self.unsubscribe_base__waypoints = rospy.get_param('/unregister_base_waypoints', False)

        try:
            self.target_acceleration_value = max(rospy.get_param('/dbw_node/decel_limit') / 2, self.target_acceleration_value)
        except KeyError:
            pass


        # Launch periodic publishing into /final_waypoints
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.update_and_publish_waypoints()
            rate.sleep()


###############################################################################

    def get_waypoint_velocity(self, waypoints, waypoint):
        return waypoints[waypoint].twist.twist.linear.x

###############################################################################

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

###############################################################################

    def distance(self, waypoints, waypoint_A, waypoint_B):
        threshold_distance = 0
        distance = lambda A, B: math.sqrt((A.x-B.x)**2 + (A.y-B.y)**2  + (A.z-B.z)**2)
        for i in range(waypoint_A, waypoint_B+1):
            threshold_distance += distance(waypoints[waypoint_A].pose.pose.position, waypoints[i].pose.pose.position)
            waypoint_A = i
        return threshold_distance

###############################################################################

    def traffic_cb(self, msg):
        waypoint_of_prev_redlight = self._waypoint_redlight
        self._waypoint_redlight = msg.data if msg.data >= 0 else None
        if waypoint_of_prev_redlight != self._waypoint_redlight:
            if True:
                self.update_and_publish_waypoints()

###############################################################################

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

###############################################################################

    def reset_waypoint_velocities(self, indexes):
       
        for _index in indexes:
            self.set_waypoint_velocity(self.base_waypoints, _index, \
		self.reset_velocity_waypoint[_index])

###############################################################################


    def next_waypoint_update(self):
 
        if not self.base_waypoints:
            return False

        if not self.current_pose:
            return False

        # Ego car x, y and yaw
        ego_car_x = self.current_pose.position.x
        ego_car_y = self.current_pose.position.y
        ego_car_theta = math.atan2(self.current_pose.orientation.y, self.current_pose.orientation.x)

        total_base_waypoints = len(self.base_waypoints)
        threshold_distance = 9999999

        t = time.time()

        _waypoints = None
        yaw_angle = 0

        if self.next_waypoint:
            do_reverse_search = False
            _index_offset = self.next_waypoint
        else:
            do_reverse_search = True
            _index_offset = 0

        for i in range(total_base_waypoints):
	    # Getindex and waypoint infor from the base waypoints
            _index = (i + _index_offset)%(total_base_waypoints)

            _waypoints_x = self.base_waypoints[_index].pose.pose.position.x
            _waypoints_y = self.base_waypoints[_index].pose.pose.position.y

            _waypoints_d = math.sqrt((ego_car_x - _waypoints_x)**2 \
				+ (ego_car_y - _waypoints_y)**2)

	    # Check against the threshold
            if _waypoints_d < threshold_distance:
                threshold_distance = _waypoints_d
                _waypoints = _index
                
            elif not do_reverse_search:
                if threshold_distance < 20:
                    break;
                else:
                    do_reverse_search = True

        if _waypoints is None:
            rospy.logwarn("No valid points found! Check!")
            return False

	# Set waypoints
        self.next_waypoint = _waypoints
        return True

###############################################################################

    def update_and_publish_waypoints(self):
	# Use look ahead distance to compute the velocity of the waypoints if red light exists
	# If exists, decelerate, and reset the waypoints velocities

        if self.next_waypoint_update():

            total_base_waypoints = len(self.base_waypoints)
            last_base__waypoints = total_base_waypoints-1

            waypoint__index = [_index % total_base_waypoints for _index in \
				range(self.next_waypoint,self.next_waypoint+LOOKAHEAD_WPS)]

            final_waypoints = [self.base_waypoints[_waypoints] for _waypoints in waypoint__index]

            if self.halt_at_red_light:
                self.reset_waypoint_velocities(waypoint__index)
                try:
                    _index_redlight = waypoint__index.index(self._waypoint_redlight)
                    self.decelerate_ego_car(final_waypoints, _index_redlight, self.minimum_stop_margin)
                except ValueError:
                    _index_redlight = None

            if self.end_waypoint_halt_override or self.reset_velocity_waypoint[-1] < 1e-5:
                try:
                    last__waypoints__index = waypoint__index.index(last_base__waypoints)
                    self.decelerate_ego_car(final_waypoints, last__waypoints__index, 0)
                except ValueError:
                    pass

            # Publish
            self.publish_msg(final_waypoints)

###############################################################################

    def publish_msg(self, final_waypoints):
	    """Publish all the messages"""
            waypoint_msg = Lane()
            waypoint_msg.header.seq = self.msg_seq
            waypoint_msg.header.stamp = rospy.Time.now()
            waypoint_msg.header.frame_id = '/world'
            waypoint_msg.waypoints = final_waypoints
            self.final_waypoints_pub.publish(waypoint_msg)
            self.msg_seq += 1

###############################################################################

    def pose_cb(self, msg):
	"""Set the pose"""
        self.current_pose = msg.pose

###############################################################################

    def waypoints_cb(self, msg):
        """
        Set waypoints and unregister
        """
        t = time.time()
        waypoints = msg.waypoints
        total_waypoints = len(waypoints)

        if self.base_waypoints and self.next_waypoint is not None:
            if not self.is_same_waypoint(self.base_waypoints[self.next_waypoint],
                                         waypoints[self.next_waypoint]):
                self.next_waypoint = None
                self.base_waypoints = None
        else:
            pass

        self.reset_velocity_waypoint = [self.get_waypoint_velocity(waypoints, _index) for _index in range(total_waypoints)]


        self.base_waypoints = waypoints

	# Unsubscribe
        if self.unsubscribe_base__waypoints:
            self.base__waypoints_sub.unregister()

###############################################################################

    def decelerate_ego_car(self, waypoints, index_to_full_stop, min_stop_margin):
        """
	Stop at given index, decelerate until that point and  no
        """

        if index_to_full_stop <= 0:
            return

        vel = 0.
        distance = 0.

        threshold_distance = self.distance(waypoints, 0, index_to_full_stop)
        step = threshold_distance / index_to_full_stop


        for _index in reversed(range(len(waypoints))):

            if _index < index_to_full_stop:
                distance += step
                if distance > self.minimum_stop_margin:
                    vel = math.sqrt(2*abs(self.target_acceleration_value)*(distance - min_stop_margin))

            if vel < self.get_waypoint_velocity(waypoints, _index):
                self.set_waypoint_velocity(waypoints, _index, vel)

###############################################################################


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

