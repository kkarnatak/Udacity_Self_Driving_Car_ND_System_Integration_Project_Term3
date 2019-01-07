#!/usr/bin/env python

###############################################################################

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math

###############################################################################

STATE_COUNT_THRESHOLD = 2

###############################################################################

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.stop_line_positions = []
        self.listener = None

        subscribe_current_pose = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        self.waypoints = rospy.wait_for_message('/base_waypoints',
                                                Lane).waypoints  # Only need to get base_waypoints once
        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb, queue_size=1)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.bridge = CvBridge()

        # Read stop line positions from the config file
        for stop_ln_pos in self.config['stop_line_positions']:
            tl = TrafficLight()
            tl.pose.pose.position.x, tl.pose.pose.position.y, tl.pose.pose.position.z \
                = stop_ln_pos[0], stop_ln_pos[1], 0

            # Append them
            self.stop_line_positions.append(self.get_closest_waypoint(tl.pose.pose.position))

        # Publish next red light
        self.publish_next_red_light = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        # Load the classification model
        print('Traffic Light Classifier Loaded.')
        self.light_classifier = TLClassifier()

        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    ###############################################################################

    def pose_cb(self, msg):
        self.pose = msg

    ###############################################################################

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints

    ###############################################################################

    def traffic_cb(self, msg):
        self.lights = msg.lights

    ###############################################################################

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        if not self.lights or not self.waypoints or not self.pose: pass

        state_msg = ''

        light_waypoint, state = self.process_traffic_lights()
        print("KKLOG", state)

        if state == TrafficLight.UNKNOWN:
            state_msg = 'Unknown'
        elif state == TrafficLight.GREEN:
            state_msg = 'Green: Fire up the gas!'
        elif state == TrafficLight.YELLOW:
            state_msg = 'Yellow: Ok, take chance!'
        elif state == TrafficLight.RED:
            state_msg = 'Red: Full Stop!'

        rospy.loginfo("Traffic Light Color Detected ==> %s", state_msg)

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        # Does the detected state not match the previous state?
        if self.state != state:
            # Reset
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            # Crossing threshold
            self.last_state = self.state
            light_waypoint = light_waypoint if state == TrafficLight.RED else (-light_waypoint)
            self.last_wp = light_waypoint

            self.publish_next_red_light.publish(Int32(light_waypoint))
        else:
            self.publish_next_red_light.publish(Int32(self.last_wp))
        self.state_count += 1

    ###############################################################################

    def get_closest_waypoint(self, position):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints
        """
        return min(xrange(len(self.waypoints)),
                   key=lambda p: self.distance_between_two_points(position,
                                                                  self.waypoints[p].pose.pose.position))

    ###############################################################################

    def projection_to_img_plane(self, point):

        # Get camera parameters

        fx = self.config['camera_info']['focal_length_x']
        fy = self.config['camera_info']['focal_length_y']

        # Get image information

        image_width = self.config['camera_info']['image_width']
        image_height = self.config['camera_info']['image_height']

        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("/base_link",
                                           "/world", now, rospy.Duration(1.0))
            (translation, rotation) = self.listener.lookupTransform("/base_link",
                                                         "/world", now)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Unable to locate camera. Tranformation not possible!")

        # If not an exception, return 0,0
        x = 0
        y = 0

        return (x, y)

    ###############################################################################

    def get_light_state(self, light):
        """Determines the current color of the traffic light

               Args:
                   light (TrafficLight): light to classify

               Returns:
                   int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        if (not self.has_image):
            self.prev_light_loc = None
            return TrafficLight.RED

        self.camera_image.encoding = 'rgb8'
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")

        if (self.listener is not None):
            x, y = self.projection_to_img_plane(light.pose.pose.position)

        return self.light_classifier.get_classification(cv_image)

    ###############################################################################

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closest to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None
        if (self.waypoints is None):
            return -1, TrafficLight.UNKNOWN

        if (self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose.position)

            func = lambda p: self.stop_line_positions[p] - car_position \
                if self.stop_line_positions[p] >= car_position else len(self.waypoints) + \
                            self.stop_line_positions[p] - car_position
            idx = min(xrange(len(self.stop_line_positions)), key=func)
            light_waypoint = self.stop_line_positions[idx]
            light = self.lights[idx]
        if light:
            # Fetch the state and waypoints
            state = light.state
            state = self.get_light_state(light)

            return light_waypoint, state
        return -1, TrafficLight.UNKNOWN

    ###############################################################################

    def distance_between_two_points(self, waypoint_A, waypoint_B):
        return math.sqrt((waypoint_A.x - waypoint_B.x) ** 2
                         + (waypoint_A.y - waypoint_B.y) ** 2
                         + (waypoint_A.z - waypoint_B.z) ** 2)


###############################################################################

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')

