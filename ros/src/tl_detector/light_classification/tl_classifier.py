from styx_msgs.msg import TrafficLight
import cv2
from keras.models import load_model
from numpy import zeros, newaxis
import rospkg
import numpy as np
import tensorflow as tf 
class TLClassifier(object):
    def __init__(self):
        
        self.image_width = 224
	self.image_height = 224
	ros_root = rospkg.get_ros_root()

	r = rospkg.RosPack()
	path = r.get_path('tl_detector')
	print(path)
        self.model = load_model(path + '/tl_classifier_sim_try.h5') 

	self.model._make_predict_function()
        self.graph = tf.get_default_graph()
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
	# Resize
        bgr_image = cv2.resize(image, (self.image_width, self.image_height)) 

	#cv2.imwrite("kk.jpg", bgr_image)
	
	rgb_image = cv2.cvtColor(bgr_image,cv2.COLOR_BGR2RGB)
	#cv2.imwrite("kk1.jpg", rgb_image)
	#print("KKLOG:", _image.shape)

	# Normalization
	rgb_image = rgb_image / 255.;

        with self.graph.as_default():
            predictions = self.model.predict(rgb_image.reshape((1, self.image_height, self.image_width, 3)))
            color = predictions[0].tolist().index(np.max(predictions[0]))
	    print('COLOR Predicted:' ,color)
            tl = TrafficLight()
            tl.state = color
            if(color == 0):
        	return TrafficLight.RED
	    elif(color == 1):
	        return TrafficLight.YELLOW
	    elif(color == 2):
	       return TrafficLight.GREEN

        return TrafficLight.UNKNOWN
