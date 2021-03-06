from styx_msgs.msg import TrafficLight

import tensorflow as tf
import numpy as np
import time
from scipy.stats import norm, mode

import os
import rospy


class TLClassifier(object):
    def __init__(self):
        
        GRAPH_FILE = 'frozen_inference_graph.pb'

        model_path = os.path.dirname(os.path.realpath(__file__))

        model_path = os.path.join(model_path, GRAPH_FILE)
        rospy.loginfo("model_path={}".format(model_path))
        
        self.detection_graph = self.load_graph(model_path)
        rospy.loginfo("model loaded")

        # `get_tensor_by_name` returns the Tensor with the associated name in the Graph.
        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')

        # Each box represents a part of the image where a particular object was detected.
        self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')

        # Each score represent how level of confidence for each of the objects.
        # Score is shown on the result image, together with the class label.
        self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')

        # The classification of the object (integer id).
        self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')


    def filter_boxes(self, min_score, boxes, scores, classes):
        """Return boxes with a confidence >= `min_score`"""
        n = len(classes)
        # print('number of classes = %d' % n)
        idxs = []
        for i in range(n):
            if scores[i] >= min_score:
                idxs.append(i)
    
        filtered_boxes = boxes[idxs, ...]
        filtered_scores = scores[idxs, ...]
        filtered_classes = classes[idxs, ...]
        return filtered_boxes, filtered_scores, filtered_classes

    def load_graph(self, graph_file):
        """Loads a frozen inference graph"""
        graph = tf.Graph()
        with graph.as_default():
            od_graph_def = tf.GraphDef()
            #od_graph_def = tf.compat.v1.GraphDef()
            with tf.gfile.GFile(graph_file, 'rb') as fid:
            #with tf.io.gfile.GFile(graph_file, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
        return graph

    def to_string(self, state):
        out = "unknown"
        if state == TrafficLight.GREEN:
            out = "green"
        elif state == TrafficLight.YELLOW:
            out = "yellow"
        elif state == TrafficLight.RED:
            out = "red"
        return out

    def get_classification(self, image, tag):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        # tag = "{:.0f}".format(time.time())[-3:]

        rospy.loginfo(str("calling classifier on light - [%s]" % tag))
        start = time.time()
        image_np = np.expand_dims(np.asarray(image, dtype=np.uint8), 0)

        with tf.Session(graph=self.detection_graph) as sess:                
            # Actual detection.

            (boxes, scores, classes) = sess.run([self.detection_boxes, self.detection_scores, self.detection_classes], 
                                                feed_dict={self.image_tensor: image_np})

            # Remove unnecessary dimensions
            boxes = np.squeeze(boxes)
            scores = np.squeeze(scores)
            classes = np.squeeze(classes)


            confidence_cutoff = 0.6
            # Filter boxes with a confidence score less than `confidence_cutoff`
            boxes, scores, classes = self.filter_boxes(confidence_cutoff, boxes, scores, classes)

            options = [TrafficLight.GREEN, TrafficLight.RED, TrafficLight.YELLOW, TrafficLight.UNKNOWN]

            if len(classes) != 0:
                result = options[int(mode(classes)[0][0])-1]

                # colors = [red, yellow, green, unknown]    
                #rospy.loginfo("upcoming light={}".format(self.to_string(result), ))
                rospy.loginfo(str('upcoming light classied as %s in %.3f s  - [%s]' % (self.to_string(result), time.time()-start, tag)))

                return result
            else:
                rospy.loginfo(str("unable to classify - [%s]" % tag))
                

        return TrafficLight.UNKNOWN
