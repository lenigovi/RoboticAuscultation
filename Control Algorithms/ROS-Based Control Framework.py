#!/usr/bin/env python
# coding: utf-8

# In[ ]:


#!/usr/bin/env python

import rospy
import tensorflow as tf
from std_msgs.msg import String

def audio_publisher():
    rospy.init_node('audio_publisher', anonymous=True)
    pub = rospy.Publisher('audio_topic', String, queue_size=10)
    
    rate = rospy.Rate(1)  # Publishing rate can be adjusted as needed
    
    while not rospy.is_shutdown():
        # Simulation of audio capture
        audio_data = "audio_data"
        pub.publish(audio_data)
        rate.sleep()

if __name__ == '__main__':
    try:
        audio_publisher()
    except rospy.ROSInterruptException:
        pass

def audio_callback(data):


class RobotController:
    def __init__(self):
        self.is_processing = False
        self.model = self.load_model()

    def load_model(self):
        model = tf.keras.models.load_model('your_model_path')
        return model

    def audio_callback(self, data):
        if not self.is_processing:
            rospy.loginfo("Received audio data: %s", data.data)
            
            # Process audio data using the trained model
            processed_result = self.process_audio(data.data)

            # Robot control based on processed audio data
            self.control_robot(processed_result)

            self.is_processing = False 

    def process_audio(self, audio_data):
        input_data = self.preprocess_audio(audio_data)
        prediction = self.model.predict(input_data)
        return prediction[0][0]

    def preprocess_audio(self, audio_data):
        return audio_data

    def control_robot(self, processed_result):
        if processed_result > 0.5:
            rospy.loginfo("Abnormality detected - Take action")
        else:
            rospy.loginfo("Normal audio - Continue normal operation")

def robot_control():
    rospy.init_node('robot_control', anonymous=True)

    controller = RobotController()

    rospy.Subscriber('audio_topic', String, controller.audio_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        robot_control()
    except rospy.ROSInterruptException:
        pass

    print("Received audio data:", data.data)

def robot_control():
    rospy.init_node('robot_control', anonymous=True)
    rospy.Subscriber('audio_topic', String, audio_callback)


class RobotController:
    def __init__(self):
        self.is_processing = False

    def audio_callback(self, data):
        if not self.is_processing:
            rospy.loginfo("Received audio data: %s", data.data)
            
            processed_result = self.process_audio(data.data)

            if processed_result > 0.5:
                rospy.loginfo("Move forward")
            else:
                rospy.loginfo("Stop")

            self.is_processing = False

    def process_audio(self, audio_data):
        return 0.7 

def robot_control():
    rospy.init_node('robot_control', anonymous=True)

    controller = RobotController()

    rospy.Subscriber('audio_topic', String, controller.audio_callback)

    rospy.spin()


if __name__ == '__main__':
    try:
        robot_control()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()

if __name__ == '__main__':
    try:
        robot_control()
    except rospy.ROSInterruptException:
        pass


    
<launch>
  <node name="audio_publisher" type="audio_publisher_node.py" output="screen"/>
  <node name="robot_control" type="robot_control_node.py" output="screen"/>
</launch>

roslaunch auscultation_control robot_auscultation.launch

