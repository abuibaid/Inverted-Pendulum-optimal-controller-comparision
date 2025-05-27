import rospy
import numpy as np
# import actionlib
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from scipy.signal import butter, lfilter, freqz


class RosCommunication():
    def __init__(self):
        # Creat Publisher
        self.pub_controller_command = rospy.Publisher('/cart_controller/command', Float64 ,queue_size=10)

        # Create message
        self.msg = Float64()

        # Create subscriber
        self.sub_sensor_reading = rospy.Subscriber('/joint_states',JointState)

        # Lowpass filter requirements.
        self.order = 6
        self.fs = 30.0       # sample rate, Hz
        self.cutoff = 3.667  # desired cutoff frequency of the filter, Hz

    def receiveState(self):
        #print(self.param.ROS['robotNr']+self.param.ROS['subscriber'])
        sensor = rospy.wait_for_message('/joint_states',JointState,timeout = 5)
        position = np.array(sensor.position).T
        velocity = np.array(sensor.velocity).T
        # effort = np.array(sensor.actual.effort).T
        state = np.concatenate([position.reshape(-1,1),velocity.reshape(-1,1)],axis = 0)

        return state-self.butter_lowpass_filter(state)

    def butter_lowpass(self,cutoff, fs, order=5):
        return butter(order, self.cutoff, fs=fs, btype='low', analog=False)

    def butter_lowpass_filter(self , data, order=5):
        b, a = self.butter_lowpass(self.cutoff, self.fs, order=order)
        y = lfilter(b, a, data)
        return y
    
    def sendingCommand(self,F):

        # Adding data      
        self.msg.data = F
        
        # print(type(msg), type(msg.data), msg.data)
        
        #Publish message
        self.pub_controller_command.publish(self.msg)


    