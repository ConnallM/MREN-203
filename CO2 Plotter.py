import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import roslibpy
from std_msgs.msg import Float32MultiArray

my_CO2_data = []
my_pos_data = []

def CO2_callback(data):
    global my_CO2_data
    my_CO2_data = data.data
    rospy.loginfo('Got the message: ' + str(msg))
    
def CO2_listener():
    rospy.init_node('CO2_listener', anonymous=True)
    rospy.Subscriber("CO2", Float32MultiArray, CO2_callback)
    rospy.spin()

def pos_callback(data):
    global my_pos_data
    my_pos_data[0] = data.x
    my_pos_data[1] = data.y
    rospy.loginfo('Got the message: ' + str(msg))
    
def pos_listener():
    rospy.init_node('pos_listener', anonymous=True)
    rospy.Subscriber("pos", Float32MultiArray, pos_callback)
    rospy.spin()


fig = plt.figure()
ax = plt.axes(projection='3d')

X, Y = np.meshgrid(my_pos_data[0], my_pos_data[1])
Z = np.array(my_CO2_data)

fig = plt.figure()
ax = plt.axes(projection='3d')
ax.contour3D(X, Y, Z, 50, cmap='binary')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('CO2 Concentration (ppm)');

