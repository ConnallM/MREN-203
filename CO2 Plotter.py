import time
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped, Pose, Point





my_CO2_data = np.array([])
x_vals = np.array([])
y_vals = np.array([])


def CO2_callback(data):
    global my_CO2_data
    CO2_data = np.append(my_CO2_data, [data.data])
    my_CO2_data = CO2_data
    rospy.loginfo('Got the message: ' + str(data))
    #rospy.loginfo(str(my_CO2_data))

def pos_callback(data):
    global x_vals
    global y_vals
    my_x_vals = np.append(x_vals, [data.pose.position.x])
    x_vals = my_x_vals
    my_y_vals = np.append(y_vals, [data.pose.position.y])
    y_vals = my_y_vals
    rospy.loginfo('Got the message: ' + str(data))
    #rospy.loginfo(str(y_vals))
    
rospy.init_node('listener', anonymous=True)
rospy.Subscriber("CO2", Float32, CO2_callback)
rospy.Subscriber("slam_out_pose", PoseStamped, pos_callback)
rospy.spin()

if __name__ == '__main__':
    
    while not rospy.is_shutdown():
        time.sleep(0)
    if len(x_vals) > len(my_CO2_data):
        x_vals = x_vals[0:len(my_CO2_data)]
        y_vals = y_vals[0:len(my_CO2_data)]
    elif len(x_vals) < len(my_CO2_data):
        my_CO2_data = my_CO2_data[0:len(x_vals)]
        
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    # Creating plot
    ax.scatter3D(x_vals, y_vals, my_CO2_data)
    plt.title("simple 3D scatter plot")
     
    # show plot
    plt.show()


"""
ax.contour3D(X, Y, Z, 50, cmap='binary')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('CO2 Concentration (ppm)');
"""

