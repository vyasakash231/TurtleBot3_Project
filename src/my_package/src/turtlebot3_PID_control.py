#!/usr/bin/python3
import rospy
from tf import transformations as T
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt, degrees, pi, cos, sin
import numpy as np
import matplotlib.pyplot as plt
"""
To run this code:
on first terminal run: roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
on second terminal run: rosrun my_package turtlebot3_PD_control.py _goal_x:=2 _goal_y:=5
(to rest the robot) on third terminal run: rosservice call /gazebo/reset_simulation
"""
class Turtle_move:
    def __init__(self):
        # Initiate/Creates a node with name 'my_turtlebot'
        rospy.init_node('my_turtlebot',anonymous = False)

        # Publisher which will publish to the topic '/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/cmd_vel',Twist, queue_size=10)

        # A subscriber to the topic '/turtle1/pose'
        self.odom_subscriber = rospy.Subscriber('/odom',Odometry, self.update_odometry) # self.update_pose is called when a message of type Pose is received

        # Add the following lines to initialize current_position_x, current_position_y and current_theta
        self.current_position_x = 0.0
        self.current_position_y = 0.0
        self.current_theta = 0
        self.current_linear_vel = 0
        self.current_angular_vel = 0

        self.break_even = 0
        self.error_prev = 0
        self.integral = 0
        self.windup_guard = 1

        self.rate = rospy.Rate(10) # 10 Hz
        self.dt = 0.1  # 1/rate = 0.1

        # for plotting
        self.time_data = []
        self.start_time = rospy.Time.now().to_sec()

    def update_odometry(self, data):
        # Callback function which is called when 'data' argument feed to it when a new message of type Odometry is received by the subscriber
        self.current_position_x = round(data.pose.pose.position.x, 4)
        self.current_position_y = round(data.pose.pose.position.y, 4)
        Quaternion = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
        (roll, pitch, yaw) = T.euler_from_quaternion(Quaternion)
        self.current_theta = yaw

        # Extract linear and angular velocities from Odometry message (considerring Unicycle model)
        self.current_linear_vel = round(data.twist.twist.linear.x, 4)
        self.current_angular_vel = round(data.twist.twist.angular.z, 4)

    # function to calculate distance btw goal and current position
    def euclidean_distance(self):
        return sqrt(pow((self.goal_odometry_y - self.current_position_y), 2) + pow((self.goal_odometry_x - self.current_position_x), 2))
            
    # function to calculate angle btw turtle and goal position
    def steering_angle(self):
        theta = atan2(self.goal_odometry_y - self.current_position_y, self.goal_odometry_x - self.current_position_x)
        
        # converting steering angle into rotational matrix
        R_of_steering_wrt_world = np.array([[np.cos(theta), -np.sin(theta),  0],
                                            [np.sin(theta),  np.cos(theta),  0],
                                            [      0      ,        0      ,  1]])   
        return R_of_steering_wrt_world
    
    def plot_data(self):
        plt.ion()  # Enable interactive mode
        plt.figure()

        # plot 1:
        plt.subplot(1, 2, 1)
        plt.plot(self.time_data,lin_velocity,label ='Linear')
        #plt.plot(self.time_data,ang_velocity,label ='Angular')
        plt.xlabel('Time (s)')
        plt.ylabel('Linear Velocity (m/s)')
        plt.legend()

        # plot 2:
        plt.subplot(1, 2, 2)
        plt.plot(x_cord,y_cord,label ='Position')
        plt.plot(0,0,"o")
        plt.plot(self.goal_odometry_x,self.goal_odometry_y,"*")
        plt.xlabel('X-axis')
        plt.ylabel('Y-axis')
        plt.legend()

        plt.suptitle('PID Control')
        plt.show(block=True)  # Add block=True to wait for the plot window to be closed

    def move2goal(self):
        x_cord.append(self.current_position_x)
        y_cord.append(self.current_position_y)
    
        # ROS Twist msg generated is assigned a new variable 'vel_msg'
        vel_msg = Twist()
        while self.euclidean_distance() >= 0.05:
            current_time = rospy.Time.now().to_sec() - self.start_time

            error = self.euclidean_distance()
            self.integral += error * self.dt
            self.integral = max(min(self.integral, self.windup_guard), -self.windup_guard)
            derivative = (error - self.error_prev) / self.dt

            # Linear velocity in the x-axis.
            vel_msg.linear.x = 0.1*error + 0.05*derivative + 0.025*self.integral  # kp * e + kd * de/dt + ki * int(e*dt)
            vel_msg.linear.y = 0 
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            
            # converting turtle angle into rotational matrix
            R_of_robot_wrt_world = np.array([[np.cos(self.current_theta), -np.sin(self.current_theta),  0],
                                             [np.sin(self.current_theta),  np.cos(self.current_theta),  0],
                                             [              0           ,             0              ,  1]]) 
            
            R_of_steering_wrt_robot = self.steering_angle() @ np.transpose(R_of_robot_wrt_world)  
            vel_msg.angular.z = 2.5*atan2(R_of_steering_wrt_robot[1,0],R_of_steering_wrt_robot[0,0])

            rospy.loginfo('position Error:{}'.format(round(self.euclidean_distance(), 4)))

            # Collect data for plotting
            self.time_data.append(current_time)
            lin_velocity.append(vel_msg.linear.x) # store velocity as list
            ang_velocity.append(vel_msg.angular.z) # store velocity as list
            
            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            self.error_prev = error

            x_cord.append(self.current_position_x)
            y_cord.append(self.current_position_y)

            # Publish at the desired rate.
            self.rate.sleep()

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        rospy.loginfo("Goal reached!")

        self.plot_data()

        # Keep the program running, If we press control + C, the node will stop.
        rospy.spin()

        # while not rospy.is_shutdown():
        #     self.rate.sleep()
"""
 when this file is being executted no class or function run directly the only condition which run first is the line which has 0 indentation except function 
 and class so only if __name__ == '__main__' is left which will be executted first and that will call turtle_move() class and move2goal() method.
"""

if __name__ == '__main__': # this condition get satisfied when turtle_position_control.py is being run directly in terminal and not imported in some other file and then executted
    
    # Using Error & Exception Handling with Try/Except
    try:
        x = Turtle_move() # creating an object of turtle_move() class
        """
        If the parameters goal_x and goal_y are not provided via the launch file 
        or as parameters when using 'rosrun My_turtlebot3 turtlebot3_goal.py _goal_x:=2 _goal_y:=3',
        they will default to 1.0.
        """
        # ROS Odometry msg generated is assigned a new variable 'goal_odometry'
        x.goal_odometry_x = float(rospy.get_param('~goal_x', 1.0)) # 1.0 is default values here
        x.goal_odometry_y = float(rospy.get_param('~goal_y', 1.0)) # 1.0 is default values here

        E = 0.1
        M = 0.5
        
        lin_velocity, ang_velocity = [], []
        x_cord, y_cord = [], []

        x.move2goal()

    except rospy.ROSInterruptException:
        pass
