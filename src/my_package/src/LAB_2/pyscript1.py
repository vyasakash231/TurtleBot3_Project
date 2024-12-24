from robot_control_class import RobotControl

rc = RobotControl()

# a = rc.get_laser(180)

# print ("The distance measured is: ", a, " m.")

range = rc.get_laser_full()

idx = [0, 10, 20, 50, 100]
b = [range[i] for i in idx]

print(range,'\n',b)
