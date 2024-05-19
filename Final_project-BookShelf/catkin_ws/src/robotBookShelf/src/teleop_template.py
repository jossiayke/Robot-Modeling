#!/usr/bin/env python3

# Teleop Keyboard code was referenced from: https://github.com/ros-teleop/teleop_twist_keyboard
#

from __future__ import print_function

import threading

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped

import sys,termios, tty, select


msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .
For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >
t : up (+z)
b : down (-z)

a : gripper hold
d : gripper release

anything else : stop
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
CTRL-C to quit
"""

moveBindings = {
        'i':(1,0,0,0,0),
        'o':(1,0,0,-1,0),
        'j':(0,0,0,1,0),
        'l':(0,0,0,-1,0),
        'u':(1,0,0,1,0),
        ',':(-1,0,0,0,0),
        '.':(-1,0,0,1,0),
        'm':(-1,0,0,-1,0),
        'O':(1,-1,0,0,0),
        'I':(1,0,0,0,0),
        'J':(0,1,0,0,0),
        'L':(0,-1,0,0,0),
        'U':(1,1,0,0,0),
        '<':(-1,0,0,0,0),
        '>':(-1,-1,0,0,0),
        'M':(-1,1,0,0,0),
        't':(0,0,1,0,0),
        'b':(0,0,-1,0,0),
        'a':(0,0,0,0,1),
        'd':(0,0,0,0,-1)
    }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

speed = 8
turn = 0.4
speed_pinch = 2

def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)
    
# Need 2 scripts
# Publisher script to each controller / teleop (this script)
    # 
# A second subscriber script (subscribes to each joint topic) and publisher
    # subscriber to Xdot
        # set up to find Jacobian once
    # callback function (conversion Xdot and thetadot)
    # publishes to joint topic (makes robot move)


if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('robo_teleop')

    pub_gripper = rospy.Publisher('/robotBookShelf/gripperSlider_Cont/command', Twist, queue_size=10) # Add your topic here between ''. Eg '/book_robot/gripper/command'
    pub_gripperPinch = rospy.Publisher('/robotBookShelf/gripFinger1_Cont/command', Float64, queue_size=10)
    pub_gripperPinch2 = rospy.Publisher('/robotBookShelf/gripFinger2_Cont/command', Float64, queue_size=10)
    
    robot_name = 'robotBookShelf'

    r= rospy.Rate(10)
    
    x = 0   # (xy is flat gripper plane) - moves gripper forward backward
    y = 0   # (xy is flat gripper plane) - moves gripper left right
    z = 0   # move in z world frame - moves gripper up and down
    th = 0  # Rotate gripper about z axis
    p = 0   # gripper pinch moves on plane parallel to xy plane
    status = 0
    count = 0
    acc = 0.1
    target_speed = 0
    target_turn = 0
    control = Twist()
    control_pinchSpeed = 0
    
    try:
        print (msg)
        print (vels(speed,turn))
        while not rospy.is_shutdown():
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
                p = moveBindings[key][4]
                count = 0
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                count = 0

                print (vels(speed,turn))
                if (status == 14):
                    print (msg)
                status = (status + 1) % 15
            elif key == ' ' or key == 'k' :
                x = 0
                y = 0
                z = 0
                th = 0
                p = 0
                control.linear.x = 0
                control.linear.y = 0
                control.linear.z = 0
                control.angular.z = 0
                
            else:
                count = count + 1
                if count > 500:
                    x = 0
                    th = 0
                if (key == '\x03'):
                    break

            target_speed_x = speed * x
            target_speed_y = speed * y
            target_speed_z = speed * z
            target_turn = turn * th
            control_pinchSpeed = speed_pinch * p

            if target_speed_x > control.linear.x:
                control.linear.x = min( target_speed_x, control.linear.x + 2 )
            elif target_speed_x < control.linear.x:
                control.linear.x = max( target_speed_x, control.linear.x - 2 )
            else:
                control.linear.x = target_speed_x
                
            if target_speed_y > control.linear.y:
                control.linear.y = min( target_speed_y, control.linear.y + 2 )
            elif target_speed_y < control.linear.y:
                control.linear.y = max( target_speed_y, control.linear.y - 2 )
            else:
                control.linear.y = target_speed_y
                
            if target_speed_z > control.linear.z:
                control.linear.z = min( target_speed_z, control.linear.z + 2 )
            elif target_speed_z < control.linear.z:
                control.linear.z = max( target_speed_z, control.linear.z - 2 )
            else:
                control.linear.z = target_speed_z
                

            if target_turn > control.angular.z:
                control.angular.z = min( target_turn, control.angular.z + 0.1 )
            elif target_turn < control.angular.z:
                control.angular.z = max( target_turn, control.angular.z - 0.1 )
            else:
                control.angular.z = target_turn
                
            

            # rospy.loginfo("Sending gripper vel command and rotation [{},0,0,0][0,{},0,0][0,0,{},0],[0,0,0,{}] and grip pinch speed {} to {}"
            #               .format(control.linear.x,
            #                       control.linear.y,
            #                       control.linear.z,
            #                       control.angular.z,
            #                       control_pinchSpeed,
            #                       robot_name))
            pub_gripper.publish(control) # publish the turn command.
            pub_gripperPinch.publish(control_pinchSpeed) # publish the turn command.
            pub_gripperPinch2.publish(control_pinchSpeed) # publish the turn command.

            r.sleep()

    except Exception as e:
        print (e)

    finally:
        pub_gripper.publish(control) # publish the turn command.
        pub_gripperPinch.publish(control_pinchSpeed) # publish the turn command.
        pub_gripperPinch2.publish(control_pinchSpeed) # publish the turn command.
        
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)