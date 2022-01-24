#!/user/bin/env python3
import rospy
import numpy as np
import math

from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose


####publisher
pub = rospy.Publisher('/gazebo/link_states', LinkStates, queue_size=1000)
pub=rospy.Publisher('chatter',Pose,queue_size=1000)


####transformation matrix
def h_matrix(theta,a,alpha,d):
    A=np.array([[math.cos(theta),(-math.sin(theta))*math.cos(alpha),math.sin(theta)*math.sin(alpha),a*math.cos(theta)],
		[math.sin(theta),math.cos(theta)*math.cos(alpha),(-math.cos(theta))*math.sin(alpha),a*math.sin(theta)],
		[0,math.sin(alpha),math.cos(alpha),d],
		[0,0,0,1]]
	      )
    return A


def callback(data):
    #link length
    l1=1;
    l2=1;
    l3=1;
    
    #####get joint value
    ####joint 1
    LinkName=data.name.index('rrbot::link2') #x2,y2
    x_joint1=data.pose[LinkName].position.x
    y_joint1=data.pose[LinkName].position.y
    theta1=math.atan(y_joint1/x_joint1);
    
    ####joint 2
    LinkName=data.name.index('rrbot::link3') #x3,y3,z3
    x_joint2=data.pose[LinkName].position.x
    y_joint2=data.pose[LinkName].position.y
    z_joint2=data.pose[LinkName].position.z
    theta2=math.atan(((y_joint2-y_joint1)/(x_joint2-x_joint1)));
    
    ####joint 3
    LinkName=data.name.index('rrbot::link4') #z4
    z_joint3=data.pose[LinkName].position.z
    theta3=z_joint2-z_joint3
    
    #####compute transformation matrix
    A1=h_matrix(theta1,l1,0,l1)
    A2=h_matrix(theta2,l2,0,0)
    A3=h_matrix(0,0,0,theta3)
    
    
    T =A1.dot(A2)
    new_pose=T.dot(A3)

    #print(new_pose)

    
    #####get position
    eePose=Pose();
    eePose.position.x=new_pose[0, 3]
    eePose.position.y=new_pose[1, 3]
    eePose.position.z=new_pose[2, 3]
    
    #####get quaternion
    RotationMatrix=new_pose[0:3, 0:3]
    #print("matrix=",RotationMatrix)
    M1=RotationMatrix
    
    r = np.math.sqrt(float(1)+M1[0,0]+M1[1,1]+M1[2,2])*0.5
    i = (M1[2,1]-M1[1,2])/(4*r)
    j = (M1[0,2]-M1[2,0])/(4*r) 
    k = (M1[1,0]-M1[0,1])/(4*r)
    #print('r=',r)
    #print('i=',i)
    #print('j=',j)
    #print('k=',k)
    
    eePose.orientation.x=i
    eePose.orientation.y=j
    eePose.orientation.z=k
    eePose.orientation.w=r
    
    #####publish pose
    pub.publish(eePose)
   
def part2_gazebo():
    rospy.init_node('part2_gazebo', anonymous=True)
    rospy.Subscriber('/gazebo/link_states', LinkStates, callback)
    rospy.spin()


if __name__=='__main__':
    part2_gazebo()

