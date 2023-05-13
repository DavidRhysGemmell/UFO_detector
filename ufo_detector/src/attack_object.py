#! /usr/bin/env python3
import rospy
from std_msgs.msg import Float64, Float32, Bool
from geometry_msgs.msg import Twist

class Attack_node:
    def __init__(self):
        rospy.init_node('Attack_node', anonymous=True)  
        rospy.loginfo("Attack node is active")
        self.detec_sub = rospy.Subscriber('UFO_detected', Bool, self.detected_sub)
        self.dist_sub = rospy.Subscriber('/UFO_distance', Float64, self.distance_sub)
        self.ang_sub = rospy.Subscriber('/UFO_angle', Float32, self.angle_sub)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.vel=Twist()
        self.ctrl_c=False
        rospy.on_shutdown(self.shutdown) 
        self.angle_vel=0
        self.linear_vel=0

        #ARGS FOR FINE TUNING
        self.linear_scale_factor = 0.3
        self.angular_scale_factor = 3


    def detected_sub(self, detected):
        self.ufo_detected = detected.data
        if self.ufo_detected == False:
            self.vel.linear.x=0
            self.vel.angular.z=0
            self.pub.publish(self.vel) #no objects = stop
    def distance_sub(self,distance):
        self.ufo_distance=distance.data

    def angle_sub(self,angle):
        self.ufo_angle=angle.data

        self.attack()


    def attack(self):

        if abs(self.ufo_angle)>1:
            self.angle_vel= self.ufo_angle/180
            self.vel.angular.z=self.angle_vel* self.angular_scale_factor

        if abs(self.ufo_angle)>=45:
            self.linear_vel=0
        else:
            self.linear_vel = (45-abs(self.ufo_angle))/45

        self.vel.linear.x=self.linear_vel* self.linear_scale_factor
        self.pub.publish(self.vel)

            



    def shutdown(self):
        self.vel.angular.z=0
        self.vel.linear.x=0
        self.pub.publish(self.vel)
        print("shutting down")

        

if __name__ == '__main__':
    
    rospy.loginfo('Code is running')
    Attack_node()
    rospy.spin()