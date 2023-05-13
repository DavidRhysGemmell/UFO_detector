#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64, Float32, Bool

class UFO_detector:
    def __init__(self):
        rospy.init_node('UFO_detector_node', anonymous=True) 
        rospy.loginfo("UFO Detector node is active")


################ EDIT-ABLE STUFF #################################
        #LIDAR SUB   
        self.sub = rospy.Subscriber('/scan', LaserScan, self.detector) #REPLACE /scan WITH LIDAR SCAN TOPIC
        ##
        #ARGS FOR TUNING
        self.object_detection_sensitivity=0.5
        self.bank_size = 40 #PUT AS LIDAR HZ. how many messages to determine relative object velocity

##################################################################
        #OTHER STUFF
        self.distance_bank = []
        self.distance_bank=[0 for i in range(self.bank_size)]
        self.distance_change_bank=[0,0,0,0]
        self.angle_bank = []
        self.angle_bank = [0 for i in range(self.bank_size)]

        self.angle_change_bank = [0,0,0,0]
        #PUBS
        self.distance_pub = rospy.Publisher('/UFO_distance', Float64, queue_size=10)
        self.angle_pub = rospy.Publisher('/UFO_angle', Float32, queue_size=10)
        self.detector_pub=rospy.Publisher('/UFO_detected', Bool, queue_size=10)
        

    def detector(self, LaserMsg):

        laser_scan_array=LaserMsg.ranges
        laser_scan_array_size=len(laser_scan_array)
        #print(laser_scan_array)
        width_in_scans=0
        UFO_detected = False
        for i in range(laser_scan_array_size-1): #not accounting for 720->0 yet
                #print([i,laser_scan_array[i]])
                k=i+1
                k_scan_distance=laser_scan_array[k]
                while k_scan_distance == float('inf'):
                    k=k+1       
                    if k== laser_scan_array_size:
                        k=0
                    k_scan_distance=laser_scan_array[k]
                #print([i,k,laser_scan_array[i], laser_scan_array[k]])
                if laser_scan_array[i]-laser_scan_array[k] > self.object_detection_sensitivity and laser_scan_array[i]!=float('inf'): #and laser_scan_array[i]!= float('inf') and laser_scan_array[i+1]!=float('inf'): #object begin
                    object_begin=i
                    j=i
                    if j==laser_scan_array_size-1:
                        j=0
                    stop_stuck_in_while = 0
                    
                    while laser_scan_array[j]-laser_scan_array[j+1] >=-self.object_detection_sensitivity and stop_stuck_in_while<=719: #until object end
                        
                        width_in_scans=width_in_scans+1
                        stop_stuck_in_while=stop_stuck_in_while+1
                        j=j+1
                        if j==laser_scan_array_size-1:
                            j=0
                    UFO_detected=True
                    object_end = j
                    object_width= width_in_scans
                    object_middle_scan=object_begin+object_width/2
                    if object_middle_scan>=laser_scan_array_size: #loop around from maximum array to zero
                        object_middle_scan=object_middle_scan-laser_scan_array_size

                    #remap from len(scan array) to 0 to 360
                    degrees_object_middle_scan= (object_middle_scan/laser_scan_array_size)*360
                    #remap from (0,360) to (0,180,-180,0)
                    self.object_middle = degrees_object_middle_scan
                    if self.object_middle >180:
                        self.object_middle = self.object_middle-360       



                    rospy.loginfo("UFO detected at %s", self.object_middle)
                    self.distance=laser_scan_array[int(object_middle_scan)]

                    self.distance_pub.publish(self.distance)

                    self.angle_pub.publish(self.object_middle) #real angle in deg, /scan message is filled anti-clockwise. consusing eh.
                    #self.velocity()
                if UFO_detected==True:
                    break        
        if UFO_detected==False:
            rospy.loginfo("No UFOs detected")
        self.detector_pub.publish(UFO_detected)
        



    def velocity(self):
        for i in range (self.bank_size-1):   
            self.distance_bank[i]=self.distance_bank[i+1] # move all distance values up one in the list, removing the earliest scan. Most recent is at the end of list
            self.angle_bank[i]=self.angle_bank[i+1]

        self.distance_bank[self.bank_size-1] = self.distance
        self.angle_bank[self.bank_size-1] = self.object_middle


        quarter_second=self.bank_size/4
        #print(self.angle_bank)
        self.angle_change_bank[3]=self.angle_bank[3*int(quarter_second)]-self.angle_bank[self.bank_size-1]
        self.distance_change_bank[3]=self.distance_bank[3*int(quarter_second)]-self.distance_bank[self.bank_size-1]
        for i in range(3):
            self.angle_change_bank[i]=self.angle_bank[i*int(quarter_second)]-self.angle_bank[(i+1)*int(quarter_second)]
            self.distance_change_bank[i]=self.distance_bank[i*int(quarter_second)]-self.distance_bank[(i+1)*int(quarter_second)]
        print(self.angle_change_bank)




    
if __name__ == '__main__':
    
    rospy.loginfo('Code is running')
    UFO_detector()
    rospy.spin()