import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist



class Swarm_subscriber:
  
    def __init__(self):
        
        # rospy.init_node('listener', anonymous=True)
        # initialize the subscriber node now.
        # here we deal with messages of type Twist()
        self.image_sub = rospy.Subscriber("/chatter", 
                                          Int32MultiArray, self.callback)
        print("Initializing the instance!")
        self.message = []
        self.takeoff = 0
        self.takeoff_style = 0
        self.land = 0
        self.land_style = 0
        self.formation_value = 0
        self.shape_value = 0      
        self.mission_value = 0
        self.change_formation_value = 0
        self.formation_rotation_value = 0
        self.formation_value = 0
        self.ugv_number = 3
        
    def callback(self, data):
        
        # now simply display what
        # you've received from the topic
        rospy.loginfo(rospy.get_caller_id() + "The  message is %s",
                      data.data)
        lst = data.data
        lst_int = []
        for i in lst:
            lst_int.append(int(i))
       
        
        self.takeoff = lst_int[0]
        self.takeoff_style = lst_int [1]
        self.land = lst_int[2]
        self.land_style = lst_int [3]
        self.formation_value = lst_int[4]
        self.shape_value = lst_int[5]        
        self.mission_value = lst_int[6]
        self.change_formation_value = lst_int[7]
        self.formation_rotation_value = lst_int[8]

        

        print('Callback executed!')
    
#--------------------------------------------------------------------------------# 