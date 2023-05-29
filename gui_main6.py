import tkinter
import tkinter.messagebox
import customtkinter
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image # kamera 
import yaml
import numpy as np
import os
import subprocess

class MyTabView(customtkinter.CTkTabview):
    def __init__(self, master, **kwargs):
        super().__init__(master, **kwargs)

        # create tabs
        self.add("Mission Planner")
        self.add("Swarm")
        self.add('Data Transfer')
        # add widgets on tabs
        self.label = customtkinter.CTkLabel(master=self.tab("Mission Planner"))
        self.label.grid(row=0, column=0, padx=20, pady=10)
        
class GUI (customtkinter.CTk):
    
    def __init__(self):
        super().__init__()
        

        rospy.init_node('talker', anonymous=True)
        customtkinter.set_appearance_mode("dark")
        customtkinter.set_default_color_theme("green")
        
        with open('/home/ozkan/crazyswarm/ros_ws/src/crazyswarm/launch/crazyflies.yaml','r') as file:
            self.file_info = yaml.full_load(file)
        
        print(self.file_info)
        
        self.geometry("1280x960")
        self.pub = rospy.Publisher('chatter', Int32MultiArray, queue_size=10)
        self.pub2 = rospy.Publisher('chatter2', Point, queue_size=10)
        self.pub3 = rospy.Publisher('chatter3', Float32MultiArray, queue_size=10)
        self.title("UAV Control System GUI")
        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(0,weight=1)
        
        # self.rate = rospy.Rate(10)  # 10hz
        self.agent_list3=[]
        self.land = 1
        self.takeoff = 0
        self.formation_status = 0
        self.shape = -1
        self.mission = 0
        self.land_style = 0
        self.takeoff_style = 0
        self.change_formation = 0
        self.formation_rotation = 0
        self.refresh_flight = 0
        self.ugv_number = 0
        self.data = Int32MultiArray()
        self.Posdata = Point()
        self.data.data = [self.takeoff, self.takeoff_style, self.land, self.land_style, self.formation_status, self.shape, self.mission, self.change_formation, self.formation_rotation, self.ugv_number]    
        self.PositionList = [0, 0, 0]
        self.floatdata = Float32MultiArray()
        
        self.tab_view=MyTabView(master=self)
        self.tab_view.grid(row=0, column=0, padx=50, pady=50)

        # GUI Init

        self.frame = customtkinter.CTkFrame(master=self.tab_view.tab("Mission Planner"), width=900, height=640, corner_radius=0)
        self.frame.grid(row=0, column=0, sticky="nsew")
        self.frame4 = customtkinter.CTkFrame(master=self.tab_view.tab("Mission Planner"), width=300, height=300, corner_radius=0)
        self.frame.grid(row=0, column=4, sticky="nsew")       
        self.frame2 = customtkinter.CTkFrame(master=self.tab_view.tab("Swarm"), width=900, height=640, corner_radius=0)
        self.frame2.grid(row=1, column=2, sticky="nsew")
        self.frame3 = customtkinter.CTkFrame(master=self.tab_view.tab("Data Transfer"), width=900, height=640, corner_radius=0)
        self.frame3.grid(row=1, column=2, sticky="nsew")
        
        self.label = customtkinter.CTkLabel(master=self.frame, text="Mission Planner Panel",font=customtkinter.CTkFont(size=15, weight="bold"))
        self.label.grid(row=1, column=2, padx=20, pady=10)
        
        #-------------TAKE OFF 
        self.combo_box_takeOff=customtkinter.CTkComboBox(self.frame,values=["Take off style","All","Desired location"])
        self.combo_box_takeOff.grid(row=7, column=0,columnspan=1, padx=20, pady=20)
        
        self.radio_var = tkinter.IntVar(value=0)
        self.button1 = customtkinter.CTkRadioButton(
            master=self.frame, text="Take off", variable=self.radio_var,command=self.take_off, value=1)
        self.button1.grid(column=0, row=8, padx=20, pady=20)

        #-------------LANDING 
        
        self.combo_box_landing=customtkinter.CTkComboBox(self.frame,values=["Landing style","All","One by one","Desired location"])
        self.combo_box_landing.grid(row=7, column=1,columnspan=1, padx=20, pady=20)
        self.button2 = customtkinter.CTkRadioButton(
            master=self.frame, text="Landing", variable=self.radio_var,command=self.landing, value=0)
        self.button2.grid(column=1, row=8,padx=20, pady=20)
        
        
        #------------ REFLESH FLIGHT
        self.button5 = customtkinter.CTkRadioButton(
            master=self.frame,text="Refresh Flight", variable=self.radio_var, command=self.refreshFlight, value = 0)
        self.button5.grid(column=2, row=5,padx=20, pady=20)
        
        self.buttonFormation = customtkinter.CTkButton(
            master=self.frame,text="Formation Flight", command=self.startFormation)
        self.buttonFormation.grid(column=3, row=5,padx=20, pady=20)
        
        self.buttonFire = customtkinter.CTkButton(
            master=self.frame,text="Fire Fighter", command=self.startFire)
        self.buttonFire.grid(column=4, row=5,padx=20, pady=20)

        self.buttonObstacle = customtkinter.CTkButton(
            master=self.frame,text="Formation Flight", command=self.startObstacle)
        self.buttonObstacle.grid(column=3, row=6,padx=20, pady=20)
                
                
        
        #--SIMULATION BUTTON
        self.button13 = customtkinter.CTkButton(
            master=self.frame,text="Simulation Flight", command=self.simulationFlight)
        self.button13.grid(column=2, row=9,padx=20, pady=20)
        
        self.button14 = customtkinter.CTkButton(
            master=self.frame,text="Real-Time Flight", command=self.realTimeFlight)
        self.button14.grid(column=3, row=9,padx=20, pady=20)
        
        #--CAMERA BUTTON
        self.button16 = customtkinter.CTkButton(
            master=self.frame,text="Connect to Camera", command=self.camera)
        self.button16.grid(column=4, row=9,padx=20, pady=20)        
        
        # CAMERA VISUALIZATION
        
        self.canvas=customtkinter.CTkCanvas(self.frame4,width= 250, height= 250)
        self.canvas.grid(column=0, row=0)
        # self.canvas.create_image(10,10,anchor=NW,image=img)        
        
        # MISSION SELECTION
        self.combo_box4=customtkinter.CTkComboBox(self.frame,values=["Mission Selection","Fire Fighter","Formation Flight","Obstacle Avoidance"])
        self.combo_box4.grid(row=5, column=1,columnspan=1, padx=20, pady=20)
        self.button6 = customtkinter.CTkButton(
            master=self.frame, text="Mission Select", command=self.mission_select)
        self.button6.grid(column=1, row=6,padx=20, pady=20)
        
        # SHAPE SELECTION  
        self.combo_box3=customtkinter.CTkComboBox(self.frame,values=["Shape Selection", 'Random(N UAVs)', 'Triangle(3 UAVs)', 'Square(4 UAVs)', 'V_shape(5 UAVs)', 'Crescent(6 UAVs)', 'Pyramid (5 UAVs)', 'Cube (8 UAVs)', 'Triangle Prisma (6 UAVs)', 'Star(10 UAVs)', 'Pentagon Prisma (10 UAVs)', 'Hexagon Prisma (12 UAVs)', 'Cylinder (10 UAVs)', 'Pentagon (5 UAVs)', 'Hexagon (6 UAVs)'])
        self.combo_box3.grid(row=5, column=0,columnspan=1, padx=20, pady=20)          
        self.button7 = customtkinter.CTkButton(
            master=self.frame, text="Shape Select", command=self.shape_select)
        self.button7.grid(column=0, row=6,padx=10, pady=10)

        # Change Formation 
        self.button3 = customtkinter.CTkRadioButton(
            master=self.frame, text="Chance Formation", variable=self.radio_var,command=self.formation_chance, value=0)
        self.button3.grid(column=2, row=8,padx=20, pady=20)

        # Rotate Formation
        self.button4 = customtkinter.CTkRadioButton(
            master=self.frame, text="Rotate Formation", variable=self.radio_var,command=self.rotate_formation, value=0)
        self.button4.grid(column=2, row=7,padx=20, pady=20)
        
        # TAB 2- --- SWARM----
        
        self.label = customtkinter.CTkLabel(master=self.frame2, text="Swarm Control Panel",font=customtkinter.CTkFont(size=15, weight="bold"))
        self.label.grid(row=1, column=1, padx=20, pady=10)        
        self.label1 = customtkinter.CTkLabel(master=self.frame2, text="Agent Channel",font=customtkinter.CTkFont(size=12, weight="bold"))
        self.label1.grid(row=4, column=0, padx=20, pady=10)
        self.label2 = customtkinter.CTkLabel(master=self.frame2, text="Agent ID",font=customtkinter.CTkFont(size=12, weight="bold"))
        self.label2.grid(row=3, column=0, padx=20, pady=10)
                # To add positions of agents in the simulation
        self.label3 = customtkinter.CTkLabel(
            master=self.frame2, text="Agent Initial Position", font=customtkinter.CTkFont(size=12, weight="bold"))
        self.label3.grid(row=5, column=0, padx=20, pady=10)
        
        self.entry_box3 = customtkinter.CTkEntry(
            master=self.frame2, placeholder_text="initialPosition in float")
        self.entry_box3.grid(row=5, column=1, columnspan=1,
                             padx=20, pady=20, sticky="nsew")

        self.entry_box1 = customtkinter.CTkEntry(
            master=self.frame2, placeholder_text="Channel number")
        self.entry_box1.grid(row=4, column=1, columnspan=1, padx=20, pady=20, sticky="nsew")
        self.entry_box2 = customtkinter.CTkEntry(
            master=self.frame2, placeholder_text="ID number in decimal")
        self.entry_box2.grid(row=3, column=1, columnspan=1, padx=20, pady=20, sticky="nsew")
        self.button9 = customtkinter.CTkButton(
            master=self.frame2, text="Add agent", command=self.add_agent_list)
        self.button9.grid(column=0, row=6,padx=10, pady=10)
        self.button10 = customtkinter.CTkButton(
            master=self.frame2, text="Discard agent", command=self.delete_agent_list)
        self.button10.grid(column=2, row=6,padx=10, pady=10)        
        
        self.textbox1 = customtkinter.CTkTextbox(master=self.frame2)
        self.textbox1.grid(row=7, column=0, columnspan=2)

        self.agent_list= "agent list"
        self.textbox1.insert("0.0", self.agent_list)  # insert at line 0 character 0
        # self.text = self.textbox.get("0.0", "end")  # get text from line 0 character 0 till the end
        # self.textbox1.delete("0.0", "end")  # delete all text
        # self.textbox1.configure(state="disabled")  # configure textbox to be read-only       
         
        self.button12 = customtkinter.CTkButton(
            master=self.frame2, text="show agent list", command=self.show_agent_list)
        
        self.button12.grid(column=1, row=6, padx=20, pady=20) 
        
        self.entry_box7 = customtkinter.CTkEntry(
            master=self.frame2, placeholder_text="Number of UGV",font=('Arial bold',16),placeholder_text_color="white",border_color="black",fg_color='red')
        self.entry_box7.grid(row=7, column=2, columnspan=2,
                             padx=20, pady=20, sticky="nsew")        
        
                  

        # TAB 3- --- DATA SENDER----
        
        self.label = customtkinter.CTkLabel(master=self.frame3, text="Data Transfer Panel",font=customtkinter.CTkFont(size=15, weight="bold"))
        self.label.grid(row=1, column=1, padx=20, pady=10)
        self.entry_box4 = customtkinter.CTkEntry(
            master=self.frame3, placeholder_text="Send X Position")
        self.entry_box4.grid(row=4, column=0, columnspan=1, padx=20, pady=20, sticky="nsew")
        self.entry_box5 = customtkinter.CTkEntry(
            master=self.frame3, placeholder_text="Send Y Position")
        self.entry_box5.grid(row=4, column=1, columnspan=1, padx=20, pady=20, sticky="nsew")
        self.entry_box6 = customtkinter.CTkEntry(
            master=self.frame3, placeholder_text="Send Z Position")
        self.entry_box6.grid(row=4, column=2, columnspan=1, padx=20, pady=20, sticky="nsew")
        self.button8 = customtkinter.CTkButton(
            master=self.frame3, text="Save Position", command=self.savePosition)
        self.button8.grid(column=1, row=5,padx=10, pady=10)
        self.button17 = customtkinter.CTkButton(
            master=self.frame3, text="Send Position", command=self.sendPosition)
        self.button17.grid(column=2, row=5,padx=10, pady=10)
        
        self.label = customtkinter.CTkLabel(master=self.frame3, text="Select agent to data transfer",font=customtkinter.CTkFont(size=12, weight="bold"))
        self.label.grid(row=2, column=0, padx=20, pady=10)        
        self.combo_box5=customtkinter.CTkComboBox(self.frame3,values=["Agent selection"])
        self.combo_box5.grid(row=2, column=1,columnspan=1, padx=20, pady=20)        
        self.button11 = customtkinter.CTkButton(
            master=self.frame3, text="select agent", command=self.select_agent)
        
        self.button11.grid(column=2, row=2, padx=20, pady=20)

        self.textbox2 = customtkinter.CTkTextbox(master=self.frame3) 
        self.textbox2.grid(row=7, column=0)
        self.textbox3 = customtkinter.CTkTextbox(master=self.frame3) 
        self.textbox3.grid(row=7, column=2)        
        
        
        self.textbox2.insert("0.0", self.agent_list)
        self.button15 = customtkinter.CTkButton(
            master=self.frame3, text="show agent list", command=self.show_agent_list)
        
        self.button15.grid(column=1, row=7, padx=20, pady=20)           
        
        
        # insert at line 0 character 0
        # self.text = self.textbox.get("0.0", "end")  # get text from line 0 character 0 till the end
        # self.textbox.delete("0.0", "end")  # delete all text
        #self.textbox.configure(state="disabled")  # configure textbox to be read-only
    
    def startFormation (self):
            os.system("gnome-terminal -x python3 /home/ozkan/crazyswarm/ros_ws/src/crazyswarm/scripts/swarm_control_15_v2.py --sim")
            rospy.loginfo("Formation mission has been established!")
    
    def startFire(self):
            os.system("gnome-terminal -x python3 /home/ozkan/crazyswarm/ros_ws/src/crazyswarm/scripts/Camera_subscriber.py")
            rospy.loginfo("Formation mission has been established!")
    def startObstacle(self):
            os.system("gnome-terminal -x python3 /home/ozkan/crazyswarm/ros_ws/src/crazyswarm/scripts/Camera_subscriber.py")
            rospy.loginfo("Formation mission has been established!")                
        
    def camera(self):
        os.system("gnome-terminal -x python3 /home/ozkan/crazyswarm/ros_ws/src/crazyswarm/scripts/Camera_subscriber.py")
    
    def savePosition(self):
        
        x= float(self.entry_box4.get())  
        y= float(self.entry_box5.get())
        z= float(self.entry_box6.get())
        agent_name = self.combo_box5.get()
        if self.entry_box4.get() != "" and self.entry_box5.get()!= "" and self.entry_box6.get()!= "":
            x= float(self.entry_box4.get())  
            y= float(self.entry_box5.get())
            z= float(self.entry_box6.get())            
            for i in range(len(self.agent_list3)+1):
                print(i)
                if agent_name == "UAV %s"%i:
                    self.PositionList[i-1][0] =x
                    self.PositionList[i-1][1] =y
                    self.PositionList[i-1][2] =z
                    self.Posdata.x = x
                    self.Posdata.y = y
                    self.Posdata.z = z
                    
        print(self.Posdata)    
        self.textbox3.delete("0.0","end")
        self.textbox3.insert("0.0",str(self.PositionList))
        
        
    def sendPosition(self):
        
        
        self.pub2.publish(self.Posdata)
        # rospy.loginfo(self.floatdata.data)

                    
    def simulationFlight(self):
        os.system("gnome-terminal -x python3 /home/ozkan/crazyswarm/ros_ws/src/crazyswarm/scripts/swarm_control_17.py --sim")
    def realTimeFlight(self):
        os.system("gnome-terminal -x python3 /home/ozkan/crazyswarm/ros_ws/src/crazyswarm/scripts/swarm_control_15.py")        
        # subprocess.call("python3 swarm_control_15.py --sim",shell=True)
        # subprocess.call(['terminator', "--", "python3", "server_deployment.py"])
    def delete_agent_list(self):
        self.PositionList = [0, 0, 0]
        with open('/home/ozkan/crazyswarm/ros_ws/src/crazyswarm/launch/crazyflies.yaml', 'r') as file:

            try:
                #channel_data = int(self.entry_box1.get())
                id_data = int(self.entry_box2.get())
                #initialPosition = self.entry_box3.get()
                documents = yaml.full_load(file)

                for item, doc in documents.items():
                        for i in doc:
                            if(i['id'] == id_data):
                                #delete the dictionary
                                doc.remove(i)
                                # After deleting the dictionary, update the yaml file 
                                with open('/home/ozkan/crazyswarm/ros_ws/src/crazyswarm/launch/crazyflies.yaml', 'w') as file: 
                                    # Don't delete commented lines in the yaml file
                                    yaml.dump(documents, file)
                                print("delete id:", i['id'])
                                #if(item.id==):
                            else:
                                print("not found")
                
            except yaml.YAMLError as exc:
                print(exc)
     
    def add_agent_list(self):
        self.PositionList = [0, 0, 0]
        with open('/home/ozkan/crazyswarm/ros_ws/src/crazyswarm/launch/crazyflies.yaml','+a') as file:
            
            try:
                channel_data = int(self.entry_box1.get())
                id_data = int(self.entry_box2.get())
                
                
                if self.entry_box3.get() != "":
                    initialPosition = self.entry_box3.get()
                   
                    dict_1 = {
                            'id'  : id_data,
                            'channel' : channel_data,
                            'initialPosition': str(initialPosition),
                            'type': 'default'
                            }
                    

                    file.write("\n")  
                    file.write(yaml.dump(data=[dict_1],indent=4))
                    file.write("\n")

                   
                else: 
                    initialPosition = [0.0,0.0,0.0]
                    dict_2 = {
                            'id'  : id_data,
                            'channel' : channel_data,
                            'type': 'default'
                            }
                    

                    file.write("\n")  
                    file.write(yaml.dump(data=[dict_2],indent=4))
                    file.write("\n")
                
            except yaml.YAMLError as exc:
                print(exc)
                
                
                
    def show_agent_list(self):
        self.agent_list2 =[]
        self.agent_list3 =[]

        with open('/home/ozkan/crazyswarm/ros_ws/src/crazyswarm/launch/crazyflies.yaml','r') as file:
            try:

                documents = yaml.full_load(file)
                self.textbox1.delete("0.0", "end")
                self.textbox2.delete("0.0", "end") 
                for item, doc in documents.items():
                    self.agent_list2.append(doc)
                    self.agent_list2.append("\n")
                    print(item, ":", doc)
                    print("\n")
                
                for item, doc in documents.items():
                        j = 1
                        for i in doc: 
                            # print ("id: ",i['id'], "channel: ",i['channel'])
                            my_text="UAV "+ str(j) +"   id: "+ str(i['id'])+ "     channel: "+str(i['channel'])+"\n"
                            self.textbox1.insert("0.0",my_text)
                            self.textbox2.insert("0.0",my_text)
                            my_text2 = "UAV "+ str(j) 
                            self.agent_list3.append(my_text2)
                            # print(self.agent_list3)
                            j = j + 1
                            
                # self.textbox1.delete("0.0", "end")    
                # self.textbox1.insert("0.0", self.agent_list2)
                # my_string=[str(self.agent_list3[i]) for i in range(len(self.agent_list3))]
                # for i in range(len(my_string)):
                #     print(my_string[i]) 

                self.combo_box5=customtkinter.CTkComboBox(self.frame3,values=self.agent_list3)
                self.combo_box5.grid(row=2, column=1,columnspan=1, padx=20, pady=20)
                
                if self.entry_box7.get() != "":
                    self.entry_box7 = customtkinter.CTkEntry(
                        master=self.frame2,placeholder_text_color="white",border_color="black",fg_color='green')
                    self.entry_box7.grid(row=7, column=2, columnspan=2,
                                        padx=20, pady=20, sticky="nsew")
                else:
                            self.entry_box7 = customtkinter.CTkEntry(
                                master=self.frame2, placeholder_text="Number of UGV",font=('Arial bold',16),placeholder_text_color="white",border_color="black",fg_color='red')
                            self.entry_box7.grid(row=7, column=2, columnspan=2,
                                                padx=20, pady=20, sticky="nsew")    
                
                if self.PositionList == [0, 0, 0]:
                    self.PositionList = np.zeros([len(self.agent_list3),3],dtype=float)
                    self.textbox3.delete("0.0","end")
                    self.textbox3.insert("0.0",str(self.PositionList))
                    
                    print(self.PositionList)                       

                print(self.PositionList)
                        
            except yaml.YAMLError as exc:
                print(exc)
                    
    def shape_select(self):
        self.shape_status=self.combo_box3.get()
        if self.shape_status == 'Random(N UAVs)':
            self.shape = 0
        elif self.shape_status == 'Triangle(3 UAVs)':
            self.shape = 3
        elif self.shape_status == 'Square(4 UAVs)':
            self.shape = 4
        elif self.shape_status == 'V_shape(5 UAVs)':
            self.shape = 5
        elif self.shape_status == 'Crescent(6 UAVs)':
            self.shape = 6
        elif self.shape_status == 'Pyramid (5 UAVs)':
            self.shape = 7
        elif self.shape_status == 'Cube (8 UAVs)':
            self.shape = 8
        elif self.shape_status == 'Triangle Prisma (6 UAVs)':
            self.shape = 9
        elif self.shape_status == 'Star(10 UAVs)':
            self.shape = 10
        elif self.shape_status == 'Pentagon Prisma (10 UAVs)':
            self.shape = 11
        elif self.shape_status == 'Hexagon Prisma (12 UAVs)':
            self.shape = 12
        elif self.shape_status == 'Cylinder (10 UAVs)':
            self.shape = 13
        elif self.shape_status == 'Pentagon (5 UAVs)':
            self.shape = 14
        elif self.shape_status == 'Hexagon (6 UAVs)':
            self.shape = 15
        else: 
            self.shape = -1

        if self.shape != -1:
            rospy.loginfo("UAVs change formation shape")
            self.data.data = [self.takeoff, self.takeoff_style, self.land, self.land_style, self.formation_status, self.shape, self.mission, self.change_formation, self.formation_rotation, self.ugv_number]    
            self.pub.publish(self.data)    
            print("Formation Data: ", self.data.data)
            print("Shape: ", self.shape_status)
           
        
    def mission_select(self):
        
        self.mission_value=self.combo_box4.get()

        # self.combo_box4=customtkinter.CTkComboBox(self.frame,values=["Mission Selection","Fire Fighter","Formation Flight","Obstacle Avoidance"])
        
        if self.mission_value == "Fire Fighter":
            self.mission = 1

        elif self.mission_value == "Formation Flight":
            self.mission = 2
            
        elif self.mission_value == "Obstacle Avoidance":
            self.mission = 3
        else: 
            self.mission_value = 0 

        

        if self.mission != 0:
            
            rospy.loginfo("UAVs change %s mission status" %self.mission_value)
            self.formation_status = 1
            self.data.data = [self.takeoff, self.takeoff_style, self.land, self.land_style, self.formation_status, self.shape, self.mission, self.change_formation, self.formation_rotation, self.ugv_number]            
            print("Mission Data: ", self.data.data)

            self.pub.publish(self.data)    
                     
   
    def main(self):
        
        self.mainloop()

    def formation_chance(self):
        if self.takeoff ==1 and self.formation_status == 1:
            print("Formation chancing command is sent")
            self.change_formation = 1
            rospy.loginfo("UAVs change formation status")
            self.data.data = [self.takeoff, self.takeoff_style, self.land, self.land_style, self.formation_status, self.shape, self.mission, self.change_formation, self.formation_rotation, self.ugv_number]            
            print("Formation Change Data", self.data)
            self.pub.publish(self.data)
            print("Formation Change Data_1", self.data.data)
            if self.change_formation == 1:
                rospy.sleep(0.4)
                self.change_formation = 0
                self.data.data = [self.takeoff, self.takeoff_style, self.land, self.land_style, self.formation_status, self.shape, self.mission, self.change_formation, self.formation_rotation, self.ugv_number]                
                self.pub.publish(self.data)
                print("Formation Change Data_2", self.data.data)

            
    def rotate_formation(self):
        if self.takeoff ==1 and self.formation_status == 1:
            print("Formation chancing command is sent")
            self.formation_rotation = 1
            rospy.loginfo("UAVs change formation status")
            self.data.data = [self.takeoff, self.takeoff_style, self.land, self.land_style, self.formation_status, self.shape, self.mission, self.change_formation, self.formation_rotation, self.ugv_number]            
            self.pub.publish(self.data)
            print("Formation Rotation Data_1", self.data.data)
            if self.formation_rotation == 1:
                rospy.sleep(0.4)
                self.formation_rotation = 0
                self.data.data = [self.takeoff, self.takeoff_style, self.land, self.land_style, self.formation_status, self.shape, self.mission, self.change_formation, self.formation_rotation, self.ugv_number]
                self.pub.publish(self.data)
                print("Formation Rotation Data_2", self.data.data)


    def take_off(self):
        
        if self.takeoff == 0 and self.land == 1:
            print("taking off")
            self.takeoff = 1
            self.land = 0
            self.takeoff_style_read = self.combo_box_takeOff.get()
            
            if self.takeoff_style_read == "All":
                self.takeoff_style = 1
                self.formation_status = 1
            elif self.takeoff_style_read == "Desired location":
                self.takeoff_style = 2
                self.formation_status = 1
            else: 
                self.takeoff_style = 0
                self.formation_status = 0 # bu değişkenin kullanılması şart değil
                
            rospy.loginfo("UAVs taking off")
            self.data.data = [self.takeoff, self.takeoff_style, self.land, self.land_style, self.formation_status, self.shape, self.mission, self.change_formation, self.formation_rotation, self.ugv_number]    
            self.pub.publish(self.data)
            print("Take Off Data: ", self.data.data)
        else:
            rospy.loginfo("UAVs already take off")
            print("Take Off Data: ", self.data.data)
               
            
    def refreshFlight(self):
            
        rospy.loginfo("Refresh The Gui")
        if self.refresh_flight == 0:
            self.land = 1
            self.takeoff = 0
            self.formation_status = 0
            self.shape = -1
            self.mission = 0
            self.land_style = 0
            self.takeoff_style = 0
            self.change_formation = 0
            self.formation_rotation = 0
            self.PositionList = [0, 0, 0]
            self.data.data = [self.takeoff, self.takeoff_style, self.land, self.land_style, self.formation_status, self.shape, self.mission, self.change_formation, self.formation_rotation, self.ugv_number]    
            self.pub.publish(self.data)
            print("Refreshed Data: ", self.data.data)
            if self.refresh_flight == 1:
                rospy.sleep(0.4)
                self.refresh_flight = 0

    def select_agent(self):
        selected_agent=self.combo_box5.get()    
            

    # def select_uav(self):

        
    #     try:
    #         count = int(self.entry_box.get())
    #         if count >0 :
    #             self.numberUAV = count
    #             info = str(count) + " number of UAV is selected"
    #             rospy.loginfo(info)
    #             self.data.data = [self.numberUAV, self.takeoff, self.land, self.formation_status]
    #             self.pub.publish(self.data)
    #         else:
    #             rospy.loginfo("you must insert positive number")
            
    #     except: ValueError

    def landing(self):    
        
        if self.takeoff == 1 and self.land == 0:
                print("landing")

                self.land = 1
                self.land_style_read = self.combo_box_landing.get()
                
                if self.land_style_read == "All":
                    self.land_style = 1
                elif self.land_style_read == "One by one":
                    self.land_style = 2
                elif self.land_style_read == "Desired location":
                    self.land_style = 3
                else:
                    self.land_style = 0
                
                self.data.data = [self.takeoff, self.takeoff_style, self.land, self.land_style, self.formation_status, self.shape, self.mission, self.change_formation, self.formation_rotation, self.ugv_number]   
        
                self.pub.publish(self.data)
                rospy.loginfo("landing UAVs with %s swarm style"%self.land_style_read)
                print("Landing Data: ", self.data.data)
        else:
                rospy.loginfo("UAVs not take off status")
                print("Landing Data: ", self.data.data)
                

if __name__ == '__main__':
    gui=GUI()
    try:
        gui.main()
        
    except rospy.ROSInterruptException:
        pass
