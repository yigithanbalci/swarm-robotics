from Tkinter import *
import ttk
import os

filestring = """<?xml version="1.0"?>

<launch>
   
  <!-- Start Gazebo with wg world running in (max) realtime -->
  <include file="$(find hector_gazebo_worlds)/launch/rolling_landscape_120m.launch"/>
   
  <!-- Spawn simulated quadrotor uav -->
  <include file="$(find quadro_demo)/launch/spawn_four_quadrotors.launch" >
    <arg name="model" value="$(find hector_quadrotor_description)/urdf/quadrotor_hokuyo_utm30lx.gazebo.xacro"/>     
  </include>        
    
  <!-- Start rviz visualization with preset config -->
    <node pkg="rviz" type="rviz" name="rviz" args="-d $(find quadro_demo)/rviz/fourhoctorconfig.rviz"/>
    
    <!-- Start scripts with arguments -->            
"""
    

def launch():
    os.system("roslaunch quadro_demo demo.launch")

def save():
    fo = open("/home/yigithan/catkin_ws/src/quadro_demo/demo.launch", "w")
    filstring = filestring
    i = 1
    for entry in entries:
        filstring = filstring + "<node name=%suav%sscript%s ns=%suav%s%s pkg=%squadro_demo%s type=%sscript.py%s args=%suav%s %s%s/>\n" % ('"',i,'"','"',i,'"','"','"','"','"','"',i,entry.get(),'"')
        i = i +1
    filstring = filstring + "</launch>"
    fo.write(filstring)

f2 = Tk()
entries = []
val=["UAV1","UAV2","UAV3","UAV4"]

for i in range(4):
    label = Label(f2, text=val[i])
    label.grid(row=i, column=0)
    entry = Entry(f2, width=25)
    entry.grid(row=i, column=1)
    entries.append(entry)

button1 = ttk.Button(f2, text="Close", command=exit)
button1.grid(row=4, column=0)
button2 = ttk.Button(f2, text="Launch", command=launch)
button2.grid(row=4, column=1)
button3 = ttk.Button(f2, text="Save", command=save)
button3.grid(row=4, column=2)

f2.mainloop()