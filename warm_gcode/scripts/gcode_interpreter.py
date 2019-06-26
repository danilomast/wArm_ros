#!/usr/bin/env python
import os
import sys
import copy
import rospy
import copy
import subprocess
from math import *
from geometry_msgs.msg import Point
from math import pi
from std_msgs.msg import String
from std_msgs.msg import Bool
from warm_arduino.msg import ArmJointState


from Tkinter import *
import Tkinter, Tkconstants, tkFileDialog

#constants
GCODE_UNIT_MM=1
DO_CALIB=0
DO_ALIGNMENT=1
RESOLUTION=1.5#mm 
AUTO_EXECUTION=1
INVERT_AXIS_FOR_INKSCAPE=1

#colours
G  = '\033[92m' # green
W  = '\033[0m'  #white
B  = '\033[94m'  #blue 


class gcode_interpreter(object):


  #Constructor function: initializes variables and defines publisher 
  def __init__(self):
    
    super(gcode_interpreter, self).__init__()
    self.pub = rospy.Publisher('coordinates', Point, queue_size=10)
    rospy.init_node('gcode_interpreter')
    self.rate = rospy.Rate(10) # 10hz

    # Instance variables
    self.box_name = ''
    self.robot = None
    self.scene = None
    self.group = None
    self.display_trajectory_publisher = None
    self.planning_frame = None
    self.eef_link = None
    self.group_names = None
    self.read_file = None
    self.parsed_lines = []



  #Not used: calibration of robot using endstops
  def robot_calibration(self):
    
    pub_calib = rospy.Publisher('joint_steps', ArmJointState, queue_size=10)
    msg = ArmJointState ()
    msg.position1=0
    msg.position2=0
    msg.position3=0
    msg.calib=1

    print("Premi 'invio' per avviare la calibrazione")
    raw_input()
    pub_calib.publish(msg)

    msg = rospy.wait_for_message("calib_feedback", Bool)
    print(msg)

    print G +"Calibrazione completata!"+W    

 
 
  # Generates a window in order to select the gcode file from the hdd and saves its path
  def read_gcode_file(self):
	
	print "============ Seleziona il file .gcode "
	root = Tk()
	root.withdraw()
	root.filename = tkFileDialog.askopenfilename(initialdir = os.path.dirname(__file__),title = "Selezione il file .gcode",filetypes = (("file gcode","*.gcode"),("file gcode","*.ngc"),("all files","*.*")))
  	self.read_file=open(root.filename,"r").readlines()
  	print "============ Percorso assoluto: "+ root.filename;	
  	root.destroy()
  	return root.filename
  	#print(self.read_file)


  # Generates a dictionary structure with all the parsed lines from the gcode file
  def parse_gcode(self):
  	print "============ Premi 'invio' per interpretare il gcode fornito "
  	raw_input()

  	x_pos=0
  	y_pos=0
  	z_pos=0

  	for line in self.read_file:
		found_xy_coord=0
  	 	line = line.replace("\n","")
  	 	line = line.replace("\r","")

  	 	if line:
  	 		sub_codes=line.split()
  	 		
  	 		#Supported GCODE instructions: G1,G01,G00
			if sub_codes[0]=="G1" or sub_codes[0]=="G01" or sub_codes[0]=="G00" :
  				for sub_code in sub_codes:
  				
  					if sub_code[0]=="X":
  						x_pos=sub_code[1:]
  						found_xy_coord = 1
  					elif sub_code[0]=="Y":
  						y_pos=sub_code[1:]
  						found_xy_coord = 1
  					elif sub_code[0]=="Z":
  						z_pos=sub_code[1:]

				if (INVERT_AXIS_FOR_INKSCAPE):
					if(float(x_pos)>0 and found_xy_coord):
						x_pos = "-" + x_pos      
					elif(float(x_pos)<0 and found_xy_coord ):
						x_pos = x_pos.replace("-", "")			      
					#print x_pos
					self.parsed_lines.append(dict(zip(['x_pos', 'y_pos', 'z_pos'],[y_pos,x_pos,z_pos])))
				else:
					self.parsed_lines.append(dict(zip(['x_pos', 'y_pos', 'z_pos'],[x_pos,y_pos,z_pos])))

  	#print(self.parsed_lines)
  	print(G + "============ Parsing completato!" + W)



  #Publish all the coordinates in the dictionary as a Point variable using ROS publisher
  def execute_gcode(self):
  	print "============ Premi 'invio' per portare la penna sul foglio"
  	raw_input()
  
  	first_pass=1
      	waypoints = []
  	for parsed_line in self.parsed_lines:
		
		new_point   = Point()
		new_point.x = float(parsed_line["x_pos"])
		new_point.y = float(parsed_line["y_pos"])
		new_point.z = float(parsed_line["z_pos"])
		
		
		if(DO_ALIGNMENT):
			if not first_pass:			
				waypoints = align(old_point,new_point)	
				
				i=0
				while i < len(waypoints)-1:
				  punto=waypoints[i]
				  print G + "x:",punto.x , "\ty:",punto.y , "\tz: ",punto.z 
				  self.pub.publish(punto)
				  self.rate.sleep()
				  if(not AUTO_EXECUTION):
				   print W + "============ Premi 'invio' per continuare l'esecuzione"
				   raw_input()
				  else:
				    if (i==0):
				      rospy.sleep(time_estimate(old_point,waypoints[0]))
				    else:
				      rospy.sleep(time_estimate(waypoints[i-1],waypoints[i]))
				  i += 1	  
							
		print B + "x:",new_point.x , "\ty:",new_point.y , "\tz: ",new_point.z
		
		self.pub.publish(new_point)
		self.rate.sleep()
		
		if (not AUTO_EXECUTION):
		  print W + "============ Premi 'invio' per continuare l'esecuzione"
		  raw_input()
		else:
		  if not first_pass:
		    if(DO_ALIGNMENT and len(waypoints)>1):
		      rospy.sleep(time_estimate(waypoints[len(waypoints)-2],new_point))
		    else:
		      rospy.sleep(time_estimate(old_point,new_point))
		  else:
		    
		    print W + "============ Premi 'invio' per cominciare l'esecuzione"
		    raw_input()
		    
		old_point = new_point
		first_pass=0



#Estimates time necessary to go from point1 to point2, with velocity estim_velocity
def time_estimate(point1,point2):
  estim_velocity=15
  #Compute euclidean distance between points
  dist=sqrt(pow((point2.x-point1.x),2) +  pow((point2.y-point1.y),2) + pow((point2.z-point1.z),2))
  print W + "Aspetto ", dist/estim_velocity , "s"
  return dist/estim_velocity
  

#Given two points, it "undersamples" the path between the two at a specified RESOLUTION
def align (point1,point2):
	punto1=copy.deepcopy(point1)
	punto2=copy.deepcopy(point2)
	
	#Compute euclidean distance between points
	dist=sqrt(pow((punto2.x-punto1.x),2) +  pow((punto2.y-punto1.y),2) + pow((punto2.z-punto1.z),2))
	#azimuth angle
	azim=atan2(punto2.y-punto1.y, punto2.x-punto1.x) 
	#elevation angle
	elev=atan2(punto2.z-punto1.z,sqrt(pow((punto2.x-punto1.x),2) +  pow((punto2.y-punto1.y),2)))
	#increments computation
	incr_dist=0
	increment=RESOLUTION
	waypoints= []

	while (incr_dist < dist):
		if (abs(dist - incr_dist) < RESOLUTION):
			increment = abs(dist - incr_dist)

		punto1.x  = punto1.x  + increment*cos(azim)*cos(elev)
		punto1.y  = punto1.y  + increment*sin(azim)*cos(elev)
		punto1.z  = punto1.z  + increment*sin(elev)
		incr_dist = incr_dist + increment

		if(abs(punto1.x)< 0.001):
			punto1.x=0
		if(abs(punto1.y)< 0.001):
			punto1.y=0		
		if(abs(punto1.z)< 0.001):
			punto1.z=0

		new_point= Point()
		new_point.x=punto1.x
		new_point.y=punto1.y
		new_point.z=punto1.z
		waypoints.append(new_point)
		
		#print "x:",punto1.x , "\ty:",punto1.y , "\tz: ",punto1.z 
	return waypoints



def main():
  try:

    print G + "======================"
    print G + "GCODE-INTERPRETER v1.0"   
    print G + "======================"
    print(B+'Unita di misura di default:'+G+' mm' if GCODE_UNIT_MM else B + 'Unita di misura di default: '+ G+'non mm')
    print(B+'Calibrazione:' + G + ' ON' if DO_CALIB else B+'Calibrazione:'+G+' OFF')
    print(B+'Allineamento:'+G+' ON' if DO_ALIGNMENT else 'Allineamento:'+G+' OFF')
    print B+'Risoluzione allineamento: '+G+str(RESOLUTION)+' mm'
    print(B+'Esecuzione automatica:'+G+' ON' if AUTO_EXECUTION else 'Esecuzione automatica:'+G+' OFF')
    print B+  "======================"
    print W+" "



    gcode_interpreter_in = gcode_interpreter()
    file_path=gcode_interpreter_in.read_gcode_file()
   
    if DO_CALIB:
      gcode_interpreter_in.robot_calibration()
    
    gcode_interpreter_in.parse_gcode()
    gcode_interpreter_in.execute_gcode()




  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

