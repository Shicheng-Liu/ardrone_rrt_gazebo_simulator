#!/usr/bin/env python

# This work is based on ardrone_tutorial https://github.com/mikehamer/ardrone_tutorials

import roslib
import rospy
import sys, random, math, pygame
from pygame.locals import *
from math import *

# Load the DroneController class, which handles interactions with the drone, and the DroneVideoDisplay class, which handles video display
from drone_controller_gazebo import BasicDroneController
from drone_video_display import DroneVideoDisplay

# Finally the GUI libraries
from PySide import QtCore, QtGui

step_length=10
# Here we define the keyboard map for our controller (note that python has no enums, so we use a class)
class KeyMapping(object):
	PitchForward     = QtCore.Qt.Key.Key_W
	PitchBackward    = QtCore.Qt.Key.Key_S
	RollLeft         = QtCore.Qt.Key.Key_A
	RollRight        = QtCore.Qt.Key.Key_D
	YawLeft          = QtCore.Qt.Key.Key_Q
	YawRight         = QtCore.Qt.Key.Key_E
	IncreaseAltitude = QtCore.Qt.Key.Key_Z
	DecreaseAltitude = QtCore.Qt.Key.Key_C
	Takeoff          = QtCore.Qt.Key.Key_Y
	Land             = QtCore.Qt.Key.Key_H
        Trajectory       = QtCore.Qt.Key.Key_T
        RRT_STAR         = QtCore.Qt.Key.Key_R
	Emergency        = QtCore.Qt.Key.Key_Space

def move(x,y):
    ardrone_pose=controller.ardrone()
    ardrone_x=ardrone_pose[0]
    ardrone_y=ardrone_pose[1]
    xr=ardrone_pose[3]
    yr=ardrone_pose[4]
    zr=ardrone_pose[5]
    w=ardrone_pose[6]
    yaw=atan2(2*(w*zr+xr*yr), 1-2*(yr*yr+zr*zr))
    while abs(yaw)>0.05:
      ardrone_pose=controller.ardrone()
      xr=ardrone_pose[3]
      yr=ardrone_pose[4]
      zr=ardrone_pose[5]
      w=ardrone_pose[6]
      yaw=atan2(2*(w*zr+xr*yr), 1-2*(yr*yr+zr*zr))
      controller.SetCommand(0,0,-2*yaw,0)
    while abs(ardrone_x-x)>0.1 or abs(ardrone_y-y)>0.1:
      ardrone_pose=controller.ardrone()
      ardrone_x=ardrone_pose[0]
      ardrone_y=ardrone_pose[1]
      xr=ardrone_pose[3]
      yr=ardrone_pose[4]
      zr=ardrone_pose[5]
      w=ardrone_pose[6]
      yaw=atan2(2*(w*zr+xr*yr), 1-2*(yr*yr+zr*zr))
      controller.SetCommand(y-ardrone_y, x-ardrone_x, -2*yaw, 0)

def distance(x1,x2):
  return sqrt((x1[0]-x2[0])*(x1[0]-x2[0])+(x1[1]-x2[1])*(x1[1]-x2[1]))

def step(x1,x2):
  if distance(x1,x2)<=step_length:
    return x2
  else:
    angle=atan2(x2[1]-x1[1],x2[0]-x1[0])
    return x1[0]+step_length*cos(angle), x1[1]+step_length*sin(angle)  

class point:
    x=0
    y=0
    last=None
    cost=0
    def __init__(self,x_value,y_value):
         self.x=x_value
         self.y=y_value

def choose_neighborhood(tree):
    count=len(tree)
    neighborhood_calculated=500*sqrt(log(count)/count)
    if neighborhood_calculated<18:
      return neighborhood_calculated
    else:
      return 18

def collision_test(new_vertex,vertex):    # a safe distance of 50 from obstacles
    k=(new_vertex.y-vertex.y)/(new_vertex.x-vertex.x)
    if (new_vertex.x<270 and new_vertex.y>150 and new_vertex.y<260) or (new_vertex.x>130 and new_vertex.y>400 and new_vertex.y<510):
      print("delete this new vertex because of collision")
      return 0
    elif (new_vertex.y-150)*(vertex.y-150)<0 and (new_vertex.x-270)*(vertex.x-270)<0 and vertex.y+k*(270-vertex.x)>150:
      print("delete this new vertex because of collision")
      return 0
    elif (new_vertex.y-260)*(vertex.y-260)<0 and (new_vertex.x-270)*(vertex.x-270)<0 and vertex.y+k*(270-vertex.x)<260:
      print("delete this new vertex because of collision")
      return 0
    elif (new_vertex.y-400)*(vertex.y-400)<0 and (new_vertex.x-130)*(vertex.x-130)<0 and vertex.y+k*(130-vertex.x)>400:
      print("delete this new vertex because of collision")
      return 0
    elif (new_vertex.y-510)*(vertex.y-510)<0 and (new_vertex.x-130)*(vertex.x-130)<0 and vertex.y+k*(130-vertex.x)<510:
      print("delete this new vertex because of collision")
      return 0
    else:
      return 1

def rrt():
    length=400
    width=600
    vertex_number=2000
    white = 255, 255, 255
    black = 0, 0, 0
    red = 255, 0, 0
    green = 0, 255, 0
    blue = 0, 0, 255
    x1=0
    y1=0
    x2=0
    y2=0
    radius=10
    radius_1=0
    radius_2=0
    count=0
    running=1
    while running:
      pygame.init()
      screen = pygame.display.set_mode((length,width))
      pygame.display.set_caption('ardrone_rrt_star')
      screen.fill(white) 
      pygame.draw.circle(screen,red,(x1,y1),radius_1,0)
      pygame.draw.circle(screen,green,(x2,y2),radius_2,0)
      pygame.draw.rect(screen, (0,0,0), (0,200,220,10), 0)
      pygame.draw.rect(screen, (0,0,0), (180,450,220,10), 0)
      tree=[]
      event=pygame.event.poll()
      if event.type == pygame.QUIT:
          running = 0
      elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
        count=count+1
        if count== 1:
          x1,y1=pygame.mouse.get_pos()
          radius_1=10
          pygame.draw.circle(screen,red,(x1,y1),radius_1,0)
          pygame.display.flip()
          ardrone_pose=controller.ardrone()
          ardrone_x=ardrone_pose[0]
          if ardrone_x>1.5:
            move(3,-1)
            move(0,-1)
          if ardrone_x>-1:
            move(0,2)
          move(0.01*y1-3,0.01*x1-2)
          print("in position")
          controller.SetCommand(0, 0, 0, 0)
        elif count==2:
          x2,y2=pygame.mouse.get_pos()
          radius_2=10
          tree.append(point(x1,y1))
          drone=tree[0]
          target=point(x2,y2)
          sign=1
          for i in range(vertex_number):
            print "number = %d" %sign
            sign=sign+1
            vertex_random=point(random.random()*length, random.random()*width)
            vertex = tree[0]
            for x in tree:                
              if distance([x.x,x.y],[vertex_random.x,vertex_random.y])<distance([vertex.x,vertex.y],[vertex_random.x,vertex_random.y]):
                vertex=x
            new=step([vertex.x,vertex.y],[vertex_random.x,vertex_random.y])
            new_vertex=point(new[0],new[1])
            new_vertex.last=vertex
            new_vertex.cost=vertex.cost+distance([x.x,x.y],[vertex_random.x,vertex_random.y])
            neighborhood=choose_neighborhood(tree)
            sign_1=collision_test(new_vertex,new_vertex.last)
            if sign_1==1:
              for x in tree:
                if distance([x.x,x.y],[new_vertex.x,new_vertex.y])<neighborhood and x.cost+distance([x.x,x.y],[new_vertex.x,new_vertex.y])<new_vertex.cost:
                  sign_2=collision_test(new_vertex,x)
                  if sign_2==1: 
                      new_vertex.last=x
                      new_vertex.cost=x.cost+distance([x.x,x.y],[new_vertex.x,new_vertex.y]) 
              tree.append(new_vertex)
              pygame.draw.line(screen,black,[new_vertex.last.x,new_vertex.last.y],[new_vertex.x,new_vertex.y])
              pygame.display.flip()
            else:
              continue
            for i in xrange(len(tree)):
              x=tree[i]
              if x!=new_vertex.last and distance([x.x,x.y],[new_vertex.x,new_vertex.y])<neighborhood and distance([x.x,x.y],[new_vertex.x,new_vertex.y])+new_vertex.cost<x.cost:
                sign_3=collision_test(x,new_vertex)
                if sign_3==1:
                  pygame.draw.line(screen,white,[x.x,x.y],[x.last.x,x.last.y])
                  x.last=new_vertex
                  x.cost=distance([x.x,x.y],[new_vertex.x,new_vertex.y])+new_vertex.cost
                  tree[i]=x
                  pygame.draw.line(screen,black,[x.x,x.y],[x.last.x,x.last.y])
                  pygame.display.flip()
            pygame.draw.circle(screen,red,(x1,y1),radius_1,0)
            pygame.draw.circle(screen,green,(x2,y2),radius_2,0)
            pygame.draw.rect(screen, (0,0,0), (0,200,220,10), 0)
            pygame.draw.rect(screen, (0,0,0), (180,450,220,10), 0)
            pygame.display.flip()

          vertex=tree[0]
          path=[]
          for x in tree:
            if distance([x.x,x.y],[target.x,target.y])<distance([vertex.x,vertex.y],[target.x,target.y]):
              vertex=x
          while vertex!=drone:
            path.append(vertex)
            pygame.draw.line(screen,blue,[vertex.x,vertex.y],[vertex.last.x,vertex.last.y],5)
            vertex=vertex.last
            pygame.display.flip()
          select=0
          for x in path[::-1]:
            select=select+1
            if select%3==0:
              x_goal=x.y/100-3
              y_goal=x.x/100-2
              move(x_goal,y_goal)
          controller.SetCommand(0, 0, 0, 0)
          #break

# Our controller definition, note that we extend the DroneVideoDisplay class
class KeyboardController(DroneVideoDisplay):
	def __init__(self):
		super(KeyboardController,self).__init__()
		
		self.pitch = 0
		self.roll = 0
		self.yaw_velocity = 0 
		self.z_velocity = 0

# We add a keyboard handler to the DroneVideoDisplay to react to keypresses
	def keyPressEvent(self, event):
		key = event.key()

		# If we have constructed the drone controller and the key is not generated from an auto-repeating key
		if controller is not None and not event.isAutoRepeat():
			# Handle the important cases first!
			if key == KeyMapping.Emergency:
				controller.SendEmergency()
			elif key == KeyMapping.Takeoff:
				controller.SendTakeoff()
			elif key == KeyMapping.Land:
				controller.SendLand()
			else:
				# Now we handle moving, notice that this section is the opposite (+=) of the keyrelease section
				if key == KeyMapping.YawLeft:
					self.yaw_velocity += 1
				elif key == KeyMapping.YawRight:
					self.yaw_velocity += -1

				elif key == KeyMapping.PitchForward:
					self.pitch += 1
				elif key == KeyMapping.PitchBackward:
					self.pitch += -1

				elif key == KeyMapping.RollLeft:
					self.roll += 1
				elif key == KeyMapping.RollRight:
					self.roll += -1

				elif key == KeyMapping.IncreaseAltitude:
					self.z_velocity += 1
				elif key == KeyMapping.DecreaseAltitude:
					self.z_velocity += -1

                                elif key == KeyMapping.Trajectory:
                                        move(0,0)
                                        move(3,1)
                                        move(3,-2)
                                        move(-3,-2)
                                        move(-3,2)
                                        move(0,0)
                                elif key == KeyMapping.RRT_STAR:
                                        rrt()
			# finally we set the command to be sent. The controller handles sending this at regular intervals
			controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)

	def keyReleaseEvent(self,event):
		key = event.key()

		# If we have constructed the drone controller and the key is not generated from an auto-repeating key
		if controller is not None and not event.isAutoRepeat():
			# Note that we don't handle the release of emergency/takeoff/landing keys here, there is no need.
			# Now we handle moving, notice that this section is the opposite (-=) of the keypress section
			if key == KeyMapping.YawLeft:
				self.yaw_velocity -= 1
			elif key == KeyMapping.YawRight:
				self.yaw_velocity -= -1

			elif key == KeyMapping.PitchForward:
				self.pitch -= 1
			elif key == KeyMapping.PitchBackward:
				self.pitch -= -1

			elif key == KeyMapping.RollLeft:
				self.roll -= 1
			elif key == KeyMapping.RollRight:
				self.roll -= -1

			elif key == KeyMapping.IncreaseAltitude:
				self.z_velocity -= 1
			elif key == KeyMapping.DecreaseAltitude:
				self.z_velocity -= -1

			# finally we set the command to be sent. The controller handles sending this at regular intervals
			controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)

# Setup the application
if __name__=='__main__':
	import sys
	# Firstly we setup a ros node, so that we can communicate with the other packages
	rospy.init_node('ardrone_rrt')

	# Now we construct our Qt Application and associated controllers and windows
	app = QtGui.QApplication(sys.argv)
	controller = BasicDroneController()
	display = KeyboardController()
	
	display.show()
	
	# executes the QT application
	status = app.exec_()

	# and only progresses to here once the application has been shutdown
	rospy.signal_shutdown('Great Flying!')
	sys.exit(status)
