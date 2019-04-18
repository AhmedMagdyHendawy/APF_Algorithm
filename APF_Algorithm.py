#!/usr/bin/env python

# APF is more suitable when the obstacles are not near from each other, to not stuck in local minima
#In case of F_rep I added all the values with abs, if we need a direction it will behave differently

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge, CvBridgeError

import cv2
from matplotlib import pyplot as plt
import numpy as np

import math
import time



# ROS

from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates

class Autonomous_Robot():
    def __init__(self,current_pose,robot_lenght,wheel_redius):
        self.current_pose=current_pose
        self.l=robot_lenght
        self.r=wheel_redius


    def GoToGoalController(self,
                            Initial_pose,
                            Final_pose,
                            Map,
                            Ts,
                            tol,  #Matrix of the three coordinate tolerance
                            K,    #Matrix of the three gains of rho, alpha, beta
                            ):
        Done=False
        xg=Final_pose[0]
        yg=Final_pose[1]
        thetag=Final_pose[2]

        Krho=K[0]
        Kalpha=K[1]
        Kbeta=K[2]

        control_input=[]
        states=[[self.current_pose[0]],[self.current_pose[1]]]
        
        self.current_pose=Initial_pose

        while Done == False :
            xc=self.current_pose[0]
            yc=self.current_pose[1]
            thetac=self.current_pose[2]
            

            rho=math.sqrt(math.pow((xg-xc),2)+math.pow((yg-yc),2))
            alpha=(math.atan2(yg-yc,xg-xc)+(yg-yc<0)*2*math.pi)-thetac
            beta=-alpha-thetac+thetag

            if abs(alpha)<=math.pi/2:
                V=Krho*rho
            else:
                V=-Krho*rho

            omega=Kalpha*alpha+Kbeta*beta

            input_tmp=[V,omega]
            control_input.append(input_tmp)

            wr=(2*V+omega*self.l)/(2*self.r)
            wl=((2*V)/self.r)-wr

            xnew=xc+Ts*((self.r*math.cos(thetac)/2)*(wr+wl))
            ynew=yc+Ts*((self.r*math.sin(thetac)/2)*(wl+wr))
            thetanew=thetac+Ts*((self.r/self.l)*(wr-wl))

            self.current_pose[0]=xnew
            self.current_pose[1]=ynew
            self.current_pose[2]=thetanew
            states[0].append(xnew)
            states[1].append(ynew)
            time.sleep(0.08)
            if (abs(xg-xc)<tol[0]) & (abs(yg-yc)<tol[1]):
                Done =True

            
        plt.scatter(states[0],states[1])
        plt.show()    
        return control_input

def color_picker(rect,thresh1):
  roi=thresh1[rect[0][1]:rect[1][1],rect[0][0]:rect[1][0]]
  average=cv2.mean(roi)
  if average==(0,0,0,0) or (average[1]<=10 and average[0]==0 and average[2]==0):
    return 0
  return 1


def heuritic_value(grid,final_pose):
    for i in range(grid.shape[0]):
        for j in range(grid.shape[1]):
            dist = np.linalg.norm(np.array((grid[i][j])["centroid"])-np.array((grid[final_pose[0]][final_pose[1]])["centroid"]))
            dist=float(dist/50)
            (grid[i][j])["heu"]=dist
            (grid[i][j])["q_x_goal"]=0.02*(np.array((grid[i][j])["centroid"])-np.array((grid[final_pose[0]][final_pose[1]])["centroid"]))[0]
            (grid[i][j])["q_y_goal"]=0.02*(np.array((grid[i][j])["centroid"])-np.array((grid[final_pose[0]][final_pose[1]])["centroid"]))[1]
    return grid

def Attraction_Force(grid,K_att):
    for i in range(grid.shape[0]):
        for j in range(grid.shape[1]):
            (grid[i][j])["F_x_att"]=-K_att*(grid[i][j])["q_x_goal"]
            (grid[i][j])["F_y_att"]=-K_att*(grid[i][j])["q_y_goal"]
            (grid[i][j])["F_att"]=math.sqrt(math.pow((grid[i][j])["F_x_att"],2)+math.pow((grid[i][j])["F_y_att"],2))
    return grid

def Repulsion_Force(grid,K_rep,Q,d_safe):
    for i in range(grid.shape[0]):
        for j in range(grid.shape[1]):
            F_x_rep=0
            F_y_rep=0
            for z in range(grid.shape[0]):
                for w in range(grid.shape[1]):
                    if ((grid[z][w])["obstacle"])==1:
                        dist = 0.02*np.linalg.norm(np.array((grid[i][j])["centroid"])-np.array((grid[z][w])["centroid"]))
                        if (dist>=Q):
                            F_x_rep=F_x_rep+0
                            F_y_rep=F_y_rep+0
                        elif (dist<0.001):
                            F_x_rep=F_x_rep+10000
                            F_y_rep=F_y_rep+10000
                        else:
                            if(i==4 and j==1):
                                print("Hello",F_x_rep,F_y_rep,(grid[i][j])["F_att"],"before")
                                print(dist)
                            elif(i==4 and j==2):
                                print("Not Hello", F_x_rep,F_y_rep,(grid[i][j])["F_att"],"before")
                                print(dist)                           
                            F_x_rep=F_x_rep+K_rep*((1/dist)-(1/Q))*((0.02*((grid[i][j])["centroid"])[0]-0.02*((grid[z][w])["centroid"])[0])/math.pow(dist,3))
                            F_y_rep=F_y_rep+K_rep*((1/dist)-(1/Q))*((0.02*((grid[i][j])["centroid"])[1]-0.02*((grid[z][w])["centroid"])[1])/math.pow(dist,3))
                            if(i==4 and j==1):
                                print("Hello",F_x_rep,F_y_rep,(grid[i][j])["F_att"])
                                print(dist)
                            elif(i==4 and j==2):
                                print("Not Hello", F_x_rep,F_y_rep,(grid[i][j])["F_att"])
                                print(dist)

                                
                    else:
                        F_x_rep=F_x_rep+0
                        F_y_rep=F_y_rep+0
            (grid[i][j])["F_x_rep"]=F_x_rep
            (grid[i][j])["F_y_rep"]=F_y_rep

            (grid[i][j])["F_rep"]=math.sqrt(math.pow(F_x_rep,2)+math.pow(F_y_rep,2))
            (grid[i][j])["F_tot"]=(grid[i][j])["F_rep"]+(grid[i][j])["F_att"]
            if(i==4 and j==1):
                print("Hello",F_x_rep,F_y_rep,(grid[i][j])["F_att"],"new")
                print(dist)
            elif(i==4 and j==2):
                print("Not Hello", F_x_rep,F_y_rep,(grid[i][j])["F_att"],"new")
                print(dist)
    return grid


def APF(grid,start_pose,final_pose):
    grid_new=heuritic_value(grid,final_pose)
    grid_new=Attraction_Force(grid_new,1)
    grid_new=Repulsion_Force(grid_new,15,1.4,0.7)
    c=start_pose[0]
    r=start_pose[1]
    (grid_new[c][r])["visited"]=1
    x=(grid_new[c][r])["centroid"][0]
    y=(grid_new[c][r])["centroid"][1]
    cv2.circle(thresh1, (x,y), 5, (255, 0, 255), -1)
    ROI=[]
    sequence=[start_pose]
    next_state=grid_new[c][r]
    
    while not ((c==final_pose[0]) and (r==final_pose[1])):
      c_tmp=c-1
      r_tmp=r-1
      ROI=[]
      Total_Force=[]
      local_min=False
      for i in range(3):
        for j in range(3):
          if not (c_tmp==c and r_tmp==r):
            if not (c_tmp < 0 or r_tmp <0):
                if not ((grid_new[c_tmp][r_tmp])["obstacle"]==1 or ((grid_new[c_tmp][r_tmp])["visited"]==1 and not(local_min))) :
                    ROI.append([c_tmp,r_tmp])
                    Total_Force.append((grid_new[c_tmp][r_tmp])["F_tot"])
          c_tmp=c_tmp+1
        r_tmp=r+i
        c_tmp=c-1
      
      F_total_mat=np.array(Total_Force)
      print(F_total_mat,ROI)
      if (F_total_mat.size == 0):
        print("stuck")
        x=next_state["centroid"][0]
        y=next_state["centroid"][1]
        cv2.circle(thresh1, (x,y), 5, (0, 0, 0), -1)
        del sequence[-1]
        local_min=True
      else:
        min_index=np.argmin(F_total_mat)
        c=ROI[min_index][0]
        r=ROI[min_index][1]
        
        (grid_new[c][r])["visited"]=1
        next_state=grid_new[c][r]
        x=next_state["centroid"][0]
        y=next_state["centroid"][1]
        if (((c==final_pose[0]) and (r==final_pose[1]))):
            cv2.circle(thresh1, (x,y), 5, (0, 255, 255), -1)
        else:
            cv2.circle(thresh1, (x,y), 5, (0, 0, 255), -1)
        sequence.append([c,r])
        print(sequence)

    return grid_new,sequence


Alautonomous_robot=Autonomous_Robot([0,0,math.pi/2],0.4,0.1/2)
image_=cv2.imread("gazebo.jpg")

crop_img = image_[0:930, 450:450+470]
ret,thresh1=cv2.threshold(crop_img,160,200,cv2.THRESH_BINARY)
Grid_Map=[]
x=0
y=0
for i in range(470/50):
    grid=[]
    for j in range(950/50):
        cv2.rectangle(crop_img,(x,y),(x+50,y+50),(0,255,0),1,1)
        cell={"centroid": [x+50/2,y+50/2],
              "obstacle": color_picker([[x,y],[x+50,y+50]],thresh1),
              "coordinates": [[x,y],[x+50,y+50]],
              "visited": 0
                }
        grid.append(cell)
        y=y+50
    Grid_Map.append(grid)
    x=x+50
    y=0
Grid_Map=np.array(Grid_Map)
print(Grid_Map[1][1])
ret,thresh1=cv2.threshold(crop_img,160,200,cv2.THRESH_BINARY)
plt.imshow(crop_img)
plt.imshow(thresh1)
plt.show()
Grid_Map, sequence=APF(Grid_Map,[1,7],[2,12])
plt.imshow(thresh1)
plt.show()





