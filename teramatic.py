import math
import os
import sys
import numpy as np
import time
from teraarm import teraArm
from SE3 import SE3

class teramatic(teraArm):

    __l1=6.10 #length of arm segment one in inches
    __l2=6.10 #length of arm segment two in inches
    __l3=2.46 #length of arm segment three in inches
    __h=3.14
    __time=5
    
    def __init__(self):
        super(teramatic,self).__init__() #initialize teramatic object that can access its parets fuctions easily
    
    def InverseKin(self,x,y,z,grab=None):#create angles from xyz position for robot arm
        l1=self.__l1 #make typeing these variables easier
        l2=self.__l2
        l3=self.__l3
        h=self.__h
        count=0 #scratch pad variables
        check2=0
        a2d=[0,0,0,0]
        index=0
        '''if z>h:
            z=z-h
        else:
            z=h'''
        if grab is None:#set default value for grab
            grab=1
        pi=math.pi
        scratch=math.sqrt(x*x+y*y+z*z)
        if(math.fabs(self.__l1-self.__l2)<=scratch and scratch<=(self.__l1+self.__l2)):
            r=math.sqrt(x*x+y*y)
            a3c=((x*x+y*y+math.pow((z),2)-math.pow(l1,2)-math.pow(l2,2))/(2*l1*l2))#calcalate ange for joint 3
            a3s=math.sqrt(1-a3c*a3c)
            a3sn=-a3s #calcultate the other option for joint 3
            a2c1=(r*(l1+l2*a3c)+(z)*l2*a3s)/(l1*l1+l2*l2+2*l1*l2*a3c) #calculate the four options for joint 2
            a2s1=((z)*(l1+l2*a3c)-r*l2*a3s)/(l1*l1+l2*l2+2*l1*l2*a3c)
            a2c2=(-r*(l1+l2*a3c)+(z)*l2*a3s)/(l1*l1+l2*l2+2*l1*l2*a3c)
            a2s2=((z)*(l1+l2*a3c)+r*l2*a3s)/(l1*l1+l2*l2+2*l1*l2*a3c)
            a2c3=(r*(l1+l2*a3c)+(z)*l2*a3sn)/(l1*l1+l2*l2+2*l1*l2*a3c)
            a2s3=((z)*(l1+l2*a3c)-r*l2*a3sn)/(l1*l1+l2*l2+2*l1*l2*a3c)
            a2c4=((-r*(l1+l2*a3c)+(z)*l2*a3sn))/(l1*l1+l2*l2+2*l1*l2*a3c)
            a2s4=((z)*(l1+l2*a3c)+r*l2*a3s)/(l1*l1+l2*l2+2*l1*l2*a3c)
            a2v=[a2c1,a2s1,a2c2,a2s2,a2c3,a2s3,a2c4,a2s4]
            a3=math.degrees(math.atan2(a3s,a3c)) #default value for joint 3
            a2=math.degrees(math.atan2(a2v[count+1],a2v[count])) #default value for joint 2
            while count<8:#calculate all of the joint position options for joint 2
                a2d[count/2]=math.degrees(math.atan2(a2v[count+1],a2v[count]))
                print math.degrees(math.atan2(a2v[count+1],a2v[count]))
                count=count+2
            for choice in range(len(a2d)): #choose the first best choice for joint 2 if possible
                if a2d[choice]<180 and a2d[choice]>0:
                    index=choice
                    if choice<2:
                        check2=math.degrees(math.atan2(a3s,a3c))
                    else:
                        check2=math.degrees(math.atan2(a3sn,a3c))
                    if check2<90 and check2>-90:
                        a3=check2#change joint 3 to to match joint 2 solution
                        a2=a2d[choice]
                        break
                    
            #a=r+self.__l2*a3s
            #b=-2*(self.__l1+self.__l2*a3c)
            #d=r-self.__l2*a3s
            #e=2*(self.__l1+self.__l2*a3c)
            #f=r+self.__l2*a3s
            #terms=[a, b, c]
            #terms2=[d ,e ,f]
            # print terms
            #print terms2
            #w=(-b-math.sqrt(math.pow(b,2)-4*a*c))/(2*a)
            #print w
            #w2=(-e-math.sqrt(e*e-4*d*f))/(2*d)
            #print w2
            #a2=2*math.atan(w)*180/math.pi
            #print a2
            #a3r=math.atan2(a3s,a3c)
            #a2r=2*math.atan(w)
            #if a2r>2*math.pi or a2>360:
            #print a2
            #a2r=a2r-2*math.pi
            #a2=a2-360
            #elif a2r<-2*math.pi:
            #a2r=a2r+2*math.pi
            #a2=a2+360
            #a1s=x/(self.__l1*math.sin(a2r)+self.__l2*math.sin(a2r+a3r))
            #a1c=-y/(self.__l1*math.sin(a2r)+self.__l2*math.sin(a2r+a3r))
            #a1=math.atan2(a1s,a1c)*180/math.pi
            #solve for the next alpha using the law of cosines
            #a3=math.atan2(a3s,a3c)*180/math.pi
            a4=180-a3-a2#calculate joint 4
            a4=180-(a4+90)
            a1=math.degrees(math.atan2(y,x))#calculate joint 1
            alphas=[a1, a2, a3,a4]#computed alphas that follow inverse kinematics
            print alphas
            #If the solution does not map to cartesian quadrants 1 and 4, find the
            #supplementry angle to alpha1. Adjust alpha2 and alpha3 so that they can
            #access quadrants 2 & 3
            if(a1>90 or a1<-90):
                if(a1>0):
                    a1=a1-180
                else:
                    a1=a1+180
                
                a2=-1*a2
                a3=-1*a3
                a4=-1*a4
                 
    
            allalphas=[a1, a2, a3, a4]
            #display the modified possibly modified alphas and a4
            print allalphas
        
        
        
            if(a1<=90 and a1>=-90 and a2<=180 and a2>=0 and a3<=90 and a3>=-90 ):
                #Send the solution to the arm if physically possible
                time=self.move([a1 ,a2, a3, a4, 0,grab]) #order the movement of the arm
                print time
                self.waitseconds(time) #give arm time to execute the command
            else:
                print 'Invalid Coordinates not physically possible' #Error Message for invalid coordinates
                alphas = []
        
        else:
            print 'Invalid Coordinates not theoretically possible' #Error Message for invalid coordinates
            alphas = []

def fwdkin(alphas=None): #check if solution is correct
        if alphas is None:
            alphas=[0,0,0,0]
        else:
            alphas=[math.radians(x) for x in alphas]
        cos=[math.cos(x) for x in alphas]
        sin=[math.sin(x) for x in alphas]
        r1=np.array([[cos[0],-1*sin[0],0] #setup rotation and translation matrices
                     ,[sin[0],cos[0],0],
                     [0,0,1]])
        d1=np.array([[0],
                     [0],
                     [self.__h]])
        r2=np.array([[1,0,0],
                     [cos[1],-1*sin[1], 0],
                     [sin[1],cos[1],0]])
        r3=np.array([[1,0,0],
                     [cos[2],-1*sin[2], 0],
                     [sin[2],cos[2],0]])
        r4=np.array([[1,0,0],
                     [cos[3],-1*sin[3], 0],
                     [sin[3],cos[3],0]])
        d3=np.array([[0],
                     [0],
                     [self.__l1]])
        d4=np.array([[0],
                     [0],
                     [self.__l2]])
        gz1=SE3(r1,d1)
        gx2=SE3(r2)
        gx3=SE3(r3,d3)
        gx4=SE3(r4,d4)
        gf=gz1.mtimes(gx2.mtimes(gx3.mtimes(gx4))) #multiply it out
        return gf

                 #def fwdkin(alphas):
                 #gz1= [ cosd(alphas(1)) -sind(alphas(1)) 0 0;...
                 #sind(alphas(1)) cosd(alphas(1)) 0 0;...
                 #0 0 1 h;...
                 #0 0 0 1];
                 #gx2 = [ 1 0 0 0;...
                 #0 cosd(alphas(2)) -sind(alphas(2)) 0;...
                 #0 sind(alphas(2)) cosd(alphas(2)) 0;...
                 #0 0 0 1];
                 #gx3 = [ 1 0 0 0;...
                 #0 cosd(alphas(3)) -sind(alphas(3)) 0;...
                 #0 sind(alphas(3)) cosd(alphas(3)) l1;...
                 #0 0 0 1];
                 #g4 = [ 1 0 0 0;...
                 #0 1 0 0;...
                 #0 0 1 l2;...
                 #0 0 0 1];
                 #k=gz1*gx2*gx3*g4;

                 #return k



