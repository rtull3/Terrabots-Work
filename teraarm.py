import os
import sys
import time
from alumi import lynx6alumi #from file name import class name

class teraArm(lynx6alumi):

    __Arm=None
    __Home=[0,0,0,0,0,1]
    __pos=[0,0,0,0,0,1]
    

    def __init__(self):
        super(teraArm,self).__init__()
    
    def waitseconds(self,duration):
        start= time.time()
        end= start+duration
        K=True
        while K:
            if time.time()>=end:
                K=False
                print 'done'

    def move(self,command=[0,0,0,0,0,1]):
        t=5 #default time of 5 seconds
        if isinstance(command,list)==True: #check for proper input
            if len(command)<5:
                print 'Input must be list of length 5 or 6'
                return
            elif len(command)==5:
                pos6=[self.__Home[5]]
                command=command+pos6
        else:
            print 'Input must be list of length 5 or 6'

        timeValues=[5, 8, 10, 12]
        for ii in range(len(command)): #determine the angle movement than set the time
            if(abs(command[ii]-self.__pos[ii])>30 and t<timeValues[0]):
                t=timeValues[0]
            
            if(abs(command[ii]-self.__pos[ii])>60 and t<timeValues[1]):
                t=timeValues[1]
            
            if(abs(command[ii]-self.__pos[ii])>90 and t<timeValues[2]):
                t=timeValues[2]
            
            if (abs(command[ii]-self.__pos[ii])>90 and t<timeValues[3]):
                t=timeValues[3]

            self.__pos[ii]=command[ii] #store the new angle for feature angle movements
        self.setArm(command,t)
        return t

def retire(self):
        self.gotoSleep() #put the robot in the sleep positoin
        self.free() #close the serial connection



