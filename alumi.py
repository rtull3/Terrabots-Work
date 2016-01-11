import os
import sys
import serial


class lynx6alumi(object):
    __alphaIDs = [0,1,2,3,4,5]
    #__alpha2musec__=
    __alphaHome=[0,0,0,0,0,1] #arms home position in degrees
    __alphaSleep =[0,125,70,70,0,1] #arm sleep postion in degrees
    __alphaOrient = [-1,1,1,-1,1,-1] #switches around the relation between degree limits and signal limits
    __alphaLims = [[-90,0,-90,-90,-90,0], #degree limits on the servos
                     [0,90,0,0,0,0.75],
                     [90,180,90,90,80,1.25]]
                     __musecLims = [[520,730,580,590,600,800], #signal limits determined through calibration sessions
                    [1500,1480,1390,1450,1430,1500],
                    [2390,2190,2290,2350,2360,2100]]
    __mm2in = 1/25.4 # millimeters to inches
    __serport = None
    __serialid= None
    __linklen= [65,144,127,62.5,15] #measured robot arm section lengths in mm
    __port=None
                     
    def __init__(self):
                         
        self.__linklen = [x*self.__mm2in for x in self.__linklen] #convert the link lengths to mm
        self.__serport = serial.Serial('/dev/tty.wchusbserial410',115200)#serial connection setup may differ depending on pc
        #self.__serport.open()

    def gotoHome(self,*arg): #sends the robot arm to the home postion
        if len(arg)<1:
            arg=[4,0] #time in seconds is not required type kaiju.gotoHome()
        self.setArm(self.__alphaHome,arg[0])
                    
    def gotoSleep(self,*arg): #sens the robot arm to the sleep postion
        if len(arg)<1:
            arg=[4,0]
        self.setArm(self.__alphaSleep,arg[0])
            
    def interpolate2(self,y2,y1,x2,x1,val): #convert angle to commands to pwm commands
        sol=((y2-y1)*(val-x1)/(x2-x1))+y1
        return sol


    def setArm(self,alpha,time=None): #sends arm movement commands to the control board over the serial connection
        ticks=[0]*6 #create a array of zeros of length 6
        if time is None: #set the time value if none given
            time=4

        if len(alpha)==5 and isinstance(alpha[0],list)==False:
            alpha = alpha+self.__alphaHome[5] #if list of length 5 add command for servo 6
        elif len(alpha)!=6 or isinstance(alpha[0],list)==True: #if format constraints not met end program
            print len(alpha[0]), isinstance(alpha[0],list)
            return
                for ii in range(len(self.__alphaIDs)): #calculate the pwm commands
            if alpha[ii]<=self.__alphaLims[2][ii] and alpha[ii]>=self.__alphaLims[0][ii]:
                if self.__alphaOrient[ii]==1 and alpha[ii]>=0:
                    ticks[ii]=self.interpolate2(self.__musecLims[2][ii],self.__musecLims[1][ii],self.__alphaLims[2][ii],self.__alphaLims[1][ii],alpha[ii])
                elif self.__alphaOrient[ii]==1 and alpha[ii]<0:
                    ticks[ii]=self.interpolate2(self.__musecLims[1][ii],self.__musecLims[0][ii],self.__alphaLims[1][ii],self.__alphaLims[0][ii],alpha[ii])
                elif self.__alphaOrient[ii]==-1 and alpha[ii]<0:
                    ticks[ii]=self.interpolate2(self.__musecLims[2][ii],self.__musecLims[1][ii],self.__alphaLims[0][ii],self.__alphaLims[1][ii],alpha[ii])
                elif self.__alphaOrient[ii]==-1 and alpha[ii]>=0:
                    ticks[ii]=self.interpolate2(self.__musecLims[1][ii],self.__musecLims[0][ii],self.__alphaLims[1][ii],self.__alphaLims[2][ii],alpha[ii])
            else:
                ticks[ii]=self.__musecLims[1][ii] #default value for movement command if limits exceeded
                print 'setting default command because limts are exceeded'
        ticks=map(int,ticks) #format ticks to an int type
        ticks=map(str,ticks) #format ticks to string type
        ticks=['P'+x for x in ticks] #more formatting to fit command string constraints
        servos=['#'+x+' ' for x in map(str,self.__alphaIDs)]
        for y in range(len(servos)):
            servos[y]=servos[y]+ticks[y]
        cmdstr=' '.join(servos)
        cmdstr= cmdstr+' T%d\r' % (time*1000)
        print cmdstr #display command string
        if self.__serport.isOpen(): #send command to control board
            self.__serport.write(cmdstr)


    def free(self): #close serial port
        self.__serport.close()
















