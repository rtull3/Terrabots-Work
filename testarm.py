from teramatic import teramatic
#Example code of how to send commands to the robot arm

kaiju = teramatic() #this code setsup the serial connection as part of the arm object

kaiju.InverseKin(7,7,7) #moves the arm to an xyz postion
#can send as many as needed
kaiju.retire() #puts the robot in its sleep postion and ends the serial postion
