import numpy as np
import math
class quaternion:
    def __init__(self,x,y,z,c):
        d = math.sqrt(x**2 + y**2 + z**2 + c**2)
        self.x = x/d
        self.y = y/d
        self.z = z/d
        self.c = c/d
    def invq(self): #inverse quaternion function
        return quaternion(-1*(self.x),-1*(self.y),-1*(self.z),self.c)
    def rotq(self,q2 : "quaternion"): # quaternion rotation
        r = quaternion(self.x,self.y,self.z,self.c)
        r.x = self.c*q2.x +self.x*q2.c -self.y*q2.z +self.z *q2.y
        r.y = self.c*q2.y + self.x*q2.z +self.y*q2.c - self.z*q2.x
        r.z = self.c*q2.z - self.x*q2.y + self.y*q2.x +self.z*q2.c
        r.c = self.c*q2.c -(self.x*q2.x + self.y*q2.y + self.z*q2.z)
        return r
    def neg(self):
        return quaternion(-1*(self.x),-1*(self.y),-1*(self.z),-1*(self.c))
#intial values
imu_q_i = quaternion(0.0032026,0.00351542,0.7084,0.70579)
tool_q_i = quaternion(0.70506,0.70914,0.002042,0.0023077)
imuTotool = ((imu_q_i.invq()).rotq(tool_q_i))


#input current imu parameters
val= input("Enter current quaternion imu values: ").split()
a,b,c,d = [float(ele) for ele in val]
currentIMU = quaternion(a,b,c,d) 
toolQuat = currentIMU.rotq(imuTotool) #the required quternion for the tool 
print(toolQuat.x,toolQuat.y,toolQuat.z,toolQuat.c)