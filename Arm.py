import math
from ax12 import Ax12

class Arm(Ax12,object):
    
    _servos = [2,4,6]
    
    L1 = 6.790  #centimeters
    L2 = 6.855  #centimeters
    L3 = 10.650 #centimeters
    a1 = 1.320  #centimeters
    
    def __init__(self):
        self.setHome()
        
    def FK(self,theta1, theta2, theta3):
        T1 = math.radians(theta1)
        T2 = math.radians(theta2)
        T3 = math.radians(theta3)
        
        A10 = [[math.cos(T1),  0, -math.sin(T1), (-a1*math.cos(T1))],
               [math.sin(T1),  0,  math.cos(T1), (-a1*math.sin(T1))],
               [           0, -1,             0,                 L1],
               [           0,  0,             0,                  1]]
        
        A21 = [[math.cos(-T2), -math.sin(-T2), 0, (L2*math.cos(-T2))],
               [math.sin(-T2),  math.cos(-T2), 0, (L2*math.sin(-T2))],
               [            0,              0, 1,                  0],
               [            0,              0, 0,                  1]]
        
        A32 = [[math.cos(-T3),  0, -math.sin(-T3), (L3*math.cos(-T3))],
               [math.sin(-T3),  0,  math.cos(-T3), (L3*math.sin(-T3))],
               [            0, -1,              0,                  0],
               [            0,  0,              0,                  1]]
        
        A30 = A10*A21*A32
        
        return A30
    
    def IK(self,x,y,z):
        T1 = math.atan2(y,x)
        T3 = -(math.acos((math.pow(x,2) +math.pow(y, 2) + math.pow((z-L1),2) - math.pow(L3,2) - math.pow((L2-a1),2))/(2*(L2-a1)*L3)));
        
        tangPhi = (z - L1)/(math.sqrt(math.pow(x,2)+math.pow(y,2)))
        tangBeta = (math.sin(T3)*L3)/((L2-a1)+(math.cos(T3)*L3))
        
        T2 = math.atan2((tangPhi - tangBeta),(1+(tangPhi*tangBeta)))
        
        angles = [math.degress(T1), math.degress(T2), math.degress(T3)]
        
        anglesToDec = self.anglesToDec(angles)
        
        return anglesToDec
    
    def anglesToDec(self, angles):
        anglesToDec = [1023.0-(512.0+((511.0/150.0)*(90.0+angles[0]))), 1023.0-(512.0+((angles[1]-90.0)*(511.0/150.0))), 1023.0-(512.0+(511.0/150.0)*angles[2])]
        
        return anglesToDec
    
    def decToAngles(self, dec):
        decToAngles = [((150.0/511.0)*(511.0-dec[0]))-90.0, ((150.0/511.0)*(511.0-dec[1]))+90.0, (150.0/511.0)*(511.0-dec[2])]
        
        return decToAngles