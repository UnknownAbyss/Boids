import math


class Vector:
    def __init__(self):
        self.components = []
    
    def setVectorC(self, *comps):
        self.components = [a for a in comps]
    
    def setVectorA(self, angle, mag):
        angrads = math.radians((angle + 360)%360)
        self.components = [mag * math.cos(angrads), mag * math.sin(-angrads)]
    
    def scalarMultiply(self, k):
        self.components = [k*a for a in self.components]
    
    def getMag(self):
        x = self.components[0]
        y = self.components[1]
        sum_sq = x**2 + y**2
        return sum_sq**0.5
    
    def setMag(self, k):
        magn = self.getMag()
        self.scalarMultiply(k * ((self.getMag())**(-1)))
    
    def getAngle(self):
        x = self.components[0]
        y = self.components[1]
        if x != 0:
            if x > 0:
                ang = math.degrees(math.atan(-y/x))
            if x < 0:
                ang = 180 + math.degrees(math.atan(-y/x))
        elif y > 0:
            ang = -90
        elif y < 0:
            ang = 90
        else:
            ang = 0
        return ang

def addVects(vec1 , vec2):
    com1 = vec1.components[0] + vec2.components[0]
    com2 = vec1.components[1] + vec2.components[1]
    newVec = Vector()
    newVec.setVectorC(com1, com2)
    return newVec

def subVects(vec1 , vec2):
    com1 = vec1.components[0] - vec2.components[0]
    com2 = vec1.components[1] - vec2.components[1]
    newVec = Vector()
    newVec.setVectorC(com1, com2)
    return newVec
