import pygame, sys, random, math
import Vectors as v2 #My own Vector module. Gets the job done
from pygame.locals import *

FPS = 30 #screen and FPS parameters
width = 1380
height = 800
size = 10 #size of boids. the rectangle in which they are bound is this width
margin = 2 #thickness of boids
radius = 100 #Radius of view
maxSpeed = 5 #Limits on kinematics
maxAccel = 5
maxboids = 100 #Well really it is the minimum. If there are less than this, we make more
marg = 10 #outer screen margin to spawn new boids

BgColor = (20, 20, 20)
White = (255, 255, 255)
Dred = (150, 0, 0)
Purple = (255, 50, 255)

pygame.init()
DispSurface = pygame.display.set_mode((width,height))
DispSurface.set_alpha(None)
Clock = pygame.time.Clock()


class Boid:

    def __init__(self):
        self.original = pygame.Surface( (size,size) , SRCALPHA)
        self.color = (random.randint(0,255), random.randint(0,255), random.randint(0,255))
        pygame.draw.polygon(self.original, self.color, ( (0, 0 + margin), (0, size - margin), (size , size/2) ) )
        self.pos = self.original.get_rect()
        self.pos = [ random.randint(0,width), random.randint(0,height) ]
        self.vel = v2.Vector()
        self.accel = v2.Vector()
        self.vel.setVectorC(random.randint(-5,5),  random.randint(-5,5))
        self.accel.setVectorC(0, 0)
        self.radius = radius
        self.angle = self.vel.getAngle()

    def update(self):
        self.vel = v2.addVects(self.vel, self.accel)
        self.pos = [self.pos[0] + self.vel.components[0], self.pos[1] + self.vel.components[1]]
        self.angle = self.vel.getAngle()
        self.image = pygame.transform.rotate(self.original, self.angle)
        self.rect = self.image.get_rect(center = self.pos)
        DispSurface.blit(self.image, self.rect)
        
        #This code is for screen wrapping of boids when we are not destroying them for going out of the screen
        # if self.pos[0] > width:
        #     self.pos[0] = self.pos[0] - width
        # if self.pos[0] < 0:
        #     self.pos[0] = width - self.pos[0]
        # if self.pos[1] > height:
        #     self.pos[1] = self.pos[1] - height
        # if self.pos[1] < 0:
        #     self.pos[1] = height - self.pos[1] 
        
        if self.vel.getMag() > maxSpeed:
            self.vel.setMag(maxSpeed)
        if self.accel.getMag() > maxAccel:
            self.accel.setMag(maxAccel)



    

boids = [Boid() for a in range(120)]

while True:

    #Code to delete boids out of our margin range and to add new boids till we reach our minimum amount
    for boid in boids:
        S = boid.pos
        if S[0] > width + marg or S[0]< -marg:
            boids.remove(boid)
        elif S[1] > height + marg or S[1] < -marg:
            boids.remove(boid)
    while len(boids) < maxboids:
        newBoid = Boid()
        out = False
        while not out:
            randX = random.choice([a for a in range(-marg + 5, width + marg - 5)])
            randY = random.choice([a for a in range(-marg + 5, height + marg - 5)])
            if randX not in range(0,width) or randY not in range(0,height):
                out = True
        newBoid.pos = (randX, randY)
        boids.append(newBoid)

    DispSurface.fill(BgColor)
    
    
    for event in pygame.event.get():
        if event.type == QUIT:
            pygame.quit()
            sys.exit()
    
    #Gets the list of boids that a given boid can see and adds it to the Proximity list of the boid
    for me in boids:
        tempBoids = boids.copy()
        tempBoids.remove(me)
        proximity = []
        for others in tempBoids:
            x1 , y1 = me.pos
            x2 , y2 = others.pos
            if (x1 - x2)**2 + (y1 - y2)**2 < radius**2:
                proximity.append(others)
        me.proximity = proximity
        
        #if there is anything in the boid's proximity, it Tries to align, avoid bumping, and generally move towards mean position
        if me.proximity:
            #alignment with direction of other boids
            sum_an1 = 0
            sum_an2 = 0
            for x in me.proximity:
                remVel = v2.Vector()
                remVel.setVectorC(0, 0)
                remVel = v2.addVects(remVel, x.vel)
                if remVel.getMag():
                    remVel.setMag(1/remVel.getMag())
                sum_an1 += remVel.components[0]
                sum_an2 += remVel.components[1]
            sum_an1 /= len(me.proximity)
            sum_an2 /= len(me.proximity)
            angVector = v2.Vector()
            angVector.setVectorC(sum_an1, sum_an2)
            me.avgAngle = angVector.getAngle() #Average angle for neighbors
            me.accel.setVectorA(me.avgAngle, 1)

            #seperation
            me_post = (me.pos[0], me.pos[1])
            pos_VectMe = v2.Vector()
            pos_VectMe.setVectorC(me_post[0], me_post[1])
            for oth in proximity:
                oth_post = (oth.pos[0], oth.pos[1])
                pos_VectOth = v2.Vector()
                pos_VectOth.setVectorC(oth_post[0], oth_post[1])
                diffVec = v2.subVects(pos_VectMe, pos_VectOth)
                if diffVec.getMag():
                    diffVec.scalarMultiply(1/(10*diffVec.getMag())) #the 10 there is strength of seperation. Larger value means less seperation
                me.accel = v2.addVects(me.accel, diffVec)

            #cohesion
            xsum = sum([oth.pos[0] for oth in proximity])/len(proximity)
            ysum = sum([oth.pos[1] for oth in proximity])/len(proximity)
            avgPos = v2.Vector()
            
            avgPos.setVectorC(xsum, ysum)
            diffAvg = v2.subVects(avgPos, pos_VectMe)
            diffAvg.scalarMultiply(0.015)
            me.avgPos = ( round(avgPos.components[0]), round(avgPos.components[1])) 
            me.accel = v2.addVects(me.accel, diffAvg)
    

    ## TRACKING AND DRAWING VISUALS AREA
    #tracking single boid and drawing stuff to visualize what is happening. Honestly this part is terrible
    me_pos = (round(boids[0].pos[0]), round(boids[0].pos[1]))
    # Draw alignment
    posVec = v2.Vector()
    posVec.setVectorC(me_pos[0], me_pos[1])

    angVec = v2.Vector()
    angVec.setVectorA(boids[0].angle, 100) # this number defines length of our alignment visual
    angVec = v2.addVects(posVec, angVec)
    me_angpos = (round(angVec.components[0]), round(angVec.components[1]))
    
    pygame.draw.circle(DispSurface, Dred, me_pos, boids[0].radius)
    pygame.draw.line(DispSurface, Purple, me_pos, me_angpos, 3) #  Draw alignment
    for boid in boids[0].proximity:
        other_pos = (round(boid.pos[0]), round(boid.pos[1]))
        pygame.draw.line(DispSurface, Purple, me_pos, other_pos, 1) #proximity line for neightbors

    pygame.draw.circle(DispSurface, White,  boids[0].avgPos, 10) #average point of mass among the neighboring boids
    


    #updating all boids for position, velocity, acceleration, angle etc
    for me in boids:
        me.update()
    
    pygame.display.update()
    Clock.tick(FPS)
