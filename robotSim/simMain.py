from environment import *
import matplotlib.pyplot as plt
import numpy as np
from robot import *
from bicycleRobot import *

world = environment()
world.print()
feature = [[1,ii] for ii in range(world.length-1)]
world.addFeature(feature)
world.addGoal([9,7])
rb = robot(0,0,0,world)
world.addRobot(rb)
world.print()
world.moveRobot([1,1],0)
world.moveRobot([0,1],0)
world.prettyPrint()
hueristic = []
world.prettyPrint(hueristic)
for y in range(world.length):
    hueristic.append([])
    for x in range(world.width):
        hueristic[y].append( math.sqrt((world.goalX-x)**2 + (world.goalY-y)**2) )
pathPlan = world.planAStar(hueristic)
world.prettyPrint(pathPlan)

#Test DP algorithm on left turn scenario world
costFxn = [1, 1, 1, 10]
driveWorld = environment(6,6,1)
driveWorld.addGoal([0,3],270)
features = [[0,0], [0,1], [0,2], [1,0], [1,1], [1,2],
    [0,4], [0,5], [1,4], [1,5],
    [3,4], [4,4], 
    [3,2], [3,1], [4,2], [4,1] ]
driveWorld.addFeature(features)
driveRobot = robot(2,5,0,driveWorld)
driveWorld.addRobot(driveRobot)
moveCosts = [15, 1, 1]
moves = [[1,0,0],[-1,0,0],[0,1,0],[0,-1,0]]
driveWorld.prettyPrint()
dpCost, dpPlan = driveWorld.planDP(moveCosts)
dpVisual, dpCoords = driveWorld.currentDP(dpCost, dpPlan)
driveWorld.prettyPrint(dpVisual)
smoothPath = driveWorld.smoothPlan(dpCoords)
driveWorld.pathPlot([smoothPath, dpCoords],1)


#simulate P, PD, and PID control of bicycle model for reference tracking
car = bicycleRobot(6) #Default is a 20 unit wheelbase, use 6 m instead
car.set_noise(np.deg2rad(5),1) #set noise to 5 deg steering, 1 m distance
car.set_steering_drift(np.deg2rad(5)) #steering drift to 5 degress

car.set_noise(0,0) 
car.set_steering_drift(np.deg2rad(5))
#simulate car moving n steps
n = 100
#PD control
car.set(0, 1, 0)
xref = list(range(0,n)) 
yref = list(range(0,n)) 
yref = np.zeros(n) 
xtraj, ytraj = car.pidRun(0.2, 0, 3.0, xref, yref)
n = len(xtraj)
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8,8))
ax1.plot(xtraj, ytraj, 'y', label='PD controller')
#PID control
car.set(0, 1, 0)
xtraj, ytraj = car.pidRun(0.2, 0.004, 3.0, xref, yref)
n = len(xtraj)
ax1.plot(xtraj, ytraj, 'g', label='PID controller')

ax1.plot(xref, yref, 'r', label='reference')
ax1.legend()
plt.show()

#implement Twiddle tuning algorithm (coordinate ascent)
