

#based on the code from https://github.com/studywolf/blog/blob/master/VREP/two_link_arm/vrep_twolink_controller.py
#explained at https://studywolf.wordpress.com/2016/04/18/using-vrep-for-simulation-of-force-controlled-models/

import numpy as np


from VrepWorld import VrepWorld

#create the world object
world = VrepWorld()

#connect to vrep server
world.init()#scene="scenes\\ePuck_wall.ttt") #remote scene import not working at the moment

#create robot object linked to ePuck
robot = world.getRobot('ePuck')

#create obstacles
obstacle1 = world.setObstacle(0.3, 0.2)
obstacle2 = world.setObstacle(0.1, 0.25)

#table for storing positions of the robot
positions = []

try:
    #start simulation
    world.startRun(5)

    robot.setWheelsVelocity(1, 1)

    while not world.runFinished:
        #robot.getProximitySensors()
        #robotVelocity = robot.getVelocity()
        robotPosition = robot.getPosition()
        positions.append(np.copy(robotPosition))  # store for plotting

        #update simulation
        world.run()

    #clean simulation
    world.endRun()

finally:
    #close connection even if we got an exception
    world.close()


#plot robot positions
import matplotlib.pyplot as plt

positions = np.array(positions)

plt.plot(positions[:, 0], positions[:, 1], 'rx', label="Position de l'ePuck")
plt.axis([0, 1, 0,1])
plt.show()