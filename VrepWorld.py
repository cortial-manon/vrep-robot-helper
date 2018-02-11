
# python remote API reference : http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm


import vrep.vrep as vrep
import os

from VrepObstacle import VrepObstacle
from VrepRobot import VrepRobot


class VrepWorld:
    def __init__(self):
        self.clientId = -1          #id for communication with vrep
        self.runFinished = True     #False is simulation is running
        self.remainingRunTime = 0   #remaining time (seconds)
        self.dt = 0                 #time interval between two simulations steps (seconds)

        self.obstacles = []


    #connect to vrep and get clientID
    def init(self, scene=""):
        # close any open connections
        vrep.simxFinish(-1)
        # Connect to the V-REP continuous server
        self.clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 500, 5)

        if self.clientID == -1:  # connection error
            raise Exception('VrepWorld : Failed connecting to remote API server')


        vrep.simxSynchronous(self.clientID, True)
        print('Connected to remote API server')


        ##### NOT WORKING YET ###
        # simxLoadScene
        # clientID: the client ID. refer to simxStart.
        # scenePathAndName: the scene filename, including the path and extension ("ttt"). The file is relative to the client or server system depending on the options value (see next argument)
        # options: options, bit-coded:
        # bit0 set: the specified file is located on the client side (in that case the function will be blocking since the scene first has to be transferred to the server). Otherwise it is located on the server side
        # operationMode: a remote API function operation mode. Recommended operation mode for this function is simx_opmode_blocking
        if scene !="":
            scenePath = os.path.join(os.path.dirname(__file__), scene)
            print(scenePath)
            err = vrep.simxLoadScene(self.clientID,scenePath,1,vrep.simx_opmode_blocking )
            if err != 0:
                raise Exception("VrepRobot could load scene. Error "+str(err))
            print('Scene loaded')



    #start simulation

    # warning, changing dt may cause simulation problems !
    def startRun(self, time, dt = 0.05):
        self.runFinished = False
        self.remainingRunTime = time
        self.dt = dt

        vrep.simxSetFloatingParameter(self.clientID,
                                      vrep.sim_floatparam_simulation_time_step,
                                      dt,
                                      vrep.simx_opmode_oneshot)

        # start our simulation in lockstep with our code
        vrep.simxStartSimulation(self.clientID,
                                 vrep.simx_opmode_blocking)


    #run one simulation step
    def run(self):
        if self.remainingRunTime < 0:
            raise Exception('VrepWorld : run time exceeded')

        vrep.simxSynchronousTrigger(self.clientID)
        self.remainingRunTime -= self.dt
        self.runFinished = self.remainingRunTime <0

    #remove obstacles and stop simulation
    def endRun(self):
        for obs in self.obstacles:
            vrep.simxRemoveModel(self.clientID, obs, vrep.simx_opmode_blocking)
        # stop the simulation
        vrep.simxStopSimulation(self.clientID, vrep.simx_opmode_blocking)

    #close connection to the server
    def close(self):
        # Before closing the connection to V-REP,
        # make sure that the last command sent out had time to arrive.
        vrep.simxGetPingTime(self.clientID)

        # Now close the connection to V-REP:
        vrep.simxFinish(self.clientID)
        print('connection closed...')


    #create a VrepRobot object with the correct handle
    def getRobot(self, name):
        if name not in ['ePuck']:
            raise Exception('VrepWorld : no robot named '+name)

        _, robot_handle = vrep.simxGetObjectHandle(self.clientID,
                                                   name,
                                                   vrep.simx_opmode_blocking)
        if _ != 0:
            raise Exception("VrepRobot could not get robot")

        return VrepRobot(name, self.clientID, robot_handle)

    # duplicate TestObstacle and put it at position (dx, dy)
    #store handle in self.obstacles
    #WARNING : works only on non-dynamic objects !
    def setObstacle(self, dx, dy):
        _, handle = vrep.simxGetObjectHandle(self.clientID,
                                                   'ObstacleTest',
                                                    vrep.simx_opmode_blocking)
        _, newHandle = vrep.simxCopyPasteObjects(self.clientID,
                                                 [handle],
                                                 vrep.simx_opmode_blocking)
        self.obstacles.append(newHandle[0])

        obs =  VrepObstacle("Obstacle"+str(len(self.obstacles)-1), self.clientID, self.obstacles[-1])
        obs.moveTo(dx, dy)

        return obs


