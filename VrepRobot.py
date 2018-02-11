
import vrep.vrep as vrep

#WARNING, this class assumes the robot is an ePuck !
class VrepRobot:
    def __init__(self, name, clientId, handle):
        self.name = name
        self.clientID = clientId
        self.handle = handle

        #get handles for the wheel motors and stop them
        _ , self.leftJointHandle = vrep.simxGetObjectHandle(self.clientID,
                                                   'ePuck_leftJoint',
                                                   vrep.simx_opmode_blocking)

        _ , self.rightJointHandle = vrep.simxGetObjectHandle(self.clientID,
                                                   'ePuck_rightJoint',
                                                   vrep.simx_opmode_blocking)

        vrep.simxSetJointTargetVelocity(self.clientID,
                                   self.leftJointHandle,
                                   0,
                                   vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetVelocity(self.clientID,
                                   self.rightJointHandle,
                                   0,
                                   vrep.simx_opmode_oneshot)

        # get handles for the proximity sensors
        self.proximitySensorsHandles = [None]*8 #empty table of size 8
        for i in range(8):
            _, self.proximitySensorsHandles[i] = vrep.simxGetObjectHandle(self.clientID,
                                                            'ePuck_proxSensor'+str(i+1),
                                                            vrep.simx_opmode_blocking)

    #send motor velocity commands
    # values degrees per second
    def setWheelsVelocity(self, leftWheel, rightWheel):
        #max theoretical : 360
        #recommanded max = 30
        vrep.simxSetJointTargetVelocity(self.clientID,
                                        self.leftJointHandle,
                                        leftWheel,
                                        vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetVelocity(self.clientID,
                                        self.rightJointHandle,
                                        rightWheel,
                                        vrep.simx_opmode_oneshot)

    #get proximity sensor values:
    #returns a table of length 8 with -1 if no detection and [detected_x, detected_y, detected_z] in the robot frame if detection
    #values in meter. Max detection range approx 0.01m
    #sign problem?
    def getProximitySensors(self):
        self.sensorValues = [-1] * 8  # empty table of size 8
        for i in range(8):
            _, a, b, _, _ = vrep.simxReadProximitySensor(self.clientID,
                                         self.proximitySensorsHandles[i],
                                         vrep.simx_opmode_streaming)#vrep.simx_opmode_buffer)
            if a :
                self.sensorValues[i] = b
        return self.sensorValues


    #get robot's center position
    #returns a table of length 3 [position_x, position_y, position_z]
    # values in meter.
    def getPosition(self):
        _, position = vrep.simxGetObjectPosition(self.clientID,
                                                   self.handle,
                                                   -1,  # retrieve absolute, not relative, position
                                                   vrep.simx_opmode_blocking)
        if _ != 0:

            raise Exception("VrepRobot could not get position")

        return position

    #get robot's orientation in euler angles
    #returns a table of length 3 [position_x, position_y, position_z]
    #values in degrees
    def getOrentation(self):
        _, angles = vrep.simxGetObjectOrientation(self.clientID,
                                                   self.handle,
                                                   -1,  # retrieve absolute, not relative, position
                                                   vrep.simx_opmode_blocking)
        if _ != 0:

            raise Exception("VrepRobot could not get orientation")

        return angles

    #get the robot's center velocity
    #returne a table of length 6 with 3 linear velocities and 3 angular velocities
    #values in meter per seconds and degrees per seconds #TO CHECK
    def getVelocity(self):
        _, linearVelocity, rotationVelocity = vrep.simxGetObjectVelocity(self.clientID,
                                                   self.handle,
                                                   vrep.simx_opmode_streaming)
        if _ != 0 and _ != 1:

            raise Exception("VrepRobot could not get velocity. Error "+str(_))
        return linearVelocity +rotationVelocity







#Epuck Main specifications	Values
#Robot radius	37 mm
#Wheel radius	20.5 mm
#Axle length	52 mm
#Encoder resolution	0.00628 rad
#Maximum angular speed	6.28 rad/s
#from https://www.cyberbotics.com/doc/guide/using-the-e-puck-robot