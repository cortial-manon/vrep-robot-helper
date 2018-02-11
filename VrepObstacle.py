import vrep.vrep as vrep

#
class VrepObstacle:
    def __init__(self, name, clientId, handle):
        self.name = name
        self.clientID = clientId
        self.handle = handle

        self.size = [0.1, 0.1, 0.2]
        self.position = [0., 0.]


    def moveTo(self, dx, dy):
        self.position = [dx, dy]
        vrep.simxSetObjectPosition(self.clientID,
                                   self.handle,
                                   -1,#world coordinates
                                   [dx, dy, self.size[2]/2],
                                   vrep.simx_opmode_oneshot)


