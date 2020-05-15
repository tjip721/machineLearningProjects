
class robot: 

    def __init__(self, x, y, theta, env):
        self.x = x
        self.y = y
        self.theta = theta 
        self.world = env
        return

    def legalMove(self, delx, dely, theta):
        #Add logic to ensure move is allowed for robot model
        #Implement bicylce model
        return True
        

    def updateState(self, x, y, theta): 
        self.x = x
        self.y = y
        self.theta = theta