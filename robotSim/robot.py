
class robot: 

    def __init__(self, x, y, theta, env):
        self.x = x
        self.y = y
        self.theta = theta 
        self.world = env
        return

    def legalMove(self, delx, dely, theta):
        #Add logic to ensure move is allowed for robot model
        x1 = self.x + delx
        y1 = self.y + dely
        if self.world.map[y1][x1] == 'x':
            return False
        #Implement bicylce model
        return True
        

    def updateState(self, x, y, theta): 
        self.x = x
        self.y = y
        self.theta = theta
        return

    #Implement kalman filter to determine robot position

    #Implement particle filter to determine robot position

    def sense(self):
        #Measure environment obstacles w/ gaussian uncertainty

        return
    
    
