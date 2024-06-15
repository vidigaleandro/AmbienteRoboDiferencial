import numpy as np

class RobotMove:
    def __init__(self):
        maxv = 1.0
        maxw = np.deg2rad(45)
    # Normalize angle to the range [-pi,pi)
    def normalizeAngle(angle):
        return np.mod(angle+np.pi, 2*np.pi) - np.pi

    def update(self,qAct, qGoal):
        L = 0.331
        r = 0.0975
        maxv = 1.0
        maxw = np.deg2rad(45)
        
        # while rho > .05:
            
        qAct = np.array(qAct)
        qGoal = np.array(qGoal)
        
        dx, dy, dth = qGoal - qAct
        theta_goal = np.arctan2(dy, dx)
        
        # Angular difference normalized
        dth = np.arctan2(np.sin(theta_goal - qAct[2]), np.cos(theta_goal - qAct[2]))

        rho = np.sqrt(dx**2 + dy**2)
        print(dth)
        kr = 1/2
        kt = 2/2
        
        v = kr*(dx*np.cos(qAct[2]) + dy*np.sin(qAct[2]))
        w = kt*(np.arctan2(dy,dx) - qAct[2])
                
        # Limit v,w to +/- max
        v = max(min(v, maxv), -maxv)
        w = max(min(w, maxw), -maxw)        
        
        vr = ((2.0*v) + (w*L))/(2.0*r)
        vl = ((2.0*v) - (w*L))/(2.0*r)
        
        return rho, vl, vr
            
