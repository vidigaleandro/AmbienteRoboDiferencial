from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
from Lidar import Lidar
import time
from RobotMove import RobotMove

client = RemoteAPIClient()
sim = client.require('sim')
pioneer = sim.getObject('./PioneerP3DX')
motorL = sim.getObject('./PioneerP3DX/leftMotor')
motorR = sim.getObject('./PioneerP3DX/rightMotor')
target = sim.getObject('/Target')

def getObjPos(objName):
    pos = sim.getObjectPosition(objName)
    ori = sim.getObjectOrientation(objName)
    print(ori)
    return [*pos[:2], ori[2]]
    
def setRobotSpeed(vl,vr):
    sim.setJointTargetVelocity(motorL,vl)
    sim.setJointTargetVelocity(motorR,vr)    
    
def readLidar():
    signalValue = sim.getStringSignal('hokuyo_range_data')
    signalAng = sim.getStringSignal('hokuyo_angle_data')
    dists = []
    angs = []
    if signalValue is not None:
        dists = sim.unpackFloatTable(signalValue)
        angs = sim.unpackFloatTable(signalAng)
        # dists = 4*np.ones(len(dists))
    return angs, dists


def main():
    
    sensors = []
    v0 = 1
    for i in range(10):
        # print(i)
        sensors.append(sim.getObject('./PioneerP3DX/ultrasonicSensor',[i]))
    lidar = Lidar()
    # print(sensors)
    # pos = sim.getObjectPose(cuboidH)[:3]
    # print(pos)
    # sim.setObjectPosition(cuboidH, [pos[0]+0.2,1,0])

    sim.setStepping(True)
    sim.startSimulation()
    drawingObjectHandle = sim.addDrawingObject(sim.drawing_linestrip, 3, 0.01,-1,100,  [255,0,0])
    lastPos = [0,0]
    robot = RobotMove()
    try:
        while (t := sim.getSimulationTime()) < 20:
            
            # angs, dists = readLidar()
            # lidar.update(angs, dists)
            time.sleep(0.001)
            pioneerPos = getObjPos(pioneer)
            targetPos = getObjPos(target)
          
            if np.linalg.norm(np.array(lastPos)[:2] - np.array(pioneerPos)[:2]) >0.2:
                sim.addDrawingObjectItem(drawingObjectHandle, pioneerPos[:2]+[0])
                lastPos = pioneerPos[:2]
            
            rho,vl,vr = robot.update(pioneerPos, targetPos)
            if rho > 0.01:
                setRobotSpeed(vl,vr)
        
            # print(f'Simulation time: {t:.2f} [s]')
            sim.step()
        # time.sleep(4)
     
    except KeyboardInterrupt:
        print("ECEPETI")
        
    finally:
        sim.removeDrawingObject(drawingObjectHandle)
        sim.stopSimulation()
        
    sim.removeDrawingObject(drawingObjectHandle)
    sim.stopSimulation()
    
    
if __name__ == "__main__":
    main()