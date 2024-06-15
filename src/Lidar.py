import matplotlib.pyplot as plt
import numpy as np
from math import pi
from typing import Optional

#Quero detectar discontinuidades
#Quero detectar o ponto mais proximo
#Quero detectar a tangente do ponto
#Bonus: Quero plotar o cenario e medições
class Lidar:
    def __init__(self, DEBUG=False):
        self.distMax = 0.999
        self.angRange = [-pi/2, pi/2]
        self.debug = DEBUG
        if self.debug:
            plt.ion()
            plt.show()
            plt.axis('equal')
        print("LIDAR INIT")
        
   
    
    def ss(self):
        print("SELF")
    
    def update(self, anglesX, distsX):
        angles = np.array(anglesX)
        dists = np.array(distsX)
        
    
        #Começando pelas discontinuidades obvias.      
        self.discontinuites = dists > self.distMax
        dists[self.discontinuites] = 0
        
        #Computar posições dos pontos
        x = dists*np.cos(angles)
        y = dists*np.sin(angles)
        self.positions = np.column_stack((x, y))
        
        #Computar discontinuidades
        distDeltas =  np.abs(dists[1:] - dists[:-1])
        #distDeltas = distDeltas[:,0]+distDeltas[:,1]
        # print("DELTAS" , distDeltas)
        # distDeltas = np.abs(dists[1:] - dists[:-1])
        isSolid  = False
        startPos = -1
        self.objects = []
        print("FIST" , dists[:220])
        print("REF:" , distDeltas[:220])
        for i in range(distDeltas.shape[0]):
            if  not isSolid: #Se não estou em soldio
                if not self.discontinuites[i]:
                    isSolid = True
                    startPos = i
                    self.objects.append([])
                    self.objects[-1].append(self.positions[i])

            else: #se estou solido                            

                avgDist = np.median(distDeltas[startPos : i-1])
                #segunda parte força a discontinuidade
                if self.discontinuites[i] or (distDeltas[i] > avgDist*5 and (i-startPos) >15):
                    print("obj", startPos,i)
                    startPos = -1
                    isSolid = False
                    self.discontinuites[i] = True
                else:
                    self.objects[-1].append(self.positions[i])
                
        # print("APPENDED")
        # print(self.objects)
        # print("END APP")
        if self.debug:
            self.draw()
        return self.objects
    
        
    def draw(self):
        plt.cla()
        ax = plt.gca()
        colors = ['k','g','r','b']
        print("SIZE OF OBJECTS: " , len(self.objects))
        for z,o in enumerate(self.objects):
            for i in range(len(o)):
                # print(positions[i])
                plt.plot([0,o[i][0]],[0,o[i][1]] , color=colors[z%4])
                # if ~o[i]:
                #     print("Disc", o[i-1])
                #     circle1 = plt.Circle(o[i], 0.2, color='green', fill=True)
                #     ax.add_artist(circle1)    
        
        plt.pause(0.001)
        
if __name__ == '__main__':
    
    
    lidar = Lidar()
    dist = np.linspace(2, 5, 40)
    print(dist[25])
    dist[25] = 100
    dist[26] = 100
    dist[27] = 100
    dist[10] = 1
    dist[30] = 1

    lidar.update(np.linspace(0, 3.14, 40), dist)
    lidar.draw()
        
        
            
        