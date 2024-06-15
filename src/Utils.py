import numpy as np
from typing import Optional
from numpy.typing import NDArray
import numpy.typing as npt
class Utils:
    def getTangentFromAngle(angle:float, distance:float, direction : str= 'CCW'):
        # Converte o ângulo de graus para radianos
        # Calcula as componentes x e y do vetor
        x = distance * np.cos(angle)
        y = distance * np.sin(angle)
        vector = np.array([x, y])
        print("Vector from point", vector)
        return Utils.getTangentFromXY([x,y],direction)

    def getTangentFromXY(vect : list[float] | npt.NDArray, direction : str= 'CCW'):
        # Converte o ângulo de graus para radianos
        # Calcula as componentes x e y do vetor
        vect = np.array(vect)
        print(vect,np.linalg.norm(vect))
        
        vect = vect/np.linalg.norm(vect)
        perpendicular = np.ones([1,1])
        if direction == 'CCW':
            perpendicular = np.array([-vect[1], vect[0]])
        elif direction == 'CW':
            perpendicular = np.array([vect[1], -vect[0]])

        return perpendicular
    
    def getVectorAtDist(p: NDArray[np.float64], distance):
        d = np.linalg.norm(p)
        print('dist' ,d)
        t = distance/d
        return p*(1-t)
        
    def dist(self, p1,p2):
            return np.linalg.norm(p1-p2)
    def transform(p: NDArray[np.float64], x: float, y: float, ang: float) -> NDArray[np.float64]:
        # Matriz de rotação
        T = np.array([[np.cos(ang), -np.sin(ang)], [np.sin(ang), np.cos(ang)]])
        # Converter p para array numpy se ainda não for
        p = np.array(p)
        # Verifica se p é uma lista de coordenadas e reformula se necessário
        if p.ndim == 1:
            p = p.reshape(1, -1)
        # Vetor de deslocamento
        offset = np.array([x, y])
        # Aplica a rotação e adiciona o deslocamento
        return np.dot(T, p.T).T + offset

if __name__ == '__main__':
    
    print(Utils.getVectorAt(np.array([0,100]),1))
