import numpy as np
from Utils import Utils
from enum import Enum

class TBState(Enum):
    MoveToGoal = 1
    PassiveContour = 2
    ActiveContour = 3
    GoalReached = 4
    Failure = 5


class TanBug:
    def __init__(self, goalPos):
        self.goalPos = goalPos
        self.state = TBState.MoveToGoal
        self.dreach = 100000
        self.dfollowed = 100000
        self.mPos = np.array([0,0])
        self.minRadius = 0.5
        
        self.max_attempts = 100  # Limita o número de iterações para evitar loops infinitos
        self.attempts = 0
        self.currentPos = np.array([0,0])
        
        self.lastPos = self.currentPos
        self.nextPos = np.array([0,0])
    
    
    def calculate_dreach(self, point):
        return np.linalg.norm(point - self.goalPos)

    def calculate_dfollowed(self, point):
        return np.linalg.norm(point - self.mPos)

    def find_best_point(self, points):
        best_point = None
        min_heuristic = float('inf')
        
        for group in points:
            for point in group:
                d_reach = self.calculate_dreach(np.array(point))
                d_followed = self.calculate_dfollowed(np.array(point))
                heuristic = d_reach + d_followed
                
                if heuristic < min_heuristic:
                    min_heuristic = heuristic
                    best_point = point
                    
        return best_point, min_heuristic



    def update(self, robotPos, lidar):
      
        self.lastPos = self.currentPos
        self.currentPos = robotPos
        
        if self.state != TBState.GoalReached and self.state != TBState.Failure:
            self.attempts += 1
            if self.attempts > self.max_attempts:
                print("Falha ao alcançar o objetivo após muitas tentativas.")
                self.state = TBState.Failure
                

            if self.state == TBState.MoveToGoal:
                if self.detect_obstacles():
                    self.state = TBState.PassiveContour
                elif np.linalg.norm(self.current_pos - self.goal_pos) <= 0.1:
                    self.state = TBState.GoalReached
                else:
                    return self.goal_pos

            elif self.state == TBState.PassiveContour or self.state == TBState.ActiveContour:
                d_followed = d_followed + Utils.dist(self.currentPos,self.lastPos)
                d_reach = Utils.dist(self.goal_pos, best_path)
                if d_reach > d_followed :
                    self.state = TBState.ActiveContour
                else:
                    self.state = TBState.PassiveContour
                    
                

        if self.state == TBState.GoalReached:
            print("Objetivo alcançado com sucesso.")
        elif self.state == TBState.Failure:
            print("Falha ao alcançar o objetivo.")
            
        return self.state, self.nextPos
        