#!/usr/bin/env python
# coding: utf-8
import numpy as np
import math
from actuator import servo

# 回転軸の方向ベクトル
# 元座標からみた相対座標系における回転軸方向のベクトルを表す
class Omega:
    def __init__(self, Lam=0, Mu=0, Nu=0, Phi=0, Theta=0, Psi=0):
        self.Lam, self.Mu, self.Nu, self.Phi, self.Theta, self.Psi = Lam, Mu, Nu, Phi, Theta, Psi
    def subsVal(self, Lam, Mu, Nu):
        self.Lam, self.Mu, self.Nu = Lam, Mu, Nu
    def subsEuler(self, Phi, Theta, Psi):
        self.Phi, self.Theta, self.Psi = Phi, Theta, Psi

# サーボを3つ使う
class Leg:
    def __init__(self, *servos):
        self.servos = servos

    def setLinkLength(self, link_len):
        self.__L = link_len
        print("link length is ", self.__L)
    
    def setAxis(self, omega):
        self.__omega = omega

    def setupNowTheta(self):
        self.__now_theta = np.array([[self.servos[0].getTargetTheta()], [self.servos[1].getTargetTheta()], [self.servos[2].getTargetTheta()]]) 

    def calcJacob(self, theta):
        return np.asarray([[0, -self.__L[2]*math.sin(theta[1]) + self.__L[3]*(-math.sin(theta[1])*math.cos(theta[2]) - math.sin(theta[2])*math.cos(theta[1])), self.__L[3]*(-math.sin(theta[1])*math.cos(theta[2]) - math.sin(theta[2])*math.cos(theta[1]))], 
                            [self.__L[2]*math.sin(theta[1])*math.cos(theta[0]) + self.__L[3]*(math.sin(theta[1])*math.cos(theta[0])*math.cos(theta[2]) + math.sin(theta[2])*math.cos(theta[0])*math.cos(theta[1])), self.__L[2]*math.sin(theta[0])*math.cos(theta[1]) + self.__L[3]*(-math.sin(theta[0])*math.sin(theta[1])*math.sin(theta[2]) + math.sin(theta[0])*math.cos(theta[1])*math.cos(theta[2])), self.__L[3]*(-math.sin(theta[0])*math.sin(theta[1])*math.sin(theta[2]) + math.sin(theta[0])*math.cos(theta[1])*math.cos(theta[2]))], 
                            [self.__L[2]*math.sin(theta[0])*math.sin(theta[1]) + self.__L[3]*(math.sin(theta[0])*math.sin(theta[1])*math.cos(theta[2]) + math.sin(theta[0])*math.sin(theta[2])*math.cos(theta[1])), -self.__L[2]*math.cos(theta[0])*math.cos(theta[1]) + self.__L[3]*(math.sin(theta[1])*math.sin(theta[2])*math.cos(theta[0]) - math.cos(theta[0])*math.cos(theta[1])*math.cos(theta[2])), self.__L[3]*(math.sin(theta[1])*math.sin(theta[2])*math.cos(theta[0]) - math.cos(theta[0])*math.cos(theta[1])*math.cos(theta[2]))]])

    def calcPose(self, theta):
        return np.asarray([[self.__L[1] + self.__L[2]*math.cos(theta[1]) + self.__L[3]*(-math.sin(theta[1])*math.sin(theta[2]) + math.cos(theta[1])*math.cos(theta[2]))], 
                            [self.__L[2]*math.sin(theta[0])*math.sin(theta[1]) + self.__L[3]*(math.sin(theta[0])*math.sin(theta[1])*math.cos(theta[2]) + math.sin(theta[0])*math.sin(theta[2])*math.cos(theta[1]))], 
                            [-self.__L[2]*math.sin(theta[1])*math.cos(theta[0]) + self.__L[3]*(-math.sin(theta[1])*math.cos(theta[0])*math.cos(theta[2]) - math.sin(theta[2])*math.cos(theta[0])*math.cos(theta[1]))]])


    # 逆運動学計算し目標角度になるように更新
    def calcServosDeg(self, delta_pose):
        J = self.calcJacob(self.__now_theta)
        print(J)
        inv_J = np.linalg.pinv(J)
        delta_theta = np.dot(inv_J, delta_pose)
        target_theta = self.__now_theta + delta_theta
        # each joint
        for i in range(3):
            self.servos[i].setTargetTheta(target_theta[i][0])
        print("hello",self.getEndEfectorPose())
        self.__now_theta = np.array([[self.servos[0].getTargetTheta()], [self.servos[1].getTargetTheta()], [self.servos[2].getTargetTheta()]]) 

    # 脚先座標の取得
    def getEndEfectorPose(self):
        pos = self.calcPose(self.__now_theta)
        #print(pos)
        #print(self.__now_theta)
        return pos

    # 可操作度を計算して取得
    def getEndEffectorManipulability(self):
        J = self.calcJacob(self.__now_theta)
        #w = np.sqrt(np.linalg.det(J * J.T))
        w = math.sqrt(abs(np.linalg.det(J * J.T)))
        return w

    # ヤコビ行列のみを取得
    def getJacobByStep(self):
        return self.calcJacob(self.__now_theta)

    def getServosDeg(self):
        return [self.servos[0].getTargetDegree(), self.servos[1].getTargetDegree(), self.servos[2].getTargetDegree()]

    def send(self):
        for servo in self.servos:
            servo.send()
