#!/usr/bin/env python
# coding: utf-8
import rospy
import time
import numpy as np
from robot.leg.actuator import servo
from robot.leg import leg
from robot.leg.leg import Omega
from robot.end_efector_manager import EndEfectorManager
from robot.legged_robot import LeggedRobot
from std_msgs.msg import Float64, Bool, String
import math
import xml.etree.ElementTree as ET
import rospkg
import configparser
from sympy import Matrix
import sys

def XacroToRPY(root, rpy_name, variable_name_list, is_right = False):
    variable_rpy = []
    rpy = str()
    for variable_name in variable_name_list:
        for property in root.iter('property'):
            if property.attrib['name'] == rpy_name:
                rpy = property.attrib['value']
            elif property.attrib['name'] == variable_name:
                variable_rpy.append(property.attrib['value'])  
    rpy_list_str = rpy.replace('${', '').replace('}', '').split(' ') 
    rpy_list = []
    # left case
    if not is_right == True:
        rpy_list.append(float(variable_rpy[0]))
        rpy_list.append(float(variable_rpy[1]))
        rpy_list.append(float(variable_rpy[2]))
    # right case
    else:
        rpy_list.append(-float(variable_rpy[0]))
        rpy_list.append(float(variable_rpy[1]))
        rpy_list.append(-float(variable_rpy[2]))
    print(rpy_list)
    return rpy_list

class RobotLegPublisher:
    def __init__(self, name=""):
        print("publisher name: ", name)
        self.pub_BLF_base_clause = rospy.Publisher(name+'/BLF_base_clause_controller/command', Float64, queue_size=10)
        self.pub_BLF_thigh = rospy.Publisher(name+'/BLF_thigh_controller/command', Float64, queue_size=10)
        self.pub_BLF_tibia = rospy.Publisher(name+'/BLF_tibia_controller/command', Float64, queue_size=10)
        self.pub_BLH_base_clause = rospy.Publisher(name+'/BLH_base_clause_controller/command', Float64, queue_size=10)
        self.pub_BLH_thigh = rospy.Publisher(name+'/BLH_thigh_controller/command', Float64, queue_size=10)
        self.pub_BLH_tibia = rospy.Publisher(name+'/BLH_tibia_controller/command', Float64, queue_size=10)
        self.pub_BRF_base_clause = rospy.Publisher(name+'/BRF_base_clause_controller/command', Float64, queue_size=10)
        self.pub_BRF_thigh = rospy.Publisher(name+'/BRF_thigh_controller/command', Float64, queue_size=10)
        self.pub_BRF_tibia = rospy.Publisher(name+'/BRF_tibia_controller/command', Float64, queue_size=10)
        self.pub_BRH_base_clause = rospy.Publisher(name+'/BRH_base_clause_controller/command', Float64, queue_size=10)
        self.pub_BRH_thigh = rospy.Publisher(name+'/BRH_thigh_controller/command', Float64, queue_size=10)
        self.pub_BRH_tibia = rospy.Publisher(name+'/BRH_tibia_controller/command', Float64, queue_size=10)

        self.pub_TLF_base_clause = rospy.Publisher(name+'/TLF_base_clause_controller/command', Float64, queue_size=10)
        self.pub_TLF_thigh = rospy.Publisher(name+'/TLF_thigh_controller/command', Float64, queue_size=10)
        self.pub_TLF_tibia = rospy.Publisher(name+'/TLF_tibia_controller/command', Float64, queue_size=10)
        self.pub_TLH_base_clause = rospy.Publisher(name+'/TLH_base_clause_controller/command', Float64, queue_size=10)
        self.pub_TLH_thigh = rospy.Publisher(name+'/TLH_thigh_controller/command', Float64, queue_size=10)
        self.pub_TLH_tibia = rospy.Publisher(name+'/TLH_tibia_controller/command', Float64, queue_size=10)
        self.pub_TRF_base_clause = rospy.Publisher(name+'/TRF_base_clause_controller/command', Float64, queue_size=10)
        self.pub_TRF_thigh = rospy.Publisher(name+'/TRF_thigh_controller/command', Float64, queue_size=10)
        self.pub_TRF_tibia = rospy.Publisher(name+'/TRF_tibia_controller/command', Float64, queue_size=10)
        self.pub_TRH_base_clause = rospy.Publisher(name+'/TRH_base_clause_controller/command', Float64, queue_size=10)
        self.pub_TRH_thigh = rospy.Publisher(name+'/TRH_thigh_controller/command', Float64, queue_size=10)
        self.pub_TRH_tibia = rospy.Publisher(name+'/TRH_tibia_controller/command', Float64, queue_size=10)
    
    def publish(self, BLF_servos, BLH_servos, BRF_servos, BRH_servos, TLF_servos, TLH_servos, TRF_servos, TRH_servos):
        self.pub_BLF_base_clause.publish(math.radians(BLF_servos[0].getTargetDegree()))
        self.pub_BLF_thigh.publish(math.radians(BLF_servos[1].getTargetDegree()))
        self.pub_BLF_tibia.publish(math.radians(BLF_servos[2].getTargetDegree()))
        self.pub_BLH_base_clause.publish(math.radians(BLH_servos[0].getTargetDegree()))
        self.pub_BLH_thigh.publish(math.radians(BLH_servos[1].getTargetDegree()))
        self.pub_BLH_tibia.publish(math.radians(BLH_servos[2].getTargetDegree()))
        self.pub_BRF_base_clause.publish(math.radians(BRF_servos[0].getTargetDegree()))
        self.pub_BRF_thigh.publish(math.radians(BRF_servos[1].getTargetDegree()))
        self.pub_BRF_tibia.publish(math.radians(BRF_servos[2].getTargetDegree()))
        self.pub_BRH_base_clause.publish(math.radians(BRH_servos[0].getTargetDegree()))
        self.pub_BRH_thigh.publish(math.radians(BRH_servos[1].getTargetDegree()))
        self.pub_BRH_tibia.publish(math.radians(BRH_servos[2].getTargetDegree()))
        
        self.pub_TLF_base_clause.publish(math.radians(TLF_servos[0].getTargetDegree()))
        self.pub_TLF_thigh.publish(math.radians(TLF_servos[1].getTargetDegree()))
        self.pub_TLF_tibia.publish(math.radians(TLF_servos[2].getTargetDegree()))
        self.pub_TLH_base_clause.publish(math.radians(TLH_servos[0].getTargetDegree()))
        self.pub_TLH_thigh.publish(math.radians(TLH_servos[1].getTargetDegree()))
        self.pub_TLH_tibia.publish(math.radians(TLH_servos[2].getTargetDegree()))
        self.pub_TRF_base_clause.publish(math.radians(TRF_servos[0].getTargetDegree()))
        self.pub_TRF_thigh.publish(math.radians(TRF_servos[1].getTargetDegree()))
        self.pub_TRF_tibia.publish(math.radians(TRF_servos[2].getTargetDegree()))
        self.pub_TRH_base_clause.publish(math.radians(TRH_servos[0].getTargetDegree()))
        self.pub_TRH_thigh.publish(math.radians(TRH_servos[1].getTargetDegree()))
        self.pub_TRH_tibia.publish(math.radians(TRH_servos[2].getTargetDegree()))

class flagInterface:
    def __init__(self):
        self.__flag = False
    @property
    def flag(self):
        return self.__flag
    @flag.setter
    def flag(self, flag):
        self.__flag = flag

if __name__ == '__main__':
    walk_flag = flagInterface()
    # is_straight: True=直進モード, False=その場旋回モード
    is_straight = flagInterface()

    def walk_flag_subscribe(msg):
        if msg.data == "False":
            walk_flag.flag = False
        elif not walk_flag.flag and msg.data.endswith("straight"):
            print("Receive Straight Command")
            walk_flag.flag = True
            is_straight.flag = True
        elif not walk_flag.flag and msg.data.endswith("turn"):
            print("Receive Turn Command")
            walk_flag.flag = True
            is_straight.flag = False

    rospack = rospkg.RosPack()
    #tree = ET.parse(rospack.get_path('lily_octpus_description') + '/urdf/params/params.xacro')
    tree = ET.parse(rospack.get_path('lily_octpus_description') + '/urdf/params/robot.xacro')
    root = tree.getroot()

    # xarcoにおける"name"を入力すると"value"を検索するラムダ式の作成
    list_atr = [child.attrib for child in root]
    searched_from_xacro = lambda name: [s for s in list_atr if s['name'] == name][0]['value']
    # 胴体の幅
    BODY_WIDTH = float(searched_from_xacro('body_width'))
    # 各脚の各長さ
    LF_LINK1_LENGTH = float(searched_from_xacro('base_clause_length'))
    LF_LINK2_LENGTH = float(searched_from_xacro('thigh_length'))
    LF_LINK3_LENGTH = float(searched_from_xacro('tibia_length'))
    LF_RPY_1 = XacroToRPY(root, "LF_base_clause_rpy", ["F_base_clause_roll", "F_base_clause_pitch", "F_base_clause_yaw"])
    LF_RPY_2 = XacroToRPY(root, "LF_thigh_rpy", ["F_thigh_roll", "F_thigh_pitch", "F_thigh_yaw"])
    LF_RPY_3 = XacroToRPY(root, "LF_tibia_rpy", ["F_tibia_roll", "F_tibia_pitch", "F_tibia_yaw"])
    LF_AXIS_1 = Omega(Phi=LF_RPY_1[0], Theta=LF_RPY_1[1], Psi=LF_RPY_1[2])
    LF_AXIS_2 = Omega(Phi=LF_RPY_2[0], Theta=LF_RPY_2[1], Psi=LF_RPY_2[2])
    LF_AXIS_3 = Omega(Phi=LF_RPY_3[0], Theta=LF_RPY_3[1], Psi=LF_RPY_3[2])
    RF_LINK1_LENGTH = float(searched_from_xacro('base_clause_length'))
    RF_LINK2_LENGTH = float(searched_from_xacro('thigh_length'))
    RF_LINK3_LENGTH = float(searched_from_xacro('tibia_length'))
    RF_RPY_1 = XacroToRPY(root, "RF_base_clause_rpy", ["F_base_clause_roll", "F_base_clause_pitch", "F_base_clause_yaw"], True)
    RF_RPY_2 = XacroToRPY(root, "RF_thigh_rpy", ["F_thigh_roll", "F_thigh_pitch", "F_thigh_yaw"], True)
    RF_RPY_3 = XacroToRPY(root, "RF_tibia_rpy", ["F_tibia_roll", "F_tibia_pitch", "F_tibia_yaw"], True)
    RF_AXIS_1 = Omega(Phi=RF_RPY_1[0], Theta=RF_RPY_1[1], Psi=RF_RPY_1[2])
    RF_AXIS_2 = Omega(Phi=RF_RPY_2[0], Theta=RF_RPY_2[1], Psi=RF_RPY_2[2])
    RF_AXIS_3 = Omega(Phi=RF_RPY_3[0], Theta=RF_RPY_3[1], Psi=RF_RPY_3[2])
    LH_LINK1_LENGTH = float(searched_from_xacro('base_clause_length'))
    LH_LINK2_LENGTH = float(searched_from_xacro('thigh_length'))
    LH_LINK3_LENGTH = float(searched_from_xacro('tibia_length'))
    LH_RPY_1 = XacroToRPY(root, "LH_base_clause_rpy", ["H_base_clause_roll", "H_base_clause_pitch", "H_base_clause_yaw"])
    LH_RPY_2 = XacroToRPY(root, "LH_thigh_rpy", ["H_thigh_roll", "H_thigh_pitch", "H_thigh_yaw"])
    LH_RPY_3 = XacroToRPY(root, "LH_tibia_rpy", ["H_tibia_roll", "H_tibia_pitch", "H_tibia_yaw"])
    LH_AXIS_1 = Omega(Phi=LH_RPY_1[0], Theta=LH_RPY_1[1], Psi=LH_RPY_1[2])
    LH_AXIS_2 = Omega(Phi=LH_RPY_2[0], Theta=LH_RPY_2[1], Psi=LH_RPY_2[2])
    LH_AXIS_3 = Omega(Phi=LH_RPY_3[0], Theta=LH_RPY_3[1], Psi=LH_RPY_3[2])
    RH_LINK1_LENGTH = float(searched_from_xacro('base_clause_length'))
    RH_LINK2_LENGTH = float(searched_from_xacro('thigh_length'))
    RH_LINK3_LENGTH = float(searched_from_xacro('tibia_length'))
    RH_RPY_1 = XacroToRPY(root, "RH_base_clause_rpy", ["H_base_clause_roll", "H_base_clause_pitch", "H_base_clause_yaw"], True)
    RH_RPY_2 = XacroToRPY(root, "RH_thigh_rpy", ["H_thigh_roll", "H_thigh_pitch", "H_thigh_yaw"], True)
    RH_RPY_3 = XacroToRPY(root, "RH_tibia_rpy", ["H_tibia_roll", "H_tibia_pitch", "H_tibia_yaw"], True)
    RH_AXIS_1 = Omega(Phi=RH_RPY_1[0], Theta=RH_RPY_1[1], Psi=RH_RPY_1[2])
    RH_AXIS_2 = Omega(Phi=RH_RPY_2[0], Theta=RH_RPY_2[1], Psi=RH_RPY_2[2])
    RH_AXIS_3 = Omega(Phi=RH_RPY_3[0], Theta=RH_RPY_3[1], Psi=RH_RPY_3[2])
    
    #### ロボットモデルの作成 ####
    ## 各脚の設定
    ## BLF
    BLF_servos = [
        servo.ServoMotor(), 
        servo.ServoMotor(), 
        servo.ServoMotor(), ]
    BLF_leg = leg.Leg(*BLF_servos)
    BLF_leg.setLegType("BLF")
    BLF_leg.setLinkLength([0, LF_LINK1_LENGTH, LF_LINK2_LENGTH, LF_LINK3_LENGTH])
    BLF_leg.setAxis([LF_AXIS_1, LF_AXIS_2, LF_AXIS_3])
    ## BLH
    BLH_servos = [
        servo.ServoMotor(), 
        servo.ServoMotor(), 
        servo.ServoMotor(), ]
    BLH_leg = leg.Leg(*BLH_servos)
    BLH_leg.setLegType("BLH")
    BLH_leg.setLinkLength([0, LH_LINK1_LENGTH, LH_LINK2_LENGTH, LH_LINK3_LENGTH])
    BLH_leg.setAxis([LH_AXIS_1, LH_AXIS_2, LH_AXIS_3])
    ## BRF
    BRF_servos = [
        servo.ServoMotor(), 
        servo.ServoMotor(), 
        servo.ServoMotor(), ]
    BRF_leg = leg.Leg(*BRF_servos)
    BRF_leg.setLegType("BRF")
    BRF_leg.setLinkLength([0, RF_LINK1_LENGTH, RF_LINK2_LENGTH, RF_LINK3_LENGTH])
    BRF_leg.setAxis([RF_AXIS_1, RF_AXIS_2, RF_AXIS_3])
    ## BRH
    BRH_servos = [
        servo.ServoMotor(), 
        servo.ServoMotor(), 
        servo.ServoMotor(), ]
    BRH_leg = leg.Leg(*BRH_servos)
    BRH_leg.setLegType("BRH")
    BRH_leg.setLinkLength([0, RH_LINK1_LENGTH, RH_LINK2_LENGTH, RH_LINK3_LENGTH])
    BRH_leg.setAxis([RH_AXIS_1, RH_AXIS_2, RH_AXIS_3])
    ## TLF
    TLF_servos = [
        servo.ServoMotor(), 
        servo.ServoMotor(), 
        servo.ServoMotor(), ]
    TLF_leg = leg.Leg(*TLF_servos)
    TLF_leg.setLegType("TLF")
    TLF_leg.setLinkLength([0, LF_LINK1_LENGTH, LF_LINK2_LENGTH, LF_LINK3_LENGTH])
    TLF_leg.setAxis([LF_AXIS_1, LF_AXIS_2, LF_AXIS_3])
    ## TLH
    TLH_servos = [
        servo.ServoMotor(), 
        servo.ServoMotor(), 
        servo.ServoMotor(), ]
    TLH_leg = leg.Leg(*TLH_servos)
    TLH_leg.setLegType("TLH")
    TLH_leg.setLinkLength([0, LH_LINK1_LENGTH, LH_LINK2_LENGTH, LH_LINK3_LENGTH])
    TLH_leg.setAxis([LH_AXIS_1, LH_AXIS_2, LH_AXIS_3])
    ## TRF
    TRF_servos = [
        servo.ServoMotor(), 
        servo.ServoMotor(), 
        servo.ServoMotor(), ]
    TRF_leg = leg.Leg(*TRF_servos)
    TRF_leg.setLegType("TRF")
    TRF_leg.setLinkLength([0, RF_LINK1_LENGTH, RF_LINK2_LENGTH, RF_LINK3_LENGTH])
    TRF_leg.setAxis([RF_AXIS_1, RF_AXIS_2, RF_AXIS_3])
    ## TRH
    TRH_servos = [
        servo.ServoMotor(), 
        servo.ServoMotor(), 
        servo.ServoMotor(), ]
    TRH_leg = leg.Leg(*TRH_servos)
    TRH_leg.setLegType("TRH")
    TRH_leg.setLinkLength([0, RH_LINK1_LENGTH, RH_LINK2_LENGTH, RH_LINK3_LENGTH])
    TRH_leg.setAxis([RH_AXIS_1, RH_AXIS_2, RH_AXIS_3])

    rospy.init_node('lily_octpus_walk', anonymous=True)

    RATE = 30
    print("RATE")
    print(RATE)
    r = rospy.Rate(RATE) # hz
    robotLegPublisher = RobotLegPublisher('')
    pub_walk_standby = rospy.Publisher(''+'/is_walk_standby', Bool, queue_size=1)

    ## 計算した角度をgazeboに出力
    def pub_pose():
        ## 計算した角度をgazeboに出力
        robotLegPublisher.publish(BLF_servos, BLH_servos, BRF_servos, BRH_servos, TLF_servos, TLH_servos, TRF_servos, TRH_servos)
        r.sleep()

    ## サーボオフセット角度の直接代入
    def setInitServoDeg(servos, list_init_degrees):
        for i in range(3):
            servos[i].setOffsetDegree(list_init_degrees[i][0])
    
    ## サーボ角度の直接代入
    def setServoDeg(servos, list_degrees):
        for i in range(3):
            servos[i].setTargetDegree(list_degrees[i][0])

    deg1 = 0
    deg2 = -80
    deg3 = 120
    deg4 = 45
    deg5 = -135
    setServoDeg(BLF_servos, [[deg1],[deg2],[deg3]])
    setServoDeg(BLH_servos, [[deg1],[deg2],[deg3]])
    setServoDeg(BRF_servos, [[deg1],[deg2],[deg3]])
    setServoDeg(BRH_servos, [[deg1],[deg2],[deg3]])
    setServoDeg(TLF_servos, [[deg1],[deg4],[deg5]])
    setServoDeg(TLH_servos, [[deg1],[deg4],[deg5]])
    setServoDeg(TRF_servos, [[deg1],[deg4],[deg5]])
    setServoDeg(TRH_servos, [[deg1],[deg4],[deg5]])
    # 脚に対してサーボユニット情報を更新
    BLF_leg.setupNowTheta()
    BLH_leg.setupNowTheta()
    BRF_leg.setupNowTheta()
    BRH_leg.setupNowTheta()
    TLF_leg.setupNowTheta()
    TLH_leg.setupNowTheta()
    TRF_leg.setupNowTheta()
    TRH_leg.setupNowTheta()

    pub_pose()
    rospy.sleep(1)
    pub_pose()
    rospy.sleep(1)

    # 脚座標系で目標座標を分割して計算
    while True:
        up_dist = 0.2
        start_pose_BLF = BLF_leg.getEndEfectorPose()
        start_pose_BLH = BLH_leg.getEndEfectorPose()
        start_pose_BRF = BRF_leg.getEndEfectorPose()
        start_pose_BRH = BRH_leg.getEndEfectorPose()
        start_pose_TLF = TLF_leg.getEndEfectorPose()
        start_pose_TLH = TLH_leg.getEndEfectorPose()
        start_pose_TRF = TRF_leg.getEndEfectorPose()
        start_pose_TRH = TRH_leg.getEndEfectorPose()
        target_pose_BLF = start_pose_BLF + Matrix([[0.0], [0], [-up_dist]])
        target_pose_BLH = start_pose_BLH + Matrix([[0.0], [0], [-up_dist]])
        target_pose_BRF = start_pose_BRF + Matrix([[0.0], [0], [-up_dist]])
        target_pose_BRH = start_pose_BRH + Matrix([[0.0], [0], [-up_dist]])
        target_pose_TLF = start_pose_TLF + Matrix([[0.0], [0], [up_dist]])
        target_pose_TLH = start_pose_TLH + Matrix([[0.0], [0], [up_dist]])
        target_pose_TRF = start_pose_TRF + Matrix([[0.0], [0], [up_dist]])
        target_pose_TRH = start_pose_TRH + Matrix([[0.0], [0], [up_dist]])
        max_step = 60
        for step in range(max_step):
            tar_pose_matrix_BLF = (target_pose_BLF - start_pose_BLF) * (float(step + 1) /  float(max_step)) + start_pose_BLF
            tar_pose_matrix_BLH = (target_pose_BLH - start_pose_BLH) * (float(step + 1) /  float(max_step)) + start_pose_BLH
            tar_pose_matrix_BRF = (target_pose_BRF - start_pose_BRF) * (float(step + 1) /  float(max_step)) + start_pose_BRF
            tar_pose_matrix_BRH = (target_pose_BRH - start_pose_BRH) * (float(step + 1) /  float(max_step)) + start_pose_BRH
            tar_pose_matrix_TLF = (target_pose_TLF - start_pose_TLF) * (float(step + 1) /  float(max_step)) + start_pose_TLF
            tar_pose_matrix_TLH = (target_pose_TLH - start_pose_TLH) * (float(step + 1) /  float(max_step)) + start_pose_TLH
            tar_pose_matrix_TRF = (target_pose_TRF - start_pose_TRF) * (float(step + 1) /  float(max_step)) + start_pose_TRF
            tar_pose_matrix_TRH = (target_pose_TRH - start_pose_TRH) * (float(step + 1) /  float(max_step)) + start_pose_TRH
            BLF_leg.calcServosDeg(tar_pose_matrix_BLF - BLF_leg.getEndEfectorPose())
            BLH_leg.calcServosDeg(tar_pose_matrix_BLH - BLH_leg.getEndEfectorPose())
            BRF_leg.calcServosDeg(tar_pose_matrix_BRF - BRF_leg.getEndEfectorPose())
            BRH_leg.calcServosDeg(tar_pose_matrix_BRH - BRH_leg.getEndEfectorPose())
            TLF_leg.calcServosDeg(tar_pose_matrix_TLF - TLF_leg.getEndEfectorPose())
            TLH_leg.calcServosDeg(tar_pose_matrix_TLH - TLH_leg.getEndEfectorPose())
            TRF_leg.calcServosDeg(tar_pose_matrix_TRF - TRF_leg.getEndEfectorPose())
            TRH_leg.calcServosDeg(tar_pose_matrix_TRH - TRH_leg.getEndEfectorPose())
            pub_pose()
            r.sleep()
        tar_pose_matrix_BLF = target_pose_BLF
        tar_pose_matrix_BLH = target_pose_BLH
        tar_pose_matrix_BRF = target_pose_BRF
        tar_pose_matrix_BRH = target_pose_BRH
        tar_pose_matrix_TLF = target_pose_TLF
        tar_pose_matrix_TLH = target_pose_TLH
        tar_pose_matrix_TRF = target_pose_TRF
        tar_pose_matrix_TRH = target_pose_TRH
        BLF_leg.calcServosDeg(tar_pose_matrix_BLF - BLF_leg.getEndEfectorPose())
        BLH_leg.calcServosDeg(tar_pose_matrix_BLH - BLH_leg.getEndEfectorPose())
        BRF_leg.calcServosDeg(tar_pose_matrix_BRF - BRF_leg.getEndEfectorPose())
        BRH_leg.calcServosDeg(tar_pose_matrix_BRH - BRH_leg.getEndEfectorPose())
        TLF_leg.calcServosDeg(tar_pose_matrix_TLF - TLF_leg.getEndEfectorPose())
        TLH_leg.calcServosDeg(tar_pose_matrix_TLH - TLH_leg.getEndEfectorPose())
        TRF_leg.calcServosDeg(tar_pose_matrix_TRF - TRF_leg.getEndEfectorPose())
        TRH_leg.calcServosDeg(tar_pose_matrix_TRH - TRH_leg.getEndEfectorPose())
        pub_pose()
        r.sleep()

        x_dist = 0.1
        start_pose_BLF = BLF_leg.getEndEfectorPose()
        start_pose_BLH = BLH_leg.getEndEfectorPose()
        start_pose_BRF = BRF_leg.getEndEfectorPose()
        start_pose_BRH = BRH_leg.getEndEfectorPose()
        start_pose_TLF = TLF_leg.getEndEfectorPose()
        start_pose_TLH = TLH_leg.getEndEfectorPose()
        start_pose_TRF = TRF_leg.getEndEfectorPose()
        start_pose_TRH = TRH_leg.getEndEfectorPose()
        target_pose_BLF = start_pose_BLF + Matrix([[-x_dist], [0], [0]])
        target_pose_BLH = start_pose_BLH + Matrix([[-x_dist], [0], [0]])
        target_pose_BRF = start_pose_BRF + Matrix([[-x_dist], [0], [0]])
        target_pose_BRH = start_pose_BRH + Matrix([[-x_dist], [0], [0]])
        target_pose_TLF = start_pose_TLF + Matrix([[-x_dist], [0], [0]])
        target_pose_TLH = start_pose_TLH + Matrix([[-x_dist], [0], [0]])
        target_pose_TRF = start_pose_TRF + Matrix([[-x_dist], [0], [0]])
        target_pose_TRH = start_pose_TRH + Matrix([[-x_dist], [0], [0]])
        max_step = 60
        for step in range(max_step):
            tar_pose_matrix_BLF = (target_pose_BLF - start_pose_BLF) * (float(step + 1) /  float(max_step)) + start_pose_BLF
            tar_pose_matrix_BLH = (target_pose_BLH - start_pose_BLH) * (float(step + 1) /  float(max_step)) + start_pose_BLH
            tar_pose_matrix_BRF = (target_pose_BRF - start_pose_BRF) * (float(step + 1) /  float(max_step)) + start_pose_BRF
            tar_pose_matrix_BRH = (target_pose_BRH - start_pose_BRH) * (float(step + 1) /  float(max_step)) + start_pose_BRH
            tar_pose_matrix_TLF = (target_pose_TLF - start_pose_TLF) * (float(step + 1) /  float(max_step)) + start_pose_TLF
            tar_pose_matrix_TLH = (target_pose_TLH - start_pose_TLH) * (float(step + 1) /  float(max_step)) + start_pose_TLH
            tar_pose_matrix_TRF = (target_pose_TRF - start_pose_TRF) * (float(step + 1) /  float(max_step)) + start_pose_TRF
            tar_pose_matrix_TRH = (target_pose_TRH - start_pose_TRH) * (float(step + 1) /  float(max_step)) + start_pose_TRH
            BLF_leg.calcServosDeg(tar_pose_matrix_BLF - BLF_leg.getEndEfectorPose())
            BLH_leg.calcServosDeg(tar_pose_matrix_BLH - BLH_leg.getEndEfectorPose())
            BRF_leg.calcServosDeg(tar_pose_matrix_BRF - BRF_leg.getEndEfectorPose())
            BRH_leg.calcServosDeg(tar_pose_matrix_BRH - BRH_leg.getEndEfectorPose())
            TLF_leg.calcServosDeg(tar_pose_matrix_TLF - TLF_leg.getEndEfectorPose())
            TLH_leg.calcServosDeg(tar_pose_matrix_TLH - TLH_leg.getEndEfectorPose())
            TRF_leg.calcServosDeg(tar_pose_matrix_TRF - TRF_leg.getEndEfectorPose())
            TRH_leg.calcServosDeg(tar_pose_matrix_TRH - TRH_leg.getEndEfectorPose())
            pub_pose()
            r.sleep()
        tar_pose_matrix_BLF = target_pose_BLF
        tar_pose_matrix_BLH = target_pose_BLH
        tar_pose_matrix_BRF = target_pose_BRF
        tar_pose_matrix_BRH = target_pose_BRH
        tar_pose_matrix_TLF = target_pose_TLF
        tar_pose_matrix_TLH = target_pose_TLH
        tar_pose_matrix_TRF = target_pose_TRF
        tar_pose_matrix_TRH = target_pose_TRH
        BLF_leg.calcServosDeg(tar_pose_matrix_BLF - BLF_leg.getEndEfectorPose())
        BLH_leg.calcServosDeg(tar_pose_matrix_BLH - BLH_leg.getEndEfectorPose())
        BRF_leg.calcServosDeg(tar_pose_matrix_BRF - BRF_leg.getEndEfectorPose())
        BRH_leg.calcServosDeg(tar_pose_matrix_BRH - BRH_leg.getEndEfectorPose())
        TLF_leg.calcServosDeg(tar_pose_matrix_TLF - TLF_leg.getEndEfectorPose())
        TLH_leg.calcServosDeg(tar_pose_matrix_TLH - TLH_leg.getEndEfectorPose())
        TRF_leg.calcServosDeg(tar_pose_matrix_TRF - TRF_leg.getEndEfectorPose())
        TRH_leg.calcServosDeg(tar_pose_matrix_TRH - TRH_leg.getEndEfectorPose())
        pub_pose()
        r.sleep()

        start_pose_BLF = BLF_leg.getEndEfectorPose()
        start_pose_BLH = BLH_leg.getEndEfectorPose()
        start_pose_BRF = BRF_leg.getEndEfectorPose()
        start_pose_BRH = BRH_leg.getEndEfectorPose()
        start_pose_TLF = TLF_leg.getEndEfectorPose()
        start_pose_TLH = TLH_leg.getEndEfectorPose()
        start_pose_TRF = TRF_leg.getEndEfectorPose()
        start_pose_TRH = TRH_leg.getEndEfectorPose()
        target_pose_BLF = start_pose_BLF + Matrix([[x_dist], [0], [0]])
        target_pose_BLH = start_pose_BLH + Matrix([[x_dist], [0], [0]])
        target_pose_BRF = start_pose_BRF + Matrix([[x_dist], [0], [0]])
        target_pose_BRH = start_pose_BRH + Matrix([[x_dist], [0], [0]])
        target_pose_TLF = start_pose_TLF + Matrix([[x_dist], [0], [0]])
        target_pose_TLH = start_pose_TLH + Matrix([[x_dist], [0], [0]])
        target_pose_TRF = start_pose_TRF + Matrix([[x_dist], [0], [0]])
        target_pose_TRH = start_pose_TRH + Matrix([[x_dist], [0], [0]])
        max_step = 60
        for step in range(max_step):
            tar_pose_matrix_BLF = (target_pose_BLF - start_pose_BLF) * (float(step + 1) /  float(max_step)) + start_pose_BLF
            tar_pose_matrix_BLH = (target_pose_BLH - start_pose_BLH) * (float(step + 1) /  float(max_step)) + start_pose_BLH
            tar_pose_matrix_BRF = (target_pose_BRF - start_pose_BRF) * (float(step + 1) /  float(max_step)) + start_pose_BRF
            tar_pose_matrix_BRH = (target_pose_BRH - start_pose_BRH) * (float(step + 1) /  float(max_step)) + start_pose_BRH
            tar_pose_matrix_TLF = (target_pose_TLF - start_pose_TLF) * (float(step + 1) /  float(max_step)) + start_pose_TLF
            tar_pose_matrix_TLH = (target_pose_TLH - start_pose_TLH) * (float(step + 1) /  float(max_step)) + start_pose_TLH
            tar_pose_matrix_TRF = (target_pose_TRF - start_pose_TRF) * (float(step + 1) /  float(max_step)) + start_pose_TRF
            tar_pose_matrix_TRH = (target_pose_TRH - start_pose_TRH) * (float(step + 1) /  float(max_step)) + start_pose_TRH
            BLF_leg.calcServosDeg(tar_pose_matrix_BLF - BLF_leg.getEndEfectorPose())
            BLH_leg.calcServosDeg(tar_pose_matrix_BLH - BLH_leg.getEndEfectorPose())
            BRF_leg.calcServosDeg(tar_pose_matrix_BRF - BRF_leg.getEndEfectorPose())
            BRH_leg.calcServosDeg(tar_pose_matrix_BRH - BRH_leg.getEndEfectorPose())
            TLF_leg.calcServosDeg(tar_pose_matrix_TLF - TLF_leg.getEndEfectorPose())
            TLH_leg.calcServosDeg(tar_pose_matrix_TLH - TLH_leg.getEndEfectorPose())
            TRF_leg.calcServosDeg(tar_pose_matrix_TRF - TRF_leg.getEndEfectorPose())
            TRH_leg.calcServosDeg(tar_pose_matrix_TRH - TRH_leg.getEndEfectorPose())
            pub_pose()
            r.sleep()
        tar_pose_matrix_BLF = target_pose_BLF
        tar_pose_matrix_BLH = target_pose_BLH
        tar_pose_matrix_BRF = target_pose_BRF
        tar_pose_matrix_BRH = target_pose_BRH
        tar_pose_matrix_TLF = target_pose_TLF
        tar_pose_matrix_TLH = target_pose_TLH
        tar_pose_matrix_TRF = target_pose_TRF
        tar_pose_matrix_TRH = target_pose_TRH
        BLF_leg.calcServosDeg(tar_pose_matrix_BLF - BLF_leg.getEndEfectorPose())
        BLH_leg.calcServosDeg(tar_pose_matrix_BLH - BLH_leg.getEndEfectorPose())
        BRF_leg.calcServosDeg(tar_pose_matrix_BRF - BRF_leg.getEndEfectorPose())
        BRH_leg.calcServosDeg(tar_pose_matrix_BRH - BRH_leg.getEndEfectorPose())
        TLF_leg.calcServosDeg(tar_pose_matrix_TLF - TLF_leg.getEndEfectorPose())
        TLH_leg.calcServosDeg(tar_pose_matrix_TLH - TLH_leg.getEndEfectorPose())
        TRF_leg.calcServosDeg(tar_pose_matrix_TRF - TRF_leg.getEndEfectorPose())
        TRH_leg.calcServosDeg(tar_pose_matrix_TRH - TRH_leg.getEndEfectorPose())
        pub_pose()
        r.sleep()

        start_pose_BLF = BLF_leg.getEndEfectorPose()
        start_pose_BLH = BLH_leg.getEndEfectorPose()
        start_pose_BRF = BRF_leg.getEndEfectorPose()
        start_pose_BRH = BRH_leg.getEndEfectorPose()
        start_pose_TLF = TLF_leg.getEndEfectorPose()
        start_pose_TLH = TLH_leg.getEndEfectorPose()
        start_pose_TRF = TRF_leg.getEndEfectorPose()
        start_pose_TRH = TRH_leg.getEndEfectorPose()
        target_pose_BLF = start_pose_BLF + Matrix([[0.0], [0], [up_dist]])
        target_pose_BLH = start_pose_BLH + Matrix([[0.0], [0], [up_dist]])
        target_pose_BRF = start_pose_BRF + Matrix([[0.0], [0], [up_dist]])
        target_pose_BRH = start_pose_BRH + Matrix([[0.0], [0], [up_dist]])
        target_pose_TLF = start_pose_TLF + Matrix([[0.0], [0], [-up_dist]])
        target_pose_TLH = start_pose_TLH + Matrix([[0.0], [0], [-up_dist]])
        target_pose_TRF = start_pose_TRF + Matrix([[0.0], [0], [-up_dist]])
        target_pose_TRH = start_pose_TRH + Matrix([[0.0], [0], [-up_dist]])
        max_step = 60
        for step in range(max_step):
            tar_pose_matrix_BLF = (target_pose_BLF - start_pose_BLF) * (float(step + 1) /  float(max_step)) + start_pose_BLF
            tar_pose_matrix_BLH = (target_pose_BLH - start_pose_BLH) * (float(step + 1) /  float(max_step)) + start_pose_BLH
            tar_pose_matrix_BRF = (target_pose_BRF - start_pose_BRF) * (float(step + 1) /  float(max_step)) + start_pose_BRF
            tar_pose_matrix_BRH = (target_pose_BRH - start_pose_BRH) * (float(step + 1) /  float(max_step)) + start_pose_BRH
            tar_pose_matrix_TLF = (target_pose_TLF - start_pose_TLF) * (float(step + 1) /  float(max_step)) + start_pose_TLF
            tar_pose_matrix_TLH = (target_pose_TLH - start_pose_TLH) * (float(step + 1) /  float(max_step)) + start_pose_TLH
            tar_pose_matrix_TRF = (target_pose_TRF - start_pose_TRF) * (float(step + 1) /  float(max_step)) + start_pose_TRF
            tar_pose_matrix_TRH = (target_pose_TRH - start_pose_TRH) * (float(step + 1) /  float(max_step)) + start_pose_TRH
            BLF_leg.calcServosDeg(tar_pose_matrix_BLF - BLF_leg.getEndEfectorPose())
            BLH_leg.calcServosDeg(tar_pose_matrix_BLH - BLH_leg.getEndEfectorPose())
            BRF_leg.calcServosDeg(tar_pose_matrix_BRF - BRF_leg.getEndEfectorPose())
            BRH_leg.calcServosDeg(tar_pose_matrix_BRH - BRH_leg.getEndEfectorPose())
            TLF_leg.calcServosDeg(tar_pose_matrix_TLF - TLF_leg.getEndEfectorPose())
            TLH_leg.calcServosDeg(tar_pose_matrix_TLH - TLH_leg.getEndEfectorPose())
            TRF_leg.calcServosDeg(tar_pose_matrix_TRF - TRF_leg.getEndEfectorPose())
            TRH_leg.calcServosDeg(tar_pose_matrix_TRH - TRH_leg.getEndEfectorPose())
            pub_pose()
            r.sleep()
        tar_pose_matrix_BLF = target_pose_BLF
        tar_pose_matrix_BLH = target_pose_BLH
        tar_pose_matrix_BRF = target_pose_BRF
        tar_pose_matrix_BRH = target_pose_BRH
        tar_pose_matrix_TLF = target_pose_TLF
        tar_pose_matrix_TLH = target_pose_TLH
        tar_pose_matrix_TRF = target_pose_TRF
        tar_pose_matrix_TRH = target_pose_TRH
        BLF_leg.calcServosDeg(tar_pose_matrix_BLF - BLF_leg.getEndEfectorPose())
        BLH_leg.calcServosDeg(tar_pose_matrix_BLH - BLH_leg.getEndEfectorPose())
        BRF_leg.calcServosDeg(tar_pose_matrix_BRF - BRF_leg.getEndEfectorPose())
        BRH_leg.calcServosDeg(tar_pose_matrix_BRH - BRH_leg.getEndEfectorPose())
        TLF_leg.calcServosDeg(tar_pose_matrix_TLF - TLF_leg.getEndEfectorPose())
        TLH_leg.calcServosDeg(tar_pose_matrix_TLH - TLH_leg.getEndEfectorPose())
        TRF_leg.calcServosDeg(tar_pose_matrix_TRF - TRF_leg.getEndEfectorPose())
        TRH_leg.calcServosDeg(tar_pose_matrix_TRH - TRH_leg.getEndEfectorPose())
        pub_pose()
        r.sleep()

    while not rospy.is_shutdown():

        pub_pose()
        rospy.sleep(0.1)

