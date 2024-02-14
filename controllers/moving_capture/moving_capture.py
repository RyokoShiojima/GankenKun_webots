#!/usr/bin/env python3

import os
import csv
import math
import sys
import shutil
from geometry import coord_trans_global_to_local
from controller import Supervisor
sys.path.append('./GankenKun')
from kinematics import *
from foot_step_planner import *
from preview_control import *
from walking import *
#from random import random
import random
from scipy.spatial.transform import Rotation as R

csvfile = "log.csv"
if os.path.exists(csvfile):
  os.remove(csvfile)

imagefile = "images/"
if os.path.exists(imagefile):
  shutil.rmtree(imagefile)
  os.makedirs(imagefile)
else:
  os.makedirs(imagefile)

motorNames = [
  "head_yaw_joint",                        # ID1
  "left_shoulder_pitch_joint [shoulder]",  # ID2
  "left_shoulder_roll_joint",              # ID3
  "left_elbow_pitch_joint",                # ID4
  "right_shoulder_pitch_joint [shoulder]", # ID5
  "right_shoulder_roll_joint",             # ID6
  "right_elbow_pitch_joint",               # ID7
  "left_waist_yaw_joint",                  # ID8
  "left_waist_roll_joint [hip]",           # ID9
  "left_waist_pitch_joint",                # ID10
  "left_knee_pitch_joint",                 # ID11
  "left_ankle_pitch_joint",                # ID12
  "left_ankle_roll_joint",                 # ID13
  "right_waist_yaw_joint",                 # ID14
  "right_waist_roll_joint [hip]",          # ID15
  "right_waist_pitch_joint",               # ID16
  "right_knee_pitch_joint",                # ID17
  "right_ankle_pitch_joint",               # ID18
  "right_ankle_roll_joint"                 # ID19
]

if __name__ == '__main__':
  #シミュレーションする時間
  sim_time = 10 * 60 * 1000  #10分

  #取得画像の保存先設定
  deviceImagePath = os.getcwd()
  image_path = deviceImagePath + '/images/'
  #シミュレータ定義
  robot = Supervisor()
  timestep = int(robot.getBasicTimeStep())
  camera = robot.getDevice('camera_sensor')
  if camera is None:
      print("Error: could not find camera device.")
  else:
    camera.enable(timestep)
  
  #歩行制御関係
  motor = [None]*len(motorNames)
  for i in range(len(motorNames)):
    motor[i] = robot.getDevice(motorNames[i])

  joint_angles = [0]*len(motorNames)

  left_foot  = [-0.02, +0.054, 0.02]
  right_foot = [-0.02, -0.054, 0.02]

  pc = preview_control(timestep/1000, 1.0, 0.27)
  walk = walking(timestep/1000, motorNames, left_foot, right_foot, joint_angles, pc)
  
  #最初の目標値の定義
  foot_step = walk.setGoalPos([0.2, 0.0, 0.0])

  #画像取得関係の数値の定義(画像番号, 何秒に一回撮るか)
  cap_num = 1
  image_time = 0

  #目標座標定義
  target_global = [0,0,0]

  while robot.step(timestep) != -1:
    #今のロボットの位置と姿勢
    node = robot.getSelf()
    x, y, z = node.getPosition()
    r = node.getOrientation()
    rot = R.from_matrix([[r[0], r[1], r[2]], [r[3], r[4], r[5]], [r[6], r[7], r[8]]])
    th, _, _ = rot.as_euler('zyx', degrees=False) #[]に3つ格納されてるうちの一番最初がth 

    #画像取得
    if image_time >= 3000: #10秒の場合は10000
      filename = f'{image_path}{cap_num:08}.jpg'
      camera.saveImage(filename, 92)
      print(f'画像取得枚数:{cap_num}')
      image_time = 0
        #print (f'{x} {y} {math.degrees(th)} {os.path.basename(filename)}')この文章をメモに書き足していく
      with open(csvfile, "a", newline="") as f:
        writer = csv.writer(f)
        if cap_num == 1:
          writer.writerow(['x', 'y', 'th_radians', 'th_deg', 'image'])
        writer.writerow([str(x), str(y), str(th), str(math.degrees(th)), os.path.basename(filename)])
      cap_num += 1
      print (f'撮った画像の座標:[{x} {y} {math.degrees(th)} ラジアン表記：{th}]')

    else:
      image_time += timestep
    
    
    #現在地から目標座標が0.5未満のときは目標座標を決め直す
    if math.dist((target_global[0], target_global[1]), (x, y)) <= 0.5:
      #target_global = [random.uniform(-4.5, 4.5), random.uniform(-3.0, 3.0), math.radians(random.uniform(-180, 180))]
      target_global = [random.uniform(-3.8, 3.8), random.uniform(-2.5, 2.5), math.radians(random.uniform(-180, 180))]
      print(f"目標座標更新：{target_global}")

    #ロボットと目標座標との相対
    target_local = coord_trans_global_to_local([x, y, th], target_global)

    #現在の方位から目標座標までの方位で一定の閾値を越してたら向きを合わせてから歩行
    direction = math.atan2(target_local[1], target_local[0])
    if direction <= math.radians(-30):
      target_local = [0.0, 0, -0.3]
    elif direction >= math.radians(30):
      target_local = [0.0, 0, 0.3]
    else:
      target_local = [0.3, 0, direction/10]

    joint_angles, lf, rf, xp, n = walk.getNextPos()
    if n == 0:
      if len(foot_step) <= 6:
        #print(f"目標Global：{target_global} 歩行目標：{target_local}")
        target_local[0] += foot_step[0][1]
        target_local[1] += foot_step[0][2]
        target_local[2] += foot_step[0][3]
        foot_step = walk.setGoalPos(target_local)
      else:
        foot_step = walk.setGoalPos()
        
    #if you want new goal, please send position
    for i in range(len(motorNames)):
      motor[i].setPosition(joint_angles[i])

    
    #一定枚数に達したら終了
    if cap_num == 5001:#コードの流れの関係上、取りたい枚数+1
        break



