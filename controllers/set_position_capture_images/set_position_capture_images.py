#!/usr/bin/env python3

import os
import sys
import csv
import time
import math
import shutil
import numpy as np
import pandas as pd
from controller import Supervisor
from scipy.spatial.transform import Rotation as R

csvfile = "capture.csv"
if os.path.exists(csvfile):
    os.remove(csvfile) 

imagefile = "images/"
if os.path.exists(imagefile):
    shutil.rmtree(imagefile)
    os.makedirs(imagefile)
else:
    os.makedirs(imagefile)

#撮影してほしい位置姿勢の読み込み
data = pd.read_csv("log.csv")
x = list(data.iloc[:,0])
y = list(data.iloc[:,1])
th = list(data.iloc[:,2])

if __name__ == '__main__':
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

    #画像取得関係の数値の定義(画像番号)
    cap_num = 1

    cnt = 0

while robot.step(timestep) != -1:
    print(f'撮影してほしい場所：{x[cnt]}, {y[cnt]}, {th[cnt]}')
    robot.getFromDef('PLAYER').getField('translation').setSFVec3f([x[cnt], y[cnt], 0.450])
    robot.getFromDef('PLAYER').getField('rotation').setSFRotation([0, 0, 1, th[cnt]])
    robot.step(timestep)
    filename = f'{image_path}{cap_num:08}.jpg'
    camera.saveImage(filename, 92)                                                                                                                                                                           
    print(f'画像取得枚数:{cap_num}')
    with open(csvfile, "a", newline="") as f:
        writer = csv.writer(f)
        if cap_num == 1:
            writer.writerow(['x', 'y', 'th_radians', 'th_deg', 'image'])
        writer.writerow([str(x[cnt]), str(y[cnt]), str(th[cnt]), str(math.degrees(th[cnt])), os.path.basename(filename)])
        cap_num += 1
    cnt += 1
    if cap_num % 100 == 0:
        robot.simulationReset()
    #robot.step(timestep) 
