#!/usr/bin/env python3

import os
import sys
from controller import Supervisor
import numpy as np
import csv
import shutil
import pandas as pd

deviceImagePath = os.getcwd()
supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())
camera = supervisor.getDevice('camera_sensor')
camera.enable(timestep)

data_list = []
data_csv = "model_data.csv"
image_path = deviceImagePath + '/images/'

def write_csv(odom_list, logcsv):
    label = ["x", "y", "theta","image"]
    df = pd.DataFrame(data=odom_list,columns=label)
    df.to_csv(logcsv,  mode='a', header=False, index=False)

def check_file(file_path):
    if os.path.exists(file_path):
        shutil.rmtree(file_path)
        os.makedirs(file_path)
        print(f'file make success!')
    else:
        os.makedirs(file_path)

def set_csv(csvname):
    if os.path.exists(csvname) == True:
        os.remove(csvname)
    else:
        pass

try:
    set_csv(data_csv)
    check_file(image_path)
    for x in np.arange(-4.3, 4.3, 0.2):
        for y in np.arange(-3, 3, 0.2):
            for th in np.arange(0, 6.28, 3.14/10):
                supervisor.getFromDef('PLAYER').getField('translation').setSFVec3f([x, y, 0.450])
                supervisor.getFromDef('PLAYER').getField('rotation').setSFRotation([0, 0, 1, th])
                supervisor.step(timestep)
                filename = "x"+format(x,"+.2f")+"_y"+format(y,"+.2f")+"_th"+format(th,"+.3f")+".jpg"
                camera.saveImage(image_path + str(filename), 80)
                new_image_name = "images/" + filename
                data_list.append([x,y,th,new_image_name])
        supervisor.simulationReset()
        supervisor.step(timestep)

    
    write_csv(data_list, data_csv)
    print("finish")

except Exception:
    error(f"Unexpected exception in main referee loop: {traceback.format_exc()}", fatal=True)
