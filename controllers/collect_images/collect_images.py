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

def write_csv(odom_list, logcsv):
    label = ["x", "y", "theta","image"]
    df = pd.DataFrame(data=odom_list,columns=label)
    df.to_csv(logcsv,  mode='a', header=False, index=False)

def set_csv(csvname):
    if os.path.exists(csvname) == True:
        os.remove(csvname)
    else:
        pass

try:
    set_csv(data_csv)
    for x in np.arange(-4.3, 4.3, 0.2):
        supervisor.simulationReset()
        for y in np.arange(-3, 3, 0.2):
            for th in np.arange(-1.57, 1.57, 3.14/10):
                supervisor.getFromDef('PLAYER').getField('translation').setSFVec3f([x, y, 0.450])
                supervisor.getFromDef('PLAYER').getField('rotation').setSFRotation([0, 0, 1, th])
                for i in range(1):
                    supervisor.step(timestep)
                filename = "x"+format(x,"+.2f")+"_y"+format(y,"+.2f")+"_th"+format(th,"+.3f")+".jpg"
                camera.saveImage(deviceImagePath + '/images/' + str(filename), 80)
                new_image_name = "images/" + filename
                data_list.append([x,y,th,new_image_name])
    
    write_csv(data_list, data_csv)

except Exception:
    error(f"Unexpected exception in main referee loop: {traceback.format_exc()}", fatal=True)
