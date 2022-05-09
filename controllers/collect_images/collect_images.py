#!/usr/bin/env python3

import os
import sys
from controller import Supervisor
import numpy as np
import time

deviceImagePath = os.getcwd()
supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())
camera = supervisor.getDevice('camera_sensor')
camera.enable(timestep)
i = 0

try:
    for x in np.arange(-4.3, 4.3, 0.2):
        for y in np.arange(-3, 3, 0.2):
            for the in np.arange(-1.57, 1.57, 3.14/10):
                supervisor.getFromDef('PLAYER').getField('translation').setSFVec3f([x, y, 0.450])
                supervisor.getFromDef('PLAYER').getField('rotation').setSFRotation([0, 0, 1, the])
                supervisor.step(timestep)
                filename = "x"+format(x,"+.2f")+"_y"+format(y,"+.2f")+"_the"+format(the,"+.3f")+".jpg"
                camera.saveImage(deviceImagePath + '/images/' + str(filename), 80) 
                i += 1
        supervisor.simulationReset()
        supervisor.step(timestep)

except Exception:
    error(f"Unexpected exception in main referee loop: {traceback.format_exc()}", fatal=True)
