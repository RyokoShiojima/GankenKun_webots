# Copyright 1996-2021 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from field import Field
from controller import Supervisor
import numpy as np
import os
import shutil
import pandas as pd
import math
import csv

"""
# start the webots supervisor
supervisor = Supervisor()
time_step = int(supervisor.getBasicTimeStep())

field = Field("kid")
children = supervisor.getRoot().getField('children')
children.importMFNodeFromString(-1, f'RobocupSoccerField {{ size "kid" }}')
children.importMFNodeFromString(-1, f'DEF PLAYER RoboCup_GankenKun {{translation -0.3 0 0.450 rotation 0 0 1 0 controller "capture_image" controllerArgs "x-0.30_y-0.00_the_0.00.jpg"}}')
player = supervisor.getFromDef('PLAYER')
player_translation = supervisor.getFromDef('PLAYER').getField('translation')
player_rotation = supervisor.getFromDef('PLAYER').getField('rotation')
player_controller = supervisor.getFromDef('PLAYER').getField('controller')
"""

class Robot(object):
    def __init__(self, x, y, th):
        self.init_pos = [x,y,th]
        self.pos = self.init_pos
        self.velocity = [0.0, 0.0] #[m/s, rad]

    def calc_odom(self, dt = 1):
        th = self.pos[2] + self.velocity[1] * dt
        x  = self.pos[0] + math.cos(th) * dt
        y  = self.pos[1] + math.sin(th) * dt
        self.pos = [x, y, th]
        self.nomalaize()

    def nomalaize(self):
        while (abs(self.pos[2]) > math.pi):
            self.pos[2] += math.pi*2 if self.pos[2] < 0  else -math.pi*2

def write_csv(odom_list, logcsv):
    label = ["x", "y", "theta","image"]
    df = pd.DataFrame(data=odom_list,columns=label)
    df.to_csv(logcsv,  mode='a', header=False, index=False)

def set_file(filename):
    if os.path.exists(filename) == True:
        shutil.rmtree(filename)
        os.makedirs(filename)
    else:
        os.makedirs(filename)

def set_csv(csvname):
    if os.path.exists(csvname) == True:
        os.remove(csvname)
    else:
        pass

def main():
    supervisor = Supervisor()
    time_step = int(supervisor.getBasicTimeStep())
    field = Field("kid")
    children = supervisor.getRoot().getField('children')
    children.importMFNodeFromString(-1, f'RobocupSoccerField {{ size "kid" }}')
    children.importMFNodeFromString(-1, f'DEF PLAYER RoboCup_GankenKun {{translation -0.3 0 0.450 rotation 0 0 1 0 controller "capture_image" controllerArgs "x-0.30_y-0.00_th_0.00.jpg"}}')
    player = supervisor.getFromDef('PLAYER')
    player_translation = supervisor.getFromDef('PLAYER').getField('translation')
    player_rotation = supervisor.getFromDef('PLAYER').getField('rotation')

    odom_plot= []
    logcsv = "odom.csv"
    #image_file = "image/"
    set_csv(logcsv)
    #set_file(image_file)

    for x in np.arange(-4.3, 4.3, 0.2):
        for y in np.arange(-3.0, 3.0, 0.2):
            for th in np.arange(-math.pi, math.pi, math.pi/18):
                mini_odom = []
                x_y_th_image = []
                robot = Robot(x,y,th)
                robot.velocity = [1, -math.pi/10]
                #odom_plot= []
    
                for t in range(10):
                    robot.calc_odom(0.5)
                    #player.remove()
                    #count = 0

                    if robot.pos[0] < -4.3 or robot.pos[0] > 4.3 or\
                            robot.pos[1] < -3 or robot.pos[1] > 3:
                        break
                    else:
                        #player.remove()
                        #image_name = "x"+format(robot.pos[0],"+.2f")+"_y"+format(robot.pos[1],"+.2f")+"_th"+format(robot.pos[2],"+.3f")+".jpg"
                        #children.importMFNodeFromString(-1, f'DEF PLAYER RoboCup_GankenKun {{translation {robot.pos[0]} {robot.pos[1]} 0.450 rotation 0 0 1 {robot.pos[2]} controller "capture_image" controllerArgs "{image_name}"}}')
                        #player = supervisor.getFromDef('PLAYER')
                        #while supervisor.step(time_step) != -1:
                        #    count += 1
                        #    if count > 10:
                        #        break
                        mini_odom.append(robot.pos)

                if len(mini_odom) == 10:
                    #write_csv(mini_odom,logcsv)
                    for x_y_th in mini_odom:
                        count= 0
                        player.remove()
                        image_name = "x"+format(x_y_th[0],"+.2f")+"_y"+format(x_y_th[1],"+.2f")+"_th"+format(x_y_th[2],"+.3f")+".jpg"
                        children.importMFNodeFromString(-1, f'DEF PLAYER RoboCup_GankenKun {{translation {x_y_th[0]} {x_y_th[1]} 0.450 rotation 0 0 1 {x_y_th[2]} controller "capture_image" controllerArgs "{image_name}"}}')
                        player = supervisor.getFromDef('PLAYER')
                        while supervisor.step(time_step) != -1:
                            count += 1
                            if count > 10:
                                break
                        new_image_name = "images/" + image_name
                        new_image_name = list(new_image_name)
                        name = ''.join(new_image_name) 
                        print(name)
                        x_y_th.extend(str(name))
                        print(x_y_th)
                        x_y_th_image.append(x_y_th)
                    write_csv(x_y_th_image,logcsv)

                    #odom_plot.append(mini_odom)
                else:
                    pass

if __name__ == "__main__":
    main()
"""
try:
    for x in np.arange(-4.3, 4.3, 0.2):
        for y in np.arange(-3, 3, 0.2):
            for the in np.arange(-1.57, 1.57, 3.14/10):
                count = 0
                player.remove()
                filename = "x"+format(x,"+.2f")+"_y"+format(y,"+.2f")+"_the"+format(the,"+.3f")+".jpg"
                children.importMFNodeFromString(-1, f'DEF PLAYER RoboCup_GankenKun {{translation {x} {y} 0.450 rotation 0 0 1 {the} controller "capture_image" controllerArgs "{filename}"}}')
                player = supervisor.getFromDef('PLAYER')
                while supervisor.step(time_step) != -1:
                    count += 1
                    if count > 10:
                        break
except Exception:
    error(f"Unexpected exception in main referee loop: {traceback.format_exc()}", fatal=True)
"""
