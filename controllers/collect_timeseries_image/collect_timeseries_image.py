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


def set_csv(csvname):
    if os.path.exists(csvname) == True:
        os.remove(csvname)
    else:
        pass

def check_file(file_path):
    if os.path.exists(file_path):
        shutil.rmtree(file_path)
    else:
        os.makedirs(file_path)

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
    set_csv(logcsv)
    cwd = os.getcwd()
    #print(cwd)
    image_file_path = "../capture_image/images/"
    #check_file(image_file_path)

    range_x  = np.arange(-4.3, 4.3, 1.0)
    range_y  = np.arange(-3.0, 3.0, 1.0)
    range_th = np.arange(0, math.pi/2, math.pi/2)

    cnt = 0
    total = len(range_x) * len(range_y) * len(range_th)
    for x in range_x:
        for y in range_y:
            for th in range_th:
                mini_odom = []
                x_y_th_image = []
                robot = Robot(x,y,th)
                robot.velocity = [1, -math.pi/10]
                #odom_plot= []
    
                print(f'{cnt} / {total}')
                cnt += 1
                for t in range(5):
                    robot.calc_odom(0.5)

                    if robot.pos[0] < -4.3 or robot.pos[0] > 4.3 or\
                            robot.pos[1] < -3 or robot.pos[1] > 3:
                        print(f'out of field {robot.pos}')
                        break
                    else:
                        mini_odom.append(robot.pos)

                if len(mini_odom) == 5:
                    #write_csv(mini_odom,logcsv)
                    for x_y_th in mini_odom:
                        count= 0
                        player.remove()
                        image_name = "x"+format(x_y_th[0],"+.2f")+"_y"+format(x_y_th[1],"+.2f")+"_th"+format(x_y_th[2],"+.3f")+".jpg"
                        children.importMFNodeFromString(-1, f'DEF PLAYER RoboCup_GankenKun {{translation {x_y_th[0]} {x_y_th[1]} 0.450 rotation 0 0 1 {x_y_th[2]} controller "capture_image" controllerArgs "{image_name}"}}')
                        player = supervisor.getFromDef('PLAYER')
                        while supervisor.step(time_step) != -1:
                            count += 1
                            if count > 5:
                                break
                        new_image_name = "images/" + image_name
                        x_y_th.append(new_image_name)
                        x_y_th_image.append(x_y_th)
                    write_csv(x_y_th_image,logcsv)

                    #odom_plot.append(mini_odom)
                else:
                    pass

import cProfile
if __name__ == "__main__":
    #cProfile.run('main()', filename='/tmp/main.prof')
    main()
