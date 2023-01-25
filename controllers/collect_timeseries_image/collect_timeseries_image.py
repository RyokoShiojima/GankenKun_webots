from field import Field
from controller import Supervisor
import numpy as np
import os
import shutil
import pandas as pd
import math
import random
import csv
import time


class Robot(object):
    def __init__(self, x, y, th, v, rad):
        self.init_pos = [x,y,th]
        self.pos = self.init_pos
        self.velocity = [v, rad] #[m/s, rad]

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
        os.makedirs(file_path)
        print(f'file make success!')
    else:
        os.makedirs(file_path)

def main():
    supervisor = Supervisor()
    timestep = int(supervisor.getBasicTimeStep())
    camera = supervisor.getDevice('camera_sensor')
    camera.enable(timestep)

    deviceImagePath = os.getcwd()
    odom_plot= []
    logcsv = "odom.csv"
    set_csv(logcsv)
    image_file_path = deviceImagePath + "/images/"
    check_file(image_file_path)

    range_x  = np.arange(-4.3, 4.3, 0.2)
    range_y  = np.arange(-3.0, 3.0, 0.2)
    range_th = np.arange(0, 6.28, math.pi/18)

    cnt = 0
    total = len(range_x) * len(range_y) * len(range_th)
    for x in range_x:
        for y in range_y:
            for th in range_th:
                count = 0
                mini_odom = []
                x_y_th_image = []
                random_rad = random.uniform(0, math.pi)
                robot = Robot(x,y,th,1,random_rad)
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
                        count += 1
                        #print(count)
                        supervisor.getFromDef('PLAYER').getField('translation').setSFVec3f([x_y_th[0], x_y_th[1], 0.450])
                        supervisor.getFromDef('PLAYER').getField('rotation').setSFRotation([0, 0, 1, x_y_th[2]])
                        #for i in range(1):
                        supervisor.step(timestep)
                        image_name = "x"+format(x_y_th[0],"+.2f")+"_y"+format(x_y_th[1],"+.2f")+"_th"+format(x_y_th[2],"+.3f")+".jpg"
                        camera.saveImage(image_file_path + str(image_name), 80)
                        #time.sleep(0.5)
                        new_image_name = "images/" + image_name
                        x_y_th.append(new_image_name)
                        x_y_th_image.append(x_y_th)
                        if count == 5:
                            supervisor.simulationReset()
                            supervisor.step(timestep)
                        else:
                            pass
                    write_csv(x_y_th_image,logcsv)

                    #odom_plot.append(mini_odom)
                else:
                    pass


import cProfile
if __name__ == "__main__":
    #cProfile.run('main()', filename='/tmp/main.prof')
    main()
