import time
import numpy as np
import os
import sys
from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle, Coord

sys.path.append("../demo/")
from port_setup import setup

reset = [153.19, 137.81, -153.54, 156.79, 87.27, 13.62]


def scara_to_mycobot_coords(x, y, z):
    distance_scara_mycobot = 70-6.5
    return distance_scara_mycobot-x, -y, z

if __name__=="__main__":
    mycobot = setup()
    angles = [0, 0, 0, 0, 0, 0]

    while True:
        mycobot.send_angles(angles, 100)
        time.sleep(3)
        print("::get_coords() ==> coords {}".format(mycobot.get_coords()))
        
        x, y, z = map(int, input("x, y, z: ").split())
        rx, ry, rz = -90, 0,-90
        coords = [x, y, z, rx, ry, rz]
        
        print("\n::send_coords() ==> coords {}".format(coords))
        try:
            mycobot.send_coords(coords=coords, speed=70, mode=0)
            time.sleep(3)
        except:
            print("MyCobot can't reach")
        print("::get_coords() ==> coords {}".format(mycobot.get_coords()))
        print('is_in_position? ==> {}'.format(mycobot.is_in_position(coords, 1)))
        break_flg = input('continue?(y/n): ')
        if break_flg=='n':
             break

    print("::set_free_mode()\n")
    mycobot.send_angles(reset, 70)
    time.sleep(3)
    mycobot.release_all_servos()
    print("Finished")
