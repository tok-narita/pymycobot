'''
move end-effector to DictCoords{}
'''

import time
import numpy as np
import os
import sys
from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle, Coord

sys.path.append("../demo/")
from port_setup import setup

reset = [153.19, 137.81, -153.54, 156.79, 87.27, 13.62]

DictCoords = {
    1: [43.4, 101.0, 296.6, -91.95, -27.48, -13.62],
    2: [266.8, 39.6, 201.2, 91.29, 1.15, 163.2],
    3: [10.1, 110.2, 188.7, 90.14, 5.62, 164.95],
    4: [90.8, 88.0, 290.1, 92.55, -42.6, 163.33],
    5: [282.6, 35.2, 151.4, -91.68, 8.59, -16.6],
    6: [20.9, 107.2, 213.6, 104.38, 81.83, 178.92],
    7: [100, 3500, 3500, 100, 100, 100],
    8: [22.016982157982746, -21.677374396364883, 27.282845872753676, -90, 0, -90]
}

if __name__=="__main__":
    mycobot = setup()
    # x, y, z = 160, 160, 160
    key = np.random.randint(1,8)

    while True:
        print("\n::send_coords() ==> coords {}".format(DictCoords[key]))      
        mycobot.sync_send_coords(coords=DictCoords[key], speed=70, mode=0, timeout=5)
        time.sleep(5)
        print("::get_coords() ==> coords {}".format(mycobot.get_coords()))
        print('is_in_position? ==> {}'.format(mycobot.is_in_position(DictCoords[key],1)))

        break_flg = input('continue?(1~7/n): ')
        if break_flg=='n':
            break
        key = int(break_flg)

    print("::set_free_mode()\n")
    mycobot.send_angles(reset, 70)
    time.sleep(5)
    mycobot.release_all_servos()
    print("Finished")
    
