import time
import numpy as np
import os
import sys
from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle, Coord

sys.path.append("../demo/")
from port_setup import setup

reset = [153.19, 137.81, -153.54, 156.79, 87.27, 13.62]

if __name__=="__main__":
    mycobot = setup()
    # x, y, z = 160, 160, 160
    while True:
        theta0 = np.random.uniform(-100, 100) 
        theta = np.random.uniform(-150.0, 150.0, 4)
        mycobot.send_angles([165.0, theta0, theta[1], theta[2], -90.0, theta[3]], 70)
        print("\n::get_coords() ==> coords {}".format(mycobot.get_coords()))
        print("::send_angles() ==> coords [{:.1f} {:.1f} {:.1f} {:.1f} {:.1f} {:.1f}]".format(165.0, theta0, theta[1], theta[2], -90.0, theta[3]))
        time.sleep(3)
        print("::get_angles() ==> coords {}".format(mycobot.get_angles()))

        break_flg = input('continue?(y/n): ')
        if break_flg=='n':
            break

    print("::set_free_mode()\n")
    mycobot.send_angles(reset, 70)
    time.sleep(5)
    mycobot.release_all_servos()
    print("Finished")
    
