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
    rnd = np.random.RandomState(0)
    while True:
        """
        except theta_3, plus direction follows the law of right-hand thread
        """
        yaw_sign = 1 if rnd.random()>=0.5 else -1
        theta_0 = rnd.uniform(0, 30) * yaw_sign
        theta_1 = rnd.uniform(-40, 30)
        theta_2 = rnd.uniform(-40, 30)
        theta_end = rnd.uniform(-30, 30)
        theta_3 = theta_end - (theta_1+theta_2)
        theta_4 = rnd.uniform(0, 30) * yaw_sign
        theta_5 = 0 
        mycobot.send_angles([theta_0, theta_1, theta_2, theta_3, theta_4, theta_5], 70)
       
        print("\n::send_angles() ==> angles [{:.1f} {:.1f} {:.1f} {:.1f} {:.1f} {:.1f}]".format(theta_0, theta_1, theta_2, theta_3, theta_4, theta_5))
        # time.sleep(3)
        print("::get_angles()  ==> angles {}".format(mycobot.get_angles()))
        print("get_coords() ==> coords {}".format(mycobot.get_coords()))

        break_flg = input('continue?(y/n): ')
        if break_flg=='n':
            break

    print("::set_free_mode()\n")
    mycobot.send_angles(reset, 70)
    time.sleep(5)
    mycobot.release_all_servos()
    print("Finished")
    
