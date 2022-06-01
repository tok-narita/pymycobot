import time
import os
import sys
from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle, Coord

sys.path.append("../demo/")
from port_setup import setup

reset = [153.19, 137.81, -153.54, 156.79, 87.27, 13.62]

if __name__=="__main__":
    mycobot = setup()
    while True:
#        mycobot.send_coords([x,y,z,0,90,0], 70, 0)
        print("\n::get_coords() ==> coords {}".format(mycobot.get_coords()))
        print("::get_angles() ==> coords {}".format(mycobot.get_angles()))
        
        break_flg = input('continue?(y/n): ')
        if break_flg=='n':
            break

    print("::set_free_mode()\n")
    mycobot.send_angles(reset, 70)
    time.sleep(5)
    mycobot.release_all_servos()
    print("Finished")
    
