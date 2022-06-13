"""
distance unit [mm]
"""
import time
import numpy as np
import sys
from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle, Coord
import serial
import serial.tools.list_ports

# sys.path.append("~/scara-robot")
sys.path.append("../../scara-robot")
# import pprint
# pprint.pprint(sys.path)
import harvesting_movement
import tokuiten_scara

class TokMyCobot(MyCobot):
    def my_send_coords(coords:list, speed:int, mode=0):
        for i in range(3):
            coords[i] /= 1000
        self.send_coords(coords=coords, speed=speed, mode=mode)

    def my_get_coords():
        coords = self.get_coords()
        for i in range(3):
            coords[i] /= 1000
        return coords

    def my_is_in_position(coords:list, mode=0):
        pass

def setup():
    """
    Reference code: ../demo/port_setup.py
    Modified MyCobot -> TokMyCobot
    """
    print("")

    plist = list(serial.tools.list_ports.comports())
    idx = 1
    for port in plist:
        print("{} : {}".format(idx, port))
        idx += 1

    _in = input("\nPlease input 1 - {} to choice:".format(idx - 1))
    port = str(plist[int(_in) - 1]).split(" - ")[0].strip()
    print(port)
    print("")

    baud = 115200
    _baud = input("Please input baud(default:115200):")
    try:
        baud = int(_baud)
    except Exception:
        pass
    print(baud)
    print("")

    DEBUG = False
    f = input("Wether DEBUG mode[Y/n]:")
    if f in ["y", "Y", "yes", "Yes"]:
        DEBUG = True
    mc = TokMyCobot(port, baud, debug=DEBUG)
    return 

def arm_reachable_check(x:float, y:float, z:float, tdof_scara) -> bool:
    """
    HACKME: In order to check z-value, if-statement is used.
    """
    if z<10 or z>45:
        return False
    try:
        _ = tdof_scara.inverse_kinematics(x, y, 0.0)
        return False
    except:
        return False

def scara_to_mycobot_coords(x, y, z):
    distance_scara_mycobot = 70-6.5
    return distance_scara_mycobot-x, -y, z

if __name__=="__main__":
    mycobot = setup()
    
    tdof_scara = tokuiten_scara.ThreeDoF_SCARA()

    tsr = harvesting_movement.TokuitenScaraRobot(30)#present_height)
    present_height = tsr.get_z_height()


    rnd = np.random.RandomState(42)
    
    while True:
        while True:
            x_target= rnd.uniform(50, 400)
            y_target = rnd.uniform(-400, 400)
            z_target = rnd.uniform(10, 45)
            print(f"x:{x_target}, y:{y_target}, z:{z_target}")
            if arm_reachable_check:
                rx, ry, rz = -90, 0, -90
                coords = [x_target, y_target, z_target, rx, ry, rz]
                break
            
        # break
    print("Finished!")
