"""
distance unit [m]
"""
import time
import numpy as np
import sys
import math
from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle, Coord
import serial
import serial.tools.list_ports
import cv2
import pyrealsense2.pyrealsense2 as rs

# sys.path.append("~/scara-robot")
sys.path.append("../../scara-robot")
# import pprint
# pprint.pprint(sys.path)
import harvesting_movement
import tokuiten_scara
from click_and_harvest import get_click_point
# from my_utils import realsense_to_arm_position

class TokMyCobot(MyCobot):
    def my_send_coords(self, coords:list, speed:int, mode=0):
        for i in range(3):
            coords[i] *= 1000
        print(coords)
        self.send_coords(coords=coords, speed=speed, mode=mode)
        time.sleep(1)

    def my_get_coords(self):
        coords = self.get_coords()
        if coords==[]:
            return coords
        else:
            print(coords)
            for i in range(3):
                coords[i] /= 1000
            return coords

    def my_is_in_position(self, coords:list, mode=0):
        pass

    def cobot_reachable_check(self, coords:list) -> bool:
        """
        HACKME: There should be more clever way...!
        """
        position_before = self.get_coords()
        self.my_send_coords(coords=coords, speed=70, mode=0)
        position_after = self.get_coords()
        if position_before != position_after:
            return True
        else:
            return False

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
    # _baud = input("Please input baud(default:115200):")
    # try:
    #     baud = int(_baud)
    # except Exception:
    #     pass
    # print(baud)
    # print("")

    DEBUG = False
    f = input("Wether DEBUG mode[Y/n]:")
    if f in ["y", "Y", "yes", "Yes"]:
        DEBUG = True
    mc = TokMyCobot(port, baud, debug=DEBUG)
    return mc

def convert(realsense_point_3D):
    """
    Basically, this function is same as my_utils/realsense_to_arm_position.py/
    However, I could not import because of relative path!!
    """
    transformation = np.loadtxt("../../scara-robot/configulation/transformation.matrix")
    realsense_point_3D.append(1.0)
    np_realsense_point_3D = np.array(realsense_point_3D)
    position_arm_coordination = np.matmul(transformation, np_realsense_point_3D)
    arm_x = position_arm_coordination[0]
    arm_y = position_arm_coordination[1]
    arm_z = position_arm_coordination[2]
    return [arm_x, arm_y, arm_z]


class ClickPointManager():
    def __init__(self, tsr):
        self.flag_click = False
        self.point_3D = None
        self.coords_target = None
        self.depth_frame = None

    def flag_click_change_False(self):
        self.flag_click = False

    def flag_click_change_True(self):
        self.flag_click = True
    
    def update_depth_frame(self, depth_frame):
        self.depth_frame = depth_frame
    
    def get_click_coords(self, event, x, y, flags, param):
        """
        1. Get coordinations of RGBD camera
        2. Change RGBD frame into Scara-Robot frame
        3. flag_click: False -> True  
        """
        # print(f"event: {event}, self.flag_click: {self.flag_click}, x: {x}")
        if event != cv2.EVENT_LBUTTONDOWN or self.flag_click==True or x>=640:
            return
        
        depth = self.depth_frame.get_distance(x, y)
        if depth == 0:
            print("ERROR: cannot get depth data ! ")
            return

        self.point_3D = rs.rs2_deproject_pixel_to_point(color_intrinsics, [x, y], depth)
        x_target, y_target, z_target = convert(self.point_3D)
        self.coords_target = [x_target, y_target, z_target]
        #y_target = y_target + 10
        print("point(rs): ", self.point_3D)
        print("point(arm): ", self.coords_target)

        self.flag_click_change_True()

def arm_reachable_check(x:float, y:float, z:float, tdof_scara) -> bool:
    """
    HACKME: For checking z-value, if-statement is used.
    """
    if z<5/100 or 45/100<z:
        return False
    try:
        _ = tdof_scara.inverse_kinematics(x, y, 0.0)
        return True
    except:
        return False


def scara_to_mycobot_coords(x, y, z):
    distance_scara_mycobot = 70/100-6.5/100
    return distance_scara_mycobot-x, -y, z

if __name__=="__main__":
    mycobot = setup()

    rnd = np.random.RandomState(42)
    num_trial = 0
    num_success = 0

    # For Camera
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    print("Start streaming")
    pipeline.start(config)

    align_to = rs.stream.color
    align = rs.align(align_to)

    try:
        # FIXME: Path is not relative
        file_id = open("../../scara-robot/configulation/present_z_height.data", "r")
        present_height = float(file_id.read())
    except OSError:
        print("Command line argument needed: Put ~/scara-robot/configulation/present_z_height.data")
        exit()

    # For IK
    tdof_scara = tokuiten_scara.ThreeDoF_SCARA()
    # For Reaching, Gripping 
    tsr = harvesting_movement.TokuitenScaraRobot(present_height)
    # present_height = tsr.get_z_height()
    
    click_point_manager = ClickPointManager(tsr)

    cv2.namedWindow('RealsenseImage', cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback('RealsenseImage',click_point_manager.get_click_coords)
    
    while True:

        click_point_manager.flag_click_change_False()

        # Generate coordinate position
        while True:
            # Reachable from Scara-Robot?
            while True:
                x_target= rnd.uniform(5, 70)/100
                y_target = rnd.uniform(-40, 40)/100
                z_target = rnd.uniform(10, 45)/100
                print(f"Candidate; x:{x_target}, y:{y_target}, z:{z_target}")
                if arm_reachable_check(x=x_target, y=y_target, z=z_target, tdof_scara=tdof_scara):
                    x_scara = x_target
                    y_scara = y_target
                    z_scara = z_target
                    print(f"x:{x_target}, y:{y_target}, z:{z_target}")
                    break

            # Transform coords_scara to coords_cobot
            x_cobot, y_cobot, z_cobot = scara_to_mycobot_coords(x=x_target, y=y_target, z=z_target)
            rx, ry, rz = -90, 0, -90
            coords_scara = [x_cobot, y_cobot, z_cobot, rx, ry, rz]
        
            # Reachable from MyCobot?
            if mycobot.cobot_reachable_check(coords=coords_scara):
                break
        print(f"::send_coords() ==> {coords_scara}")
        
        click_point_manager.flag_click_change_False()

        # Show Depth&Color Image
        while cv2.waitKey(1)<0: #click_point_manager.flag_click:
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            color_intrinsics = color_frame.profile.as_video_stream_profile().intrinsics

            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            images = np.hstack((color_image, depth_colormap))

            cv2.imshow("RealsenseImage",images)

            click_point_manager.update_depth_frame(depth_frame=depth_frame)
            if click_point_manager.flag_click:
                input_charactors  = input("type \"ok\" to move arm: ")
                if input_charactors == "ok":
                    break
                else:
                    click_point_manager.flag_click_change_False()

        (x_target, y_target, z_target) = click_point_manager.coords_target
        # Click and harvest
        """
        This part is same as click_and harvest.py
        """
        x_home_position = 0.0
        y_home_position = 370.0/1000
        tool_angle_home_rad = 0.0

        tool_angle_target_rad = 0.0/180*math.pi
        Preparation_distance_from_tomato = 50.0/1000 # [m]

        x_front = x_target + Preparation_distance_from_tomato * math.sin( tool_angle_target_rad )
        y_front = y_target - Preparation_distance_from_tomato * math.cos( tool_angle_target_rad )


        z_present = tsr.get_z_height() 
        tsr.reach_joint_space(x_home_position, y_home_position, z_present, tool_angle_home_rad)
        file_id = open("../../scara-robot/configulation/present_z_height.data", "w")
        file_id.write(str( float(z_present) ))
        file_id.close()


        # Preparation
        tsr.open_gripper()
        tsr.move_gripper_initial_position()

        # Reaching to the front side of a target tomato
        tsr.reach_joint_space(x_front, y_front, z_target, tool_angle_target_rad)
        file_id = open("../../scara-robot/configulation/present_z_height.data", "w")
        file_id.write(str( float(z_present) ))
        file_id.close()

        time.sleep(1)

        tsr.reach(x_target, y_target, z_target, tool_angle_target_rad)
        file_id = open("../../scara-robot/configulation/present_z_height.data", "w")
        file_id.write(str( float(z_present) ))
        file_id.close()

        tsr.close_gripper()
        tsr.twist_gripper()
        #tsr.twist_gripper()

        #time.sleep(5)

        tsr.move_gripper_initial_position()

        z_present = tsr.get_z_height()
        tsr.reach_joint_space(x_home_position, y_home_position, z_present, tool_angle_home_rad)
        file_id = open("../../scara-robot/configulation/present_z_height.data", "w")
        file_id.write(str( float(z_present) ))
        file_id.close()

        time.sleep(3)
        tsr.open_gripper()


        # Success?
        while True:
            input_success = input("Success? (yes:'y', no:'n'): ")
            input_confirm = input("Is input correct? (yes:'y', no:'n'): ")
            if input_confirm != 'n':
                break
        if input_success == 'n':
            num_trial += 1
            num_success += 0
        else:
            num_trial += 1
            num_success += 1
        
        # Compute success rate
        success_rate = 100*(num_success/num_trial)
        print("Success rate is {:.2f}% ({}/{})".format(success_rate, num_success, num_trial))
        
        # Continue?
        input_resume = input("Continue? (yes:'y', no:'n'): ")
        if input_resume == 'n':
            break
        
        print("")
        # coords_of_mycobot_before = mycobot.my_get_coords()
        # mycobot.my_send_coords(coords=, speed=70, mode=0)
        # time.sleep(3)
        # coords_of_mycobot_after = mycobot.my_get_coords()
        # print(coords_of_mycobot_after)
    # print("Finished!")
    
    cv2.destroyAllWindows()
    print("\n====== Final Report ======")
    print("Success rate is {:.2f}% ({}/{})".format(success_rate, num_success, num_trial))
    
