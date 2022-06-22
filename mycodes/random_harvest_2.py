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
# from my_utils import realsense_to_arm_position
sys.path.append("../demo/")
from port_setup import setup
# import pprint
# pprint.pprint(sys.path)
import harvesting_movement
import tokuiten_scara
from click_and_harvest import get_click_point
# from my_utils import realsense_to_arm_position

# def setup():
#     """
#     Reference code: ../demo/port_setup.py
#     Modified MyCobot -> TokMyCobot
#     """
#     print("")

#     plist = list(serial.tools.list_ports.comports())
#     idx = 1
#     for port in plist:
#         print("{} : {}".format(idx, port))
#         idx += 1

#     _in = input("\nPlease input 1 - {} to choice:".format(idx - 1))
#     port = str(plist[int(_in) - 1]).split(" - ")[0].strip()
#     print(port)
#     print("")

#     baud = 115200
#     # _baud = input("Please input baud(default:115200):")
#     # try:
#     #     baud = int(_baud)
#     # except Exception:
#     #     pass
#     # print(baud)
#     # print("")

#     DEBUG = False
#     f = input("Wether DEBUG mode[Y/n]:")
#     if f in ["y", "Y", "yes", "Yes"]:
#         DEBUG = True
#     mc = MyCobot(port, baud, debug=DEBUG)
#     return mc

def move_cobot(mycobot, rnd):
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
    mycobot.send_angles([theta_0, theta_1, theta_2, theta_3, theta_4, theta_5], 30)
       
    print("\n::send_angles() ==> angles [{:.1f} {:.1f} {:.1f} {:.1f} {:.1f} {:.1f}]".format(theta_0, theta_1, theta_2, theta_3, theta_4, theta_5))
    # time.sleep(3)
    print("::get_angles()  ==> angles {}".format(mycobot.get_angles()))
    print("get_coords() ==> coords {}".format(mycobot.get_coords()))

class ResultAgg():
    def __init__(self):
        self.num_trial = 0
        self.num_success = 0
        self.success_rate = 0

    def judge_calc(self):
        # Success?
        while True:
            input_success = input("Success? (yes:'y', no:'n'): ")
            input_confirm = input("Is input correct? (yes:'y', no:'n'): ")
            if input_confirm != 'n':
                break
        if input_success == 'n':
            self.num_trial += 1
            self.num_success += 0
        else:
            self.num_trial += 1
            self.num_success += 1
        
        # Compute success rate
        self.success_rate = 100*(self.num_success/self.num_trial)

    def show_result(self):
        print("Success rate is {:.2f}% ({}/{})"\
            .format(self.success_rate, self.num_success, self.num_trial))

def arm_reachable_check(x:float, y:float, z:float, tdof_scara) -> bool:
    """
    HACKME: For checking z-value, if-statement is used.
    """
    print(f"x_target: {x:.2f}, y_target: {y:.2f}, z_target: {z:.2f}")
    if z<5/100 or 45/100<z:
        return False
    try:
        _ = tdof_scara.inverse_kinematics(x, y, 0.0)
        return True
    except:
        return False



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
        self.get_coords = False

    def update_depth_frame(self, depth_frame):
        self.depth_frame = depth_frame
    
    def get_click_coords(self, event, x, y, flags, param):
        """
        1. Get coordinations of RGBD camera
        2. Change RGBD frame into Scara-Robot frame
        3. flag_click: False -> True  
        """
        # print(f"event: {event}, self.flag_click: {self.flag_click}, x: {x}")
        if event != cv2.EVENT_LBUTTONDOWN or x>=640:
            return
        
        depth = self.depth_frame.get_distance(x, y)
        if depth == 0:
            print("ERROR: cannot get depth data ! ")
            return

        self.point_3D = rs.rs2_deproject_pixel_to_point(color_intrinsics, [x, y], depth)
        x_target, y_target, z_target = convert(self.point_3D)
        self.coords_target = [x_target, y_target, z_target]
        # y_target = y_target + 10
        # print("point(rs): ", self.point_3D)
        # print("point(arm): ", self.coords_target)
        input_charactors  = input("type \"ok\" to move arm: ")
        if input_charactors == "ok":
            self.get_coords = True

    

if __name__=='__main__':
    mycobot = setup()
    result = ResultAgg()
    rnd = np.random.RandomState(42)


    while True: # Test-loop

        # while True:
        #     # Move mycobot randomly
        #     time.sleep(1)
        #     if mycobot.get_angles()!=[]:
        #         move_cobot(mycobot=mycobot, rnd=rnd)
        #         print("\nMovement of arm was successed! --> angles", mycobot.get_angles())
        #         break
        #     else:
        #         move_cobot(mycobot=mycobot, rnd=rnd)

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
        cv2.setMouseCallback('RealsenseImage', click_point_manager.get_click_coords)


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
                
            if click_point_manager.get_coords:
                print(click_point_manager.get_coords)
                click_point_manager.get_coords = False
                break
            else:
                click_point_manager.get_coords = False
    
        (x_target, y_target, z_target) = click_point_manager.coords_target
        
        if not arm_reachable_check(x=x_target, y=y_target, z=z_target, tdof_scara=tdof_scara):
            print("arm_reachable_check(x=x_target, y=y_target, z=z_target, tdof_scara=tdof_scara)")
            print(arm_reachable_check(x=x_target, y=y_target, z=z_target, tdof_scara=tdof_scara))
            continue
        else:
            pipeline.stop()
            cv2.waitKey(1)
            cv2.destroyAllWindows()
            cv2.waitKey(1)

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
        # tsr.twist_gripper()

        tsr.move_gripper_initial_position()

        z_present = tsr.get_z_height()
        tsr.reach_joint_space(x_home_position, y_home_position, z_present, tool_angle_home_rad)
        file_id = open("../../scara-robot/configulation/present_z_height.data", "w")
        file_id.write(str( float(z_present) ))
        file_id.close()

        time.sleep(3)
        tsr.open_gripper()

        result.judge_calc()
        result.show_result()