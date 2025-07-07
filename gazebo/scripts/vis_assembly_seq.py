from utils import *
import rospy
import rospkg
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from robot_digital_twin.srv import TriggerImageSave, TriggerImageSaveRequest
from std_msgs.msg import Float64
import rosnode
import argparse
import os
import glob

class Brick():
    def __init__(self, graph_node, lego_lib, brick_cnt, brick_id):
        self.brick_id = self.read_brick_id(brick_id)
        self.x = graph_node["x"] if "x" in graph_node.keys() else -1
        self.y = graph_node["y"] if "y" in graph_node.keys() else -1
        self.z = graph_node["z"] + 1 if "z" in graph_node.keys() else -1
        self.ori = graph_node["ori"] if "ori" in graph_node.keys() else -1
        self.press_side = graph_node["press_side"] if "press_side" in graph_node.keys() else 1
        self.press_offset = graph_node["press_offset"] if "press_offset" in graph_node.keys() else 0
        self.seq = brick_cnt[self.brick_id]
        self.height = lego_lib[str(self.brick_id)]["height"]
        self.width = lego_lib[str(self.brick_id)]["width"]
        self.name = "b" + str(self.brick_id) + "_" + str(self.seq)
    
    def read_brick_id(self, in_id):
        if(in_id in [2, 3, 4, 5, 6, 9, 10, 12]):
            return in_id
        elif(in_id == 7 or in_id == 8):
            return 5
        elif(in_id == 11):
            return 9
        return 0

class Lego():
    def __init__(self, task_fname, envsetup_fname, lego_lib, plate_x=0, plate_y=0, plate_z=0, plate_height=48, plate_width=48):
        self.plate_x = plate_x
        self.plate_y = plate_y
        self.plate_z = plate_z + 0.0016
        self.plate_height = plate_height
        self.plate_width = plate_width
        self.plate_pose = np.matrix([[1, 0, 0, self.plate_x],
                                     [0, 1, 0, self.plate_y],
                                     [0, 0, 1, self.plate_z],
                                     [0, 0, 0, 1]])
        self.brick_height_m = 0.0096
        self.P_len = 0.008
        self.brick_len_offset = 0.0002
        self.task_graph = load_json(task_fname)
        self.lego_lib = load_json(lego_lib)
        self.envsetup = load_json(envsetup_fname)
        self.brick_cnt = dict()

        r1_j1_topic = "/r1/joint1_position_controller/command"
        r1_j2_topic = "/r1/joint2_position_controller/command"
        r1_j3_topic = "/r1/joint3_position_controller/command"
        r1_j4_topic = "/r1/joint4_position_controller/command"
        r1_j5_topic = "/r1/joint5_position_controller/command"
        r1_j6_topic = "/r1/joint6_position_controller/command"
        self.r1_j1_pub = rospy.Publisher(r1_j1_topic, Float64, queue_size=1)
        self.r1_j2_pub = rospy.Publisher(r1_j2_topic, Float64, queue_size=1)
        self.r1_j3_pub = rospy.Publisher(r1_j3_topic, Float64, queue_size=1)
        self.r1_j4_pub = rospy.Publisher(r1_j4_topic, Float64, queue_size=1)
        self.r1_j5_pub = rospy.Publisher(r1_j5_topic, Float64, queue_size=1)
        self.r1_j6_pub = rospy.Publisher(r1_j6_topic, Float64, queue_size=1)
        self.r1_j1_msg = Float64()
        self.r1_j2_msg = Float64()
        self.r1_j3_msg = Float64()
        self.r1_j4_msg = Float64() 
        self.r1_j5_msg = Float64()
        self.r1_j6_msg = Float64()

        r2_j1_topic = "/r2/joint1_position_controller/command"
        r2_j2_topic = "/r2/joint2_position_controller/command"
        r2_j3_topic = "/r2/joint3_position_controller/command"
        r2_j4_topic = "/r2/joint4_position_controller/command"
        r2_j5_topic = "/r2/joint5_position_controller/command"
        r2_j6_topic = "/r2/joint6_position_controller/command"
        self.r2_j1_pub = rospy.Publisher(r2_j1_topic, Float64, queue_size=1)
        self.r2_j2_pub = rospy.Publisher(r2_j2_topic, Float64, queue_size=1)
        self.r2_j3_pub = rospy.Publisher(r2_j3_topic, Float64, queue_size=1)
        self.r2_j4_pub = rospy.Publisher(r2_j4_topic, Float64, queue_size=1)
        self.r2_j5_pub = rospy.Publisher(r2_j5_topic, Float64, queue_size=1)
        self.r2_j6_pub = rospy.Publisher(r2_j6_topic, Float64, queue_size=1)
        self.r2_j1_msg = Float64()
        self.r2_j2_msg = Float64()
        self.r2_j3_msg = Float64()
        self.r2_j4_msg = Float64()
        self.r2_j5_msg = Float64()
        self.r2_j6_msg = Float64()
        
    def parse_brick(self, graph_node, brick_id):
        if(brick_id not in self.brick_cnt.keys()):
            self.brick_cnt[brick_id] = 1
        else:
            self.brick_cnt[brick_id] += 1
       
        return Brick(graph_node, self.lego_lib, self.brick_cnt, brick_id)
    
    def calc_brick_loc(self, graph_node, brick_id):
        brick = self.parse_brick(graph_node, brick_id)
        topleft_offset = np.identity(4)
        brick_offset = np.identity(4)
        brick_center_offset = np.identity(4)
        z_90 = np.matrix([[0, -1, 0, 0],
                          [1, 0, 0, 0],
                          [0, 0, 1, 0],
                          [0, 0, 0, 1]])
        brick_offset[0, 3] = brick.x * self.P_len - self.brick_len_offset
        brick_offset[1, 3] = brick.y * self.P_len - self.brick_len_offset
        brick_offset[2, 3] = (brick.z-1) * self.brick_height_m

        brick_center_offset[0, 3] = (brick.height * self.P_len - self.brick_len_offset) / 2.0
        brick_center_offset[1, 3] = (brick.width * self.P_len - self.brick_len_offset) / 2.0
        brick_center_offset[2, 3] = 0

        topleft_offset[0, 3] = -(self.plate_height * self.P_len - self.brick_len_offset) / 2.0
        topleft_offset[1, 3] = -(self.plate_width * self.P_len - self.brick_len_offset) / 2.0
        topleft_offset[2, 3] = 0

        out_pose = self.plate_pose @ topleft_offset @ brick_offset @ brick_center_offset
        if(brick.ori == 1):
             brick_center_offset[1, 3] = -brick_center_offset[1, 3]
             out_pose = self.plate_pose @ topleft_offset @ brick_offset @ z_90 @ brick_center_offset
        return brick.name, out_pose
    
    def _load_matrix_from_string(self, data_string):
            """Helper function to load numpy matrix from a multiline string."""
            rows = data_string.strip().split('\n')
            return np.matrix([list(map(float, row.split())) for row in rows])

    def _T_matrix(self, alpha, a, d, theta):
        """Calculates the transformation matrix for a single DH link."""
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        cos_alpha = np.cos(alpha)
        sin_alpha = np.sin(alpha)

        T = np.matrix([
            [cos_alpha, -sin_alpha * cos_theta,  sin_alpha * sin_theta, d * cos_alpha],
            [sin_alpha,  cos_alpha * cos_theta, -cos_alpha * sin_theta, d * sin_alpha],
            [        0,            sin_theta,            cos_theta,             a],
            [        0,                    0,                    0,             1]
        ])
        return T

    def FK(self, joint_angles_rad, dh_params, base_frame):
        """Calculates Forward Kinematics based on DH parameters."""
        T = base_frame.copy()
        num_joints = dh_params.shape[0]

        if len(joint_angles_rad) != num_joints:
             raise ValueError(f"Number of joint angles ({len(joint_angles_rad)}) does not match number of DH rows ({num_joints})")

        for i in range(num_joints):
            # DH parameters from file: alpha, a, d, theta_offset
            alpha_offset, a, d, theta = dh_params[i, 0], dh_params[i, 1], dh_params[i, 2], dh_params[i, 3]
            # Calculate actual joint angle for the link
            alpha = joint_angles_rad[i] + alpha_offset
            # Get transformation for this link
            T_link = self._T_matrix(alpha, a, d, theta)
            # Multiply into the total transformation
            T = T @ T_link
        return T

    def calc_attached_brick_loc(self, graph_node, brick_id):
        """Calculates the world pose of a brick attached to a robot."""
        brick = self.parse_brick(graph_node, brick_id) # Gets name, id etc.
        robot_id = graph_node["attached_robot_id"]
        manipulate_type = graph_node["manipulate_type"] # mode in C++

        # Fixed joint angles (degrees to radians)
        joint_angles_deg = np.array([0.0, -15.456, -40.357, 0.0, -65.099, 0.0])
        joint_angles_rad = np.deg2rad(joint_angles_deg)

        # --- Define DH Parameters and Base Frames ---
        # DH Parameters (alpha, a, d, theta_offset)
        dh_tool_str = """
            0 0 0.00 -1.57079632679
            -1.57079632679 0 0.26 3.1415926
            0 0 0.015 -1.57079632679
            0 -0.29 0 1.57079632679
            0 0 0 -1.57079632679
            0 -0.1954 0.0007 3.1415926
        """
        dh_tool_alt_str = """
            0 0 0 -1.57079632679
            -1.57079632679 0 0.26 3.1415926
            0 0 0.015 -1.57079632679
            0 -0.29 0 1.57079632679
            0 0 0 -1.57079632679
            0 -0.1896 -0.0174 3.1415926
        """
        dh_tool = self._load_matrix_from_string(dh_tool_str)
        dh_tool_alt = self._load_matrix_from_string(dh_tool_alt_str)

        # Base Frames
        r1_base_str = """
        1 0 0 0
        0 1 0 0
        0 0 1 1.2546
        0 0 0 1
        """
        
        r2_base_str = """
        -0.99998762 -0.00497638 0.00000000 0.88072888
        0.00497638 -0.99998762 0.00000000 -0.01296058
        0 0 1 1.2546
        0 0 0 1
        """
        r1_base = self._load_matrix_from_string(r1_base_str)
        r2_base = self._load_matrix_from_string(r2_base_str)
        # --- End Definitions ---

        # Select parameters based on robot_id and manipulate_type
        if robot_id == 1: # Assuming robot ID 1 corresponds to r1
            base_frame = r1_base
            dh_params = dh_tool_alt if manipulate_type == 1 else dh_tool
        elif robot_id == 2: # Assuming robot ID 2 corresponds to r2
            base_frame = r2_base
            # Assuming r2 also uses alt DH for type 1, adjust if needed
            dh_params = dh_tool_alt if manipulate_type == 1 else dh_tool
        else:
            rospy.logerr(f"Unknown robot_id: {robot_id}")
            return None, None # Return None if robot_id is invalid

        # Calculate Forward Kinematics to get the tool flange pose
        T_flange = self.FK(joint_angles_rad, dh_params, base_frame)

        # T_init corresponds to the FK result potentially modified by manipulate_type
        T_init = T_flange
        if manipulate_type == 1:
            # This is the y_p90, z_180 from update_bricks in C++
            y_p90_mode1 = np.matrix([
                [ 0, 0, 1, 0],
                [ 0, 1, 0, 0],
                [-1, 0, 0, 0],
                [ 0, 0, 0, 1]
            ])
            z_180_mode1 = np.matrix([
                [-1, 0, 0, 0],
                [ 0,-1, 0, 0],
                [ 0, 0, 1, 0],
                [ 0, 0, 0, 1]
            ])
            T_init = T_flange @ y_p90_mode1 @ z_180_mode1 # Apply transforms relative to flange

        # --- Apply transformations from C++ update function ---
        # Define standard rotation matrices used in update()
        y_180 = np.matrix([
            [-1, 0, 0, 0],
            [ 0, 1, 0, 0],
            [ 0, 0,-1, 0],
            [ 0, 0, 0, 1]
        ])
        z_180 = np.matrix([
            [-1, 0, 0, 0],
            [ 0,-1, 0, 0],
            [ 0, 0, 1, 0],
            [ 0, 0, 0, 1]
        ])
        z_90 = np.matrix([
            [ 0,-1, 0, 0],
            [ 1, 0, 0, 0],
            [ 0, 0, 1, 0],
            [ 0, 0, 0, 1]
        ])

        # Apply base rotations: new_brick_T = T_init * y_180 * z_180;
        T_intermediate = T_init @ y_180 @ z_180

        # Calculate and apply press_side offset transformation ('tmp' in C++)
        tmp_offset = np.identity(4)
        brick_height_studs = brick.height
        brick_width_studs = brick.width
        # Use the press_offset from the brick object, which defaults to 0 if not in graph_node
        press_offset_val = brick.press_offset

        if brick.press_side == 1:
            tx = (brick_height_studs * self.P_len - self.brick_len_offset) / 2.0
            if brick_width_studs % 2 == 0: # Even width
                center_press_offset_idx = (brick_width_studs // 2) - 1
                ty = (center_press_offset_idx - press_offset_val) * self.P_len
            else: # Odd width (must be 1)
                ty = -(self.P_len - self.brick_len_offset) / 2.0 # Matches C++ for width=1

            tmp_offset[0, 3] = tx
            tmp_offset[1, 3] = ty
            # Apply the tmp offset: new_brick_T = new_brick_T * tmp;
            T_brick = T_intermediate @ tmp_offset

        elif brick.press_side == 4:
            tx = -(brick_height_studs * self.P_len - self.brick_len_offset) / 2.0
            if brick_width_studs % 2 == 0: # Even width
                center_press_offset_idx = (brick_width_studs // 2) - 1
                ty = (center_press_offset_idx - press_offset_val) * self.P_len
            else: # Odd width (must be 1)
                ty = (self.P_len - self.brick_len_offset) / 2.0 # Matches C++ for width=1

            tmp_offset[0, 3] = tx
            tmp_offset[1, 3] = ty
            # Apply the tmp offset: new_brick_T = new_brick_T * z_180 * tmp;
            T_brick = T_intermediate @ z_180 @ tmp_offset

        elif brick.press_side == 2:
            ty = -(brick_width_studs * self.P_len - self.brick_len_offset) / 2.0
            if brick_height_studs % 2 == 0: # Even height
                center_press_offset_idx = (brick_height_studs // 2) - 1
                tx = (press_offset_val - center_press_offset_idx) * self.P_len
            else: # Odd height (must be 1)
                 tx = -(self.P_len - self.brick_len_offset) / 2.0 # Matches C++ for height=1

            tmp_offset[0, 3] = tx
            tmp_offset[1, 3] = ty
            # Apply the tmp offset: new_brick_T = new_brick_T * z_90 * tmp;
            T_brick = T_intermediate @ z_90 @ tmp_offset

        elif brick.press_side == 3:
            ty = (brick_width_studs * self.P_len - self.brick_len_offset) / 2.0
            if brick_height_studs % 2 == 0: # Even height
                center_press_offset_idx = (brick_height_studs // 2) - 1
                tx = (press_offset_val - center_press_offset_idx) * self.P_len
            else: # Odd height (must be 1)
                tx = (self.P_len - self.brick_len_offset) / 2.0 # Matches C++ for height=1

            tmp_offset[0, 3] = tx
            tmp_offset[1, 3] = ty
            # Apply the tmp offset: new_brick_T = new_brick_T * z_90 * z_180 * tmp;
            T_brick = T_intermediate @ z_90 @ z_180 @ tmp_offset

        else:
            # Default if press_side is unknown
            rospy.logwarn(f"Unknown press side {brick.press_side}, using intermediate transform.")
            T_brick = T_intermediate


        return brick.name, T_brick

    def set_pose(self, T, name):
        x = T[0, 3]
        y = T[1, 3]
        z = T[2, 3]
        r = R.from_matrix(T[:3, :3])
        quat = r.as_quat()
        state_msg = ModelState()
        state_msg.model_name = name
        state_msg.pose.position.x = x
        state_msg.pose.position.y = y
        state_msg.pose.position.z = z
        state_msg.pose.orientation.x = quat[0]
        state_msg.pose.orientation.y = quat[1]
        state_msg.pose.orientation.z = quat[2]
        state_msg.pose.orientation.w = quat[3]

        rospy.wait_for_service('/gazebo/set_model_state')
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state(state_msg)
        return resp.success
    
    def add_environment(self):
        self.reset()
        self.set_pose(self.plate_pose, "assemble_plate")
        for key in self.envsetup.keys():
            node = self.envsetup[key]
            if "fixed" in node.keys() and node["fixed"] == True:
                # key is of the foramt "b{id}_{cnt}", separate by "_"
                brick_id = int(key.split("_")[0][1:])
                bname, T = self.calc_brick_loc(node, brick_id)
                ret = self.set_pose(T, bname)
                if(not ret):
                    print(bname, "failed!")
                #time.sleep(0.1)


    def add_step(self, step):
        key = str(step)
        node = self.task_graph[key]
        print("Adding step: ", key, "x:", node["x"], "y:", node["y"], "z:", node["z"], "ori:", node["ori"], "id:", node["brick_id"])
        if "brick_id" not in node.keys():
            print("No brick_id in task graph node, skipping...")
            return

        brick_id = node["brick_id"]
        bname, T = self.calc_brick_loc(node, brick_id)
        ret = self.set_pose(T, bname)
        if(not ret):
            print(bname, "failed!")
        #time.sleep(0.1)
    
    def reset(self):
        for bid in range(13):
            for cnt in range(1, 1000):
                name = "b" + str(bid) + "_" + str(cnt)
                ret = self.set_pose(np.identity(4), name)
                if(not ret):
                    self.brick_cnt[bid] = 0
                    break
    
    def update_robot(self, pose1, pose2):
        i = 0
        while i < 20:
            i += 1
            self.r1_j1_msg.data = pose1[0]
            self.r1_j2_msg.data = pose1[1]
            self.r1_j3_msg.data = pose1[2]
            self.r1_j4_msg.data = pose1[3]
            self.r1_j5_msg.data = pose1[4]
            self.r1_j6_msg.data = pose1[5]
            self.r1_j1_pub.publish(self.r1_j1_msg)
            self.r1_j2_pub.publish(self.r1_j2_msg)
            self.r1_j3_pub.publish(self.r1_j3_msg)
            self.r1_j4_pub.publish(self.r1_j4_msg)
            self.r1_j5_pub.publish(self.r1_j5_msg)
            self.r1_j6_pub.publish(self.r1_j6_msg)

            self.r2_j1_msg.data = pose2[0]
            self.r2_j2_msg.data = pose2[1]
            self.r2_j3_msg.data = pose2[2]
            self.r2_j4_msg.data = pose2[3]
            self.r2_j5_msg.data = pose2[4]
            self.r2_j6_msg.data = pose2[5]
            self.r2_j1_pub.publish(self.r2_j1_msg)
            self.r2_j2_pub.publish(self.r2_j2_msg)
            self.r2_j3_pub.publish(self.r2_j3_msg)
            self.r2_j4_pub.publish(self.r2_j4_msg)
            self.r2_j5_pub.publish(self.r2_j5_msg)
            self.r2_j6_pub.publish(self.r2_j6_msg)
            time.sleep(0.1)
    
    def save_imgs(self, save_dir, prefix):
        rospy.wait_for_service('/save_gazebo_images')
        save_service = rospy.ServiceProxy('/save_gazebo_images', TriggerImageSave)
        req = TriggerImageSaveRequest()
        req.base_save_path = os.path.join(save_dir, prefix)
        print("Saving images to: ", req.base_save_path)

        resp = save_service(req)
        if resp.success:
            rospy.loginfo("Images saved successfully.")
            print(resp.image_path_cam1, resp.image_path_cam2)
            # move cam1 image to cam1 folder
            cam1_path = os.path.join(save_dir, "cam1", os.path.basename(resp.image_path_cam1))
            cam2_path = os.path.join(save_dir, "cam2", os.path.basename(resp.image_path_cam2))
            depth1_path = os.path.join(save_dir, "cam1", os.path.basename(resp.depth_path_cam1))
            depth2_path = os.path.join(save_dir, "cam2", os.path.basename(resp.depth_path_cam2))
            depth1_np_path = os.path.join(save_dir, "cam1", os.path.basename(resp.depth_path_cam1).replace(".png", ".npz"))
            depth2_np_path = os.path.join(save_dir, "cam2", os.path.basename(resp.depth_path_cam2).replace(".png", ".npz"))
            os.rename(resp.image_path_cam1, cam1_path)
            os.rename(resp.image_path_cam2, cam2_path)
            os.rename(resp.depth_path_cam1, depth1_path)
            os.rename(resp.depth_path_cam2, depth2_path)
            os.rename(resp.depth_path_cam1.replace(".png", ".npz"), depth1_np_path)
            os.rename(resp.depth_path_cam2.replace(".png", ".npz"), depth2_np_path)

        else:
            rospy.logerr("Failed to save images.")
            print(req.message)

if __name__ == '__main__':
    rospy.init_node('vis_lego')
    rospy.sleep(1)
    
    # the task is xxxx.json, read xxxx from the command line
    parser = argparse.ArgumentParser(description='Visualize Lego task graph')
    parser.add_argument('--base_dir', type=str, default='.', help='Base directory for tasks and lego library.')
    parser.add_argument('--save_dir', type=str, default='./outputs', help='Directory to save images.')
    parser.add_argument('--task', type=str, help='Specific task name (without .json) to visualize.')
    args = parser.parse_args()

    tasks_dir = os.path.join(args.base_dir, "assembly_tasks")
    envsetup_dir = os.path.join(args.base_dir, "env_setup")
    lego_lib_path = os.path.join(args.base_dir, "lego_library.json")
    plate_x_val = 0.40842053781513565
    plate_y_val = 0.04519264491562785
    plate_z_val = 0.1899 + 0.926
    home_pose = np.array([0.0, -15.456, -40.357, 0, -65.099, 0]) / 180.0 * np.pi

    task_name = args.task

    rospy.loginfo(f"--- Processing task: {task_name} ---")
    task_fname = os.path.join(tasks_dir, task_name + ".json")
    envsetup_fname = os.path.join(envsetup_dir, f'env_setup_{task_name}.json')
    save_dir = os.path.join(args.save_dir, task_name)
    if not os.path.exists(save_dir):
        os.makedirs(save_dir, exist_ok=True)
    os.makedirs(os.path.join(save_dir, "cam1"), exist_ok=True)
    os.makedirs(os.path.join(save_dir, "cam2"), exist_ok=True)
    print("make dir at ", os.path.join(save_dir, "cam1"), os.path.join(save_dir, "cam2"))

    if not os.path.exists(task_fname):
        rospy.logerr(f"Task file not found: {task_fname}")
        exit(1) 

    #try:
    lego = Lego(task_fname, envsetup_fname, lego_lib_path,
                plate_x=plate_x_val,
                plate_y=plate_y_val,
                plate_z=plate_z_val)

    lego.update_robot(home_pose, home_pose)
    rospy.sleep(1) # Give robot time to settle

    lego.add_environment()
    rospy.loginfo("Environment setup complete. Waiting before adding steps...")
    
    rospy.sleep(3) # Wait for visualization to stabilize
    lego.save_imgs(save_dir, '0000')
    rospy.loginfo("Initial images saved. Starting to add steps...")

    for step in range(1, len(lego.task_graph)+1):
        lego.add_step(step)
        rospy.loginfo(f"Added step {step} to the environment.")
        rospy.sleep(3)
        lego.save_imgs(save_dir, f'{step:04d}')

    
    rospy.loginfo(f"--- Finished task: {task_name} ---")
    rospy.sleep(1) # Small delay before next task

    #except Exception as e:
    #    rospy.logerr(f"Error processing task {task_name}: {e}")

