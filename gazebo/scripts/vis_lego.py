from utils import *
import rospy
import rospkg
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import rosnode
import argparse
import os
import glob

class Brick():
    def __init__(self, graph_node, lego_lib, brick_cnt):
        self.brick_id = self.read_brick_id(graph_node["brick_id"])
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
    def __init__(self, task_fname, lego_lib, plate_x=0, plate_y=0, plate_z=0, plate_height=48, plate_width=48):
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
        self.brick_cnt = dict()

    def parse_brick(self, graph_node):
        brick_id = graph_node["brick_id"]
        if(brick_id not in self.brick_cnt.keys()):
            self.brick_cnt[brick_id] = 1
        else:
            self.brick_cnt[brick_id] += 1
        return Brick(graph_node, self.lego_lib, self.brick_cnt)
    
    def calc_brick_loc(self, graph_node):
        brick = self.parse_brick(graph_node)
        topleft_offset = np.identity(4)
        brick_offset = np.identity(4)
        brick_center_offset = np.identity(4)
        z_90 = np.matrix([[0, -1, 0, 0],
                          [1, 0, 0, 0],
                          [0, 0, 1, 0],
                          [0, 0, 0, 1]])
        brick_offset[0, 3] = brick.x * self.P_len - self.brick_len_offset
        brick_offset[1, 3] = brick.y * self.P_len - self.brick_len_offset
        brick_offset[2, 3] = brick.z * self.brick_height_m

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
            print(T)
            T = T @ T_link
        return T

    def calc_attached_brick_loc(self, graph_node):
        """Calculates the world pose of a brick attached to a robot."""
        brick = self.parse_brick(graph_node) # Gets name, id etc.
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
        0 0 1 0.33
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
        if robot_id == 0: # Assuming robot ID 1 corresponds to r1
            base_frame = r1_base
            dh_params = dh_tool_alt if manipulate_type == 1 else dh_tool
        elif robot_id == 1: # Assuming robot ID 2 corresponds to r2
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
        
    def visualize(self):
        self.reset()
        self.set_pose(self.plate_pose, "assemble_plate")
        for key in self.task_graph.keys():
            node = self.task_graph[key]
            if "attached_robot_id" in node.keys():
                bname, T = self.calc_attached_brick_loc(node)
                print("attached", bname, T)
            else:
                bname, T = self.calc_brick_loc(node)
                print(bname, T)
            ret = self.set_pose(T, bname)
            if(not ret):
                print(bname, "failed!")
            time.sleep(0.1)
    
    def reset(self):
        for bid in range(13):
            for cnt in range(1, 1000):
                name = "b" + str(bid) + "_" + str(cnt)
                ret = self.set_pose(np.identity(4), name)
                if(not ret):
                    self.brick_cnt[bid] = 0
                    break

if __name__ == '__main__':
    rospy.init_node('vis_lego')
    rospy.sleep(1)
    
    # the task is xxxx.json, read xxxx from the command line
    parser = argparse.ArgumentParser(description='Visualize Lego task graph')
    parser.add_argument('--task', type=str, help='Specific task name (without .json) to visualize.')
    parser.add_argument('--all', action='store_true', help='Visualize all tasks found in ./scripts/tasks/')
    args = parser.parse_args()

    tasks_dir = "./scripts/tasks/"
    lego_lib_path = "./scripts/lego_library.json"
    plate_x_val = 0.40842053781513565
    plate_y_val = 0.04519264491562785
    plate_z_val = 0.1899 + 0.926
    home_pose = np.array([0.0, -15.456, -40.357, 0, -65.099, 0]) / 180.0 * np.pi

    task_names = []
    if args.all:
        # Find all json files in the tasks directory
        json_files = glob.glob(os.path.join(tasks_dir, '*.json'))
        # Extract task names (filename without extension)
        task_names = [os.path.splitext(os.path.basename(f))[0] for f in json_files]
        if not task_names:
            rospy.logwarn(f"No task files (.json) found in {tasks_dir}")
    elif args.task:
        task_names.append(args.task)
    else:
        rospy.logerr("Please specify a task name with --task or use --all to process all tasks.")
        exit(1)

    rospy.loginfo(f"Processing tasks: {task_names}")

    for task_name in task_names:
        rospy.loginfo(f"--- Processing task: {task_name} ---")
        task_fname = os.path.join(tasks_dir, task_name + ".json")

        if not os.path.exists(task_fname):
            rospy.logerr(f"Task file not found: {task_fname}")
            continue # Skip to the next task

        #try:
        lego = Lego(task_fname, lego_lib_path,
                    plate_x=plate_x_val,
                    plate_y=plate_y_val,
                    plate_z=plate_z_val)

        lego.update_robot(home_pose, home_pose)
        rospy.sleep(1) # Give robot time to settle

        lego.visualize()
        rospy.loginfo(f"Visualization complete for {task_name}. Waiting before saving images...")
        rospy.sleep(4) # Wait for visualization to stabilize

        lego.save_imgs(task_name)
        rospy.loginfo(f"--- Finished task: {task_name} ---")
        rospy.sleep(1) # Small delay before next task

        #except Exception as e:
        #    rospy.logerr(f"Error processing task {task_name}: {e}")

    rospy.loginfo("All tasks processed.")
