from utils import *
import rospy
import rospkg
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import rosnode

class Brick():
    def __init__(self, graph_node, lego_lib, brick_cnt):
        self.brick_id = self.read_brick_id(graph_node["brick_id"])
        self.x = graph_node["x"]
        self.y = graph_node["y"]
        self.z = graph_node["z"] + 1
        self.ori = graph_node["ori"]
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
        for i in range(1, len(self.task_graph)+1):
            node = self.task_graph[str(i)]
            bname, T = self.calc_brick_loc(node)
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
    task_fname = "./scripts/task_graph.json"
    lego = Lego(task_fname, "./scripts/lego_library.json")
    lego.visualize()
