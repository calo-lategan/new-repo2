import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from servo_controller_msgs.msg import Grasp, ServosPosition 
from kinematics.kinematics_control import set_pose_target
from kinematics_msgs.srv import GetRobotPose, SetRobotPose
from servo_controller.bus_servo_control import set_servo_position


class GraspNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        self.joints_pub = self.create_publisher(ServosPosition, '/servo_controller', 1)
        self.kinematics_client = self.create_client(SetRobotPose, '/kinematics/set_pose_target')
        self.kinematics_client.wait_for_service()

        self.create_subscription(Grasp, '/grasp', self.grasp_callback, 1)

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()
    def grasp_callback(self, grasp):
        self.set_target(grasp)  # Call the method to set the target. 调用设置目标方法
        if grasp.mode == 'pick':
            self.execute_pick_sequence()
        elif grasp.mode == 'place':
            self.execute_place_sequence()

    def set_target(self, grasp):
        model_name = grasp.mode
        pose_t = grasp.position

        pick_pitch = grasp.pitch
        self.angle = grasp.angle
        self.grasp = grasp
        pose_t.z += 0.05
        msg = set_pose_target(pose_t, pick_pitch,  [-90.0, 90.0], 1.0)
        res = self.send_request(self.kinematics_client, msg)
        self.target1 = res.pulse 

        pose_t.z -= 0.05
        msg = set_pose_target(pose_t, pick_pitch,  [-90.0, 90.0], 1.0)
        res = self.send_request(self.kinematics_client, msg)
        self.targe2 = res.pulse 
        
        pose_t.z += 0.05
        msg = set_pose_target(pose_t, pick_pitch,  [-90.0, 90.0], 1.0)
        res = self.send_request(self.kinematics_client, msg)
        self.targe3 = res.pulse 


    def execute_pick_sequence(self):
        # Handle the grasping action sequence. 处理夹取的动作序列
        queue_list = [
            [self.move_toward, 1],
            [self.move_approach, 1],
            [self.gripper_align, 0.5],
            [self.move_target, 1],
            [self.gripper_move, 0.3],
            [self.move_retreat, 0.5],
            [self.move_toward_init, 1]
        ]
        self.execute_queue(queue_list)

    def execute_place_sequence(self):
        # Handle the placing action sequence. 处理放置的动作序列
        queue_list = [
            [self.move_toward, 1],
            [self.move_approach, 1],
            [self.move_target, 1],
            [self.gripper_move_, 0.05],
            [self.gripper_align, 0.5],
            [self.gripper_move, 0.3],
            [self.move_retreat, 1],
            [self.move_toward_init, 1],
            [self.move_init, 1]
        ]
        self.execute_queue(queue_list)

    def execute_queue(self, queue_list):
        for action, duration in queue_list:
            action(duration)    

    def move_toward(self, t=1): 
        # Move towards the object's direction. 移到朝向物体
        if self.target2 is not None:
            servo_data = self.target2[1][::-1]
            set_servo_position(self.joints_pub, t, ((10, self.grasp.pre_grasp_posture), (1, servo_data[-1]))) 
            time.sleep(t)

    def move_approach(self, t=1.5): 
        # Move above the object. 移到物体上方
        # The reason for recalculating is that the position has been updated, and if there are multiple solutions, the one with the smallest change should be selected. 这里再计算一次是因为位置更新了，如果有多解涉及到取变化最小的解
        # self.target1 = set_pose_target([self.position.x + self.approach.x, 
                                        # self.position.y + self.approach.y, 
                                        # self.position.z + self.approach.z], self.pitch, [-90, 90], 1)
        if self.target1 != []:
            servo_data = self.target1[1]
            set_servo_position(self.joints_pub, t, ((1, servo_data[0]), (2, servo_data[1]), (3, servo_data[2]), (4, servo_data[3])))
            time.sleep(t + 0.1)
    
    def move_target(self, t=1.8):
        # Move to the target position. 移到目标位置
        # self.target2 = set_pose_target([self.position.x , self.position.y, self.position.z], self.pitch, [-90, 90], 1)
        if self.target2 != []:
            servo_data = self.target2[1]
            set_servo_position(self.joints_pub, t, ((1, servo_data[0]), (2, servo_data[1]), (3, servo_data[2]), (4, servo_data[3])))
            time.sleep(t + 0.5)

    def gripper_move_(self, t=0.05):
        # The gripper opens or closes slightly. 夹持器开合一点点
        # gripper_control.set_grasp(self.grasp_pub, t, self.grasp.pre_grasp_posture - 30)
        time.sleep(t + 0.1)

    def gripper_align(self, t=0.5):
        # The gripper aligns. 夹持器对齐
        set_servo_position(self.joints_pub, t, ((5, 500 + int(1000*(self.angle + self.target2[3][-1])/240.0)), ))
        time.sleep(t + 0.3)

    def gripper_move(self, t=0.5):
        # The gripper opens and closes. 夹持器开合
        #print(self.grasp.grasp_posture)
        set_servo_position(self.joints_pub, t, ((10, self.grasp.grasp_posture), ))
        # gripper_control.set_grasp(self.grasp_pub, t, self.grasp.grasp_posture)
        time.sleep(t + 0.3)
    
    def move_retreat(self, t=1):
        # Move away from the object. 远离物体
        # self.target3 = set_pose_target([self.position.x + self.retreat.x, 
                                        # self.position.y + self.retreat.y, 
                                        # self.position.z + self.retreat.z], self.pitch, [-90, 90], 1)
        if self.target3[1] != []:
            servo_data = self.target3[1]
            set_servo_position(self.joints_pub, t, ((1, servo_data[0]), (2, servo_data[1]), (3, servo_data[2]), (4, servo_data[3])))

            time.sleep(t)
        
    
    def move_toward_init(self, t=1): 
        # Move towards the object's direction. 移到朝向物体
        # print(2222, self.target1, self.target2, self.target3)
        set_servo_position(self.joints_pub, t, ((5, 500), (4, 140), (3, 70), (2, 610)))

    def move_init(self, t=1): 
        pass
        #set_servo_position(self.joints_pub, t, ((1, 500),))
        #rospy.sleep(t/1000.0)  
def main():
    node = GraspNode('grasp')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
