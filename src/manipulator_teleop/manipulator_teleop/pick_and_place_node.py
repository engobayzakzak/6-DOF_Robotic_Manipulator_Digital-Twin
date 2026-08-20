import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
import numpy as np
import time

class PickAndPlaceSequencer(Node):
    def __init__(self):
        super().__init__('pick_and_place_sequencer')

        # Safe parameter declaration check
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )

        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        self.current_joint_positions = None

        # Subscribe to active joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.get_logger().info('Connecting to /arm_controller action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Controller Manager connected!')

        # 7-Stage Trajectory Waypoints [Joint angles in radians]
        self.stages = [
            ('STAGE 1: PRE-PICK APPROACH (Hover over Table)', [0.45, -0.65, 1.40, 0.0, 0.82, 0.45], 3.0),
            ('STAGE 2: PICK DESCENT (Grasp Blue Part)',       [0.45, -0.85, 1.70, 0.0, 0.72, 0.45], 2.0),
            ('STAGE 3: POST-PICK LIFT (Clear Table Surface)', [0.45, -0.55, 1.20, 0.0, 0.87, 0.45], 2.0),
            ('STAGE 4: OBSTACLE TRANSIT (Clear Red Pillar)',  [-0.30, -0.40, 1.10, 0.0, 0.87, -0.30], 3.5),
            ('STAGE 5: PLACE DESCENT (Drop Target Zone)',     [-0.60, -0.80, 1.65, 0.0, 0.72, -0.60], 2.5),
            ('STAGE 6: POST-PLACE RETRACT (Ascend)',          [-0.60, -0.50, 1.20, 0.0, 0.87, -0.60], 2.0),
            ('STAGE 7: RETURN TO HOME POSITION',              [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 3.0)
        ]

    def joint_state_callback(self, msg: JointState):
        if len(msg.position) >= 6:
            pos_dict = dict(zip(msg.name, msg.position))
            self.current_joint_positions = [pos_dict.get(j, 0.0) for j in self.joint_names]

    def execute_stage(self, stage_name, target_positions, duration_sec):
        self.get_logger().info(f'>>> Executing: {stage_name}')

        # Prevent zero-distance motion traps
        if self.current_joint_positions is not None:
            dist = np.linalg.norm(np.array(self.current_joint_positions) - np.array(target_positions))
            if dist < 0.02:
                self.get_logger().info(f'Robot already at {stage_name} target. Skipping.')
                return True

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in target_positions]
        point.velocities = [0.0] * 6

        sec = int(duration_sec)
        nanosec = int((duration_sec - sec) * 1e9)
        point.time_from_start = Duration(sec=sec, nanosec=nanosec)

        goal_msg.trajectory.points.append(point)

        send_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error(f'Stage {stage_name} was rejected by controller!')
            return False

        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        result = res_future.result()

        if result.result.error_code == 0:
            self.get_logger().info(f'✓ {stage_name} SUCCESS.')
            return True
        else:
            self.get_logger().warn(f'✗ {stage_name} failed with code: {result.result.error_code}')
            return False

    def run_sequence(self):
        # Wait for the first valid joint state packet
        while rclpy.ok() and self.current_joint_positions is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info(f'Current robot joint configuration: {[round(x, 2) for x in self.current_joint_positions]}')
        self.get_logger().info('Starting 7-Stage Industrial Pick-and-Place Cycle...\n')

        for stage_name, target_pos, duration in self.stages:
            success = self.execute_stage(stage_name, target_pos, duration)
            if not success:
                self.get_logger().error('Sequence aborted.')
                return
            time.sleep(0.3)

        self.get_logger().info('====================================================')
        self.get_logger().info(' AUTONOMOUS PICK-AND-PLACE FULL CYCLE COMPLETED!    ')
        self.get_logger().info('====================================================')

def main(args=None):
    rclpy.init(args=args)
    sequencer = PickAndPlaceSequencer()
    try:
        sequencer.run_sequence()
    except KeyboardInterrupt:
        pass
    finally:
        sequencer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
