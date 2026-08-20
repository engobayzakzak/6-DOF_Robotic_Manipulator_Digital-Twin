import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, RobotState
from sensor_msgs.msg import JointState
import time
import numpy as np

class MotionPlanBenchmarker(Node):
    def __init__(self):
        super().__init__('motion_plan_benchmarker')

        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self.get_logger().info('Connecting to MoveGroup Action Server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Connected to MoveGroup! Ready to benchmark OMPL planners.')

        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        self.planners = ['RRTConnectkConfigDefault', 'RRTstarkConfigDefault', 'PRMstarkConfigDefault']
        self.trials = 5

        # Traversing between two distinct collision-free workcell configurations
        self.start_pose = [0.0, -0.7854, 1.5708, 0.0, 0.7854, 0.0]     # Standard Ready Pose
        self.goal_pose  = [0.80, -0.5000, 1.2000, 0.0, 0.6000, 0.80]   # Lateral Transited Pose

    def create_goal_message(self, planner_id):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'manipulator'
        goal_msg.request.planner_id = planner_id
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.num_planning_attempts = 10
        goal_msg.planning_options.plan_only = True

        # 1. Explicitly inject Start State with valid Header Stamp
        start_state = RobotState()
        start_state.joint_state.header.stamp = self.get_clock().now().to_msg()
        start_state.joint_state.header.frame_id = 'base_link'
        start_state.joint_state.name = self.joint_names
        start_state.joint_state.position = [float(p) for p in self.start_pose]
        goal_msg.request.start_state = start_state

        # 2. Workspace Bounds Definition
        goal_msg.request.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
        goal_msg.request.workspace_parameters.header.frame_id = 'base_link'
        goal_msg.request.workspace_parameters.min_corner.x = -1.5
        goal_msg.request.workspace_parameters.min_corner.y = -1.5
        goal_msg.request.workspace_parameters.min_corner.z = -0.5
        goal_msg.request.workspace_parameters.max_corner.x = 1.5
        goal_msg.request.workspace_parameters.max_corner.y = 1.5
        goal_msg.request.workspace_parameters.max_corner.z = 2.0

        # 3. Inject Goal Joint Constraints
        constraints = Constraints()
        for i, angle in enumerate(self.goal_pose):
            jc = JointConstraint()
            jc.joint_name = self.joint_names[i]
            jc.position = float(angle)
            jc.tolerance_above = 0.05
            jc.tolerance_below = 0.05
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        goal_msg.request.goal_constraints.append(constraints)
        return goal_msg

    def run_benchmark(self):
        results = {p: {'time': [], 'length': [], 'success': 0} for p in self.planners}

        self.get_logger().info('====================================================')
        self.get_logger().info(' STARTING OMPL ALGORITHM BENCHMARK (5 TRIALS EACH)  ')
        self.get_logger().info('====================================================')

        for planner in self.planners:
            self.get_logger().info(f'\n--- Benchmarking Planner: {planner} ---')
            for trial in range(self.trials):
                goal_msg = self.create_goal_message(planner)

                t0 = time.perf_counter()
                future = self._action_client.send_goal_async(goal_msg)
                rclpy.spin_until_future_complete(self, future)
                goal_handle = future.result()

                if not goal_handle.accepted:
                    self.get_logger().warn(f'Trial {trial+1}: Goal rejected by MoveGroup')
                    continue

                res_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, res_future)
                res = res_future.result().result
                t1 = time.perf_counter()

                plan_time_ms = (t1 - t0) * 1000.0

                if res.error_code.val == 1:
                    traj = res.planned_trajectory.joint_trajectory.points
                    path_len = sum(
                        np.linalg.norm(np.array(traj[k].positions) - np.array(traj[k-1].positions))
                        for k in range(1, len(traj))
                    )
                    results[planner]['time'].append(plan_time_ms)
                    results[planner]['length'].append(path_len)
                    results[planner]['success'] += 1
                    self.get_logger().info(f'Trial {trial+1}: SUCCESS | Plan Time: {plan_time_ms:.2f} ms | Path Length: {path_len:.3f} rad')
                else:
                    self.get_logger().warn(f'Trial {trial+1}: FAILED (Error Code: {res.error_code.val})')

        self.print_summary(results)

    def print_summary(self, results):
        print('\n' + '='*78)
        print(f'{"OMPL PLANNER KINEMATIC BENCHMARK REPORT":^78}')
        print('='*78)
        print(f'{"Planner":<26} | {"Success Rate":<14} | {"Avg Time (ms)":<16} | {"Avg Length (rad)":<16}')
        print('-'*78)
        for p, data in results.items():
            succ_rate = f"{(data['success']/self.trials)*100:.1f}%"
            avg_t = f"{np.mean(data['time']):.2f} ± {np.std(data['time']):.1f}" if data['time'] else "N/A"
            avg_l = f"{np.mean(data['length']):.3f} ± {np.std(data['length']):.2f}" if data['length'] else "N/A"
            print(f'{p:<26} | {succ_rate:<14} | {avg_t:<16} | {avg_l:<16}')
        print('='*78 + '\n')

def main(args=None):
    rclpy.init(args=args)
    benchmarker = MotionPlanBenchmarker()
    try:
        benchmarker.run_benchmark()
    except KeyboardInterrupt:
        pass
    finally:
        benchmarker.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
