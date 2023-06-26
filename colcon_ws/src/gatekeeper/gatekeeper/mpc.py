import numpy as np
import gatekeeper.mpc_planner_OSQP as mpc_planner
import time


import math

from geometry_msgs.msg import Point, Pose, PoseStamped, PoseArray, Vector3
from gatekeeper_msgs.msg import DITrajectory
from decomp_ros_msgs.msg import Polyhedron, PolyhedronStamped

import rclpy
from rclpy.node import Node


def toPoint(x,y,z):
    p = Point()
    p.x = x
    p.y = y
    p.z = z
    return p

def toVector3(x,y,z):
    p = Vector3()
    p.x = x
    p.y = y
    p.z = z
    return p



class MPCPlanner(Node):

    def __init__(self):
        super().__init__("mpc_planner")


        ## subscribers
        self.subscription = self.create_subscription(
                PoseStamped,
                "goal_pose",
                self.goal_callback, 10)

        self.sfc_sub = self.create_subscription(
                PolyhedronStamped,
                "sfc",
                self.sfc_callback, 10)

        ## publishers
        self.di_traj_pub = self.create_publisher(
                DITrajectory,
                "mpc_trajectory", 10
                )
        self.di_traj_viz_pub = self.create_publisher(
                PoseArray,
                "mpc_trajectory_viz", 10
                )

        ## initialize mpc object
        self.mpc = mpc_planner.MPCPlanner(max_accel=200.0)
        self.get_logger().info("Initialize the MPC planner!")

        # initialize the self.state
        self.state = PoseStamped()
        self.state.header.stamp = self.get_clock().now().to_msg()
        self.state.header.frame_id = "camera_depth_optical_frame"

        # initialize the goal
        self.goal_pose = PoseStamped()
        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
        self.goal_pose.header.frame_id = "camera_depth_optical_frame"
        self.goal_pose.pose.position.x = 1.0;

        # start a timer
        timer = self.create_timer(0.25, self.solve_mpc)

    def goal_callback(self, msg):

        p = msg.pose.position

        self.get_logger().info(f"I got a goal position of {p.x}, {p.y}, {p.z}")

        self.goal_pose = msg

        self.goal_pose.pose.position.z = 8.0

    def state_callback(self, msg):

        self.state = msg


    def sfc_callback(self, msg):

        N_sfc = len(msg.poly.a)
        print(f"got sfc with {N_sfc} faces")

        A = np.zeros([N_sfc, 3])
        for i in range(N_sfc):
            A[i, 0] = msg.poly.a[i].x
            A[i, 1] = msg.poly.a[i].y
            A[i, 2] = msg.poly.a[i].z

        self.mpc.set_safe_polyhedron(A, msg.poly.b)

        return


    def solve_mpc(self):
        print("im getting called")

        # set the target location
        gp = self.goal_pose.pose.position
        self.mpc.set_target_location([gp.x, gp.y, gp.z])

        # set the current state

        # set the sfc

        # solve
        res = self.mpc.solve()

        if res:
            print("success")
            self.publish_trajectory()
            return;



        else:
            # print a warning
            return;


    def publish_trajectory(self):
        xs = self.mpc.sol_x
        us = self.mpc.sol_u

        di_msg = DITrajectory()
        di_msg.header.frame_id = "camera_depth_optical_frame"
        di_msg.header.stamp = self.get_clock().now().to_msg()

        di_msg.dt = self.mpc.DT

        for x in xs:
            p = toPoint(x[0],x[2],x[4])
            v = toVector3(x[1], x[3], x[5])
            di_msg.pos.append(p)
            di_msg.vel.append(v)
        for u in us:
            a = toVector3(u[0], u[1], u[2])
            di_msg.acc.append(a)

        # todo: yaw

        self.di_traj_pub.publish(di_msg)

        ## now publish the PoseArray msg
        pose_array_msg = PoseArray()
        pose_array_msg.header = di_msg.header
        for x in xs:
            p = toPoint(x[0],x[2],x[4])
            pose = Pose()
            pose.position = p
            pose_array_msg.poses.append(pose)

        self.di_traj_viz_pub.publish(pose_array_msg)

        




def main(args=None):

    rclpy.init(args=args)

    node = MPCPlanner()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

