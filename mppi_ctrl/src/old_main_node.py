#!/usr/bin/env python3
"""   
 Software License Agreement (BSD License)
 Copyright (c) 2022 Ulsan National Institute of Science and Technology (UNIST)
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this
 list of conditions and the following disclaimer.
 2. Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************** 
  @author: Hojin Lee <hojinlee@unist.ac.kr>
  @date: September 10, 2022
  @copyright 2022 Ulsan National Institute of Science and Technology (UNIST)
  @brief: ROS node for sampling based nonlinear model predictive controller (Model predictive path integrator) 
  @details: main node for MPPI
"""

from re import L
import rospy
import time
import threading
import numpy as np
import math 
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from hmcl_msgs.msg import vehicleCmd, Waypoints
from visualization_msgs.msg import MarkerArray
from ackermann_msgs.msg import AckermannDriveStamped
from grid_map_msgs.msg import GridMap
import torch 
from mppi_ctrl.mppi_offroad import offroad_mppi
from mppi_ctrl.utils import torch_path_to_marker, quaternion_to_euler, get_odom_euler, get_local_vel,  wrap_to_pi
from mppi_ctrl.gpgridmap import GPGridMap
import rospkg
import yaml
import os 
rospack = rospkg.RosPack()
pkg_dir = rospack.get_path('mppi_ctrl')


class MPPIWarpper:
    def __init__(self):       
        config_name = "hound_mppi.yaml"
        config_path = "/root/catkin_ws/src/hound_core/config/" + config_name
        with open(config_path) as f:
            Config = yaml.safe_load(f)
        self.MPPI_config = Config["MPPI_config"]
        self.debug = Config["debug"]
        self.Dynamics_config = Config["Dynamics_config"]
        self.Cost_config = Config["Cost_config"]
        self.Sampling_config = Config["Sampling_config"]
        self.MPPI_config = Config["MPPI_config"]
        self.Map_config = Config["Map_config"]
        self.map_res = self.Map_config["map_res"]
        self.map_size = self.Map_config["map_size"]
        self.controller = offroad_mppi(Config)

        self.prediction_hz = rospy.get_param('~prediction_hz', default=10)
        self.n_nodes = rospy.get_param('~n_nodes', default=10)
        self.t_horizon = rospy.get_param('~t_horizon', default=2.0)                   
        self.mppi_n_sample = 4000 ## rospy.get_param('~mppi_n_sample', default=4000)                           
        self.torch_device = "cuda:0"   ## Specify the name of GPU 
        self.torch_dtype  = torch.double
        self.dt = self.t_horizon / self.n_nodes*1.0        
         # x, y, psi, vx, vy, wz, z ,  roll, pitch 
         # 0  1  2     3  4   5   6    7, 8   
        self.cur_x = np.transpose(np.zeros([1,9]))        
        self.path_look_ahead_idx = 1
        # Initialize Grid map instance and 3D vehicle model         
        self.local_map = GPGridMap(device = self.torch_device, dt = self.dt)        
        self.VehicleModel = VehicleModel(device_ = self.torch_device, dt = self.dt,N_node = self.n_nodes,  map_info = self.local_map, mppi_n_sample = self.mppi_n_sample)        
        self.local_map.vehicle_model = self.VehicleModel
        
        self.odom_available   = False 
        self.vehicle_status_available = False 
        self.waypoint_available = False 
        self.map_available = False        
        
        self.prev_vehicleCmd = None
        self.vehicleCmd = vehicleCmd()
        self._thread = threading.Thread()        
        
        self.odom = Odometry()
        self.waypoint = PoseStamped()        
        
        odom_topic = "/Odometry"
        
        control_topic = "/lowlevel_ctrl/hound/control"                    
        status_topic = "/is_data_busy"
        global_map_topic = "/traversability_estimation/global_map"        
        
        traj_topic = "/local_traj"
        traj_marker = "/mppi_ref_path"
        self.target_path = None
        self.logging = False
        # Publishers                
        self.status_pub = rospy.Publisher(status_topic, Bool, queue_size=2)            
        # self.control_pub = rospy.Publisher(control_topic, vehicleCmd, queue_size=1, tcp_nodelay=True)        
        self.control_pub = rospy.Publisher(control_topic, AckermannDriveStamped, queue_size=1, tcp_nodelay=True)        
        
        self.ref_traj_marker_pub = rospy.Publisher(traj_marker, MarkerArray, queue_size=2)        
        # Subscribers
        self.local_traj_sub = rospy.Subscriber(traj_topic, Waypoints, self.traj_callback)        
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odom_callback)                        
        
        self.ctrl_sub = rospy.Subscriber(control_topic, vehicleCmd, self.ctrl_callback)        
        
        # self.local_map_sub = rospy.Subscriber(global_map_topic, GridMap, self.gridmap_callback)        

        # controller callback         
        self.cmd_timer = rospy.Timer(rospy.Duration(1/self.prediction_hz), self.cmd_callback)         
        rate = rospy.Rate(1)     
        while not rospy.is_shutdown():            
            msg = Bool()
            msg.data = True
            self.status_pub.publish(msg)          
        rate.sleep()
        
        os.system(
            "rosservice call /elevation_mapping/clear_map"
        )  ## clear the elevation map.
        time.sleep(1)
        ## initialize controller:
        self.main_loop()
    def odom_callback(self, odom):
        if self.imu is None:
            return
        self.obtain_state(odom)
        if not self.state_init:
            self.state_init = True
        self.odom_update = True  ## indicate that a new reading is available

        
        if self.odom_available is False:
            self.odom_available = True 
        

    def main_loop(self):
        ## the pycuda-torch lovechild prefers it if you keep it in a single context rather than invoking
        # it in a callback which causes it to create new contexts faster than it can delete the old ones leading to rapid memory growth
        while not rospy.is_shutdown():
            if (
                self.state_init
                and self.map_init
                and self.goal_init
                and self.odom_update
            ):
                ## generate the control message:
                now = time.time()
                self.process_grid_map()
                speed_limit = np.sqrt(
                    (self.Dynamics_config["D"] * 9.8)
                    / self.path_poses[self.current_wp_index, 3]
                )
                speed_limit = min(speed_limit, self.speed_limit)
                lookahead = np.clip(
                    speed_limit, self.lookahead / 2, self.lookahead
                )  ## speed dependent lookahead
                self.goal, terminate, self.current_wp_index = self.update_goal(
                    self.goal,
                    np.copy(self.state[:3]),
                    self.path_poses,
                    self.current_wp_index,
                    lookahead,
                    wp_radius=self.wp_radius,
                    looping=self.looping,
                )
                self.controller.set_hard_limit(self.hard_limit)
                if terminate:
                    self.goal_init = False
                    ctrl = np.zeros(2)
                else:
                    ctrl = self.controller.update(
                        self.state,
                        self.goal,
                        self.map_elev * self.elevation_multiplier,
                        self.map_norm,
                        self.map_cost,
                        self.map_cent,
                        speed_limit,
                    )
                self.state[15:17] = ctrl
                self.send_ctrl(ctrl)
                self.publish_markers(self.controller.print_states)
                self.odom_update = False
                self.loop_dt = 1e3 * (time.time() - now)
                if self.debug:
                    print("loop_dt: ", self.loop_dt)
            # self.publish_diagnostics()

    def traj_callback(self,msg):         

        self.target_path = np.zeros([4,len(msg.waypoints)])        
        for i in range(len(msg.waypoints)):            
            self.target_path[0,i] = msg.waypoints[i].pose.pose.position.x
            self.target_path[1,i] = msg.waypoints[i].pose.pose.position.y
            self.target_path[3,i] = msg.waypoints[i].pose.pose.position.z
            quat = [msg.waypoints[i].pose.pose.orientation.w,msg.waypoints[i].pose.pose.orientation.x,msg.waypoints[i].pose.pose.orientation.y,msg.waypoints[i].pose.pose.orientation.z]
            [roll, pitch, yaw] = quaternion_to_euler(quat)
            yaw = wrap_to_pi(yaw)
            self.target_path[2,i] = yaw
       
    def gridmap_callback(self,msg):                     
        if self.map_available is False:
            self.map_available = True
        # self.local_map.set_map(msg)
        pose = torch.Tensor([1.0, 0.0, 0.0]).to(dtype=self.torch_dtype, device=self.torch_device)
        if not torch.is_tensor(pose):
            state = torch.tensor(pose)
        
        
    def ctrl_callback(self,msg):
        self.vehicleCmd = msg    
        
  

        
    def cmd_callback(self,timer):
        
        # if self.vehicle_status_available is False:
        #     rospy.loginfo("Vehicle status is not available yet")
        #     return
        if self.odom_available is False:
            rospy.loginfo("Odom is not available yet")
            return
        # elif self.map_available is False:
        #     rospy.loginfo("Map is not available yet")
        #     return        
        if self.target_path is None:
            return


        self.controller.set_hard_limit(self.hard_limit)
        if terminate:
            self.goal_init = False
            ctrl = np.zeros(2)
        else:
            ctrl = self.controller.update(
                self.state,
                self.goal,
                self.map_elev * self.elevation_multiplier,
                self.map_norm,
                self.map_cost,
                self.map_cent,
                speed_limit,
            )


        path = self.target_path.copy()
        min_dist = 1e5
        min_idx = 0
        dist = []
        for i in range(path.shape[1]):
            dist_tmp = math.sqrt((path[0,i] - self.odom.pose.pose.position.x)**2+(path[1,i] - self.odom.pose.pose.position.y)**2)
            dist.append(dist_tmp)
            if min_dist >= dist_tmp:
                min_dist = dist_tmp
                min_idx = i            
        # for waypoint in msg.waypoints:                
        min_idx = np.min([path.shape[1], self.path_look_ahead_idx+min_idx])        
        min_idx+=1
        for i in range(path.shape[0]):
            path[i,:] = np.roll(path[i,:],-1*min_idx)
            path[i,path.shape[1]-min_idx:] = path[i,path.shape[1]-min_idx-1]        
        if path.shape[1] > self.n_nodes:
            path = path[:,:self.n_nodes]
        self.target_path_torch = torch.from_numpy(path).to(device = self.torch_device)
        self.VehicleModel.set_path(self.target_path_torch)
        target_path_marker = torch_path_to_marker(self.target_path_torch)
        self.ref_traj_marker_pub.publish(target_path_marker)
        ###########################################
        current_euler = get_odom_euler(self.odom)
        for i in range(3):
            current_euler[i] = wrap_to_pi(current_euler[i])
        if(abs(current_euler[0]) > 80*math.pi/180):
            return
        
        local_vel = get_local_vel(self.odom, is_odom_local_frame = False)
        
        ## clip this value as vehicle dynamics is invalid in low speed range
        local_vel[0] = np.max([local_vel[0],0.05])

        # x, y, psi, vx, vy, wz, z, roll, pitch 
        # 0  1  2     3  4   5   6 7,    8    \
        self.cur_x = np.transpose(np.array([self.odom.pose.pose.position.x,
                                            self.odom.pose.pose.position.y,
                                            current_euler[2],
                                            local_vel[0],
                                            local_vel[1],
                                            self.odom.twist.twist.angular.z,
                                            self.odom.pose.pose.position.z,                                                   
                                            current_euler[0],
                                            current_euler[1]]))
        self.cur_u = np.transpose(np.array([ self.vehicleCmd.steering,self.vehicleCmd.acceleration]))
        
        start = time.time()                         
        x = torch.tensor(self.cur_x).to(dtype=self.torch_dtype, device=self.torch_device)
        u = torch.tensor(self.cur_u).to(dtype=self.torch_dtype, device=self.torch_device)
        
        action = self.VehicleModel.mppi.command(x)
        
        opt_action = action.cpu().numpy()
        delta = opt_action[3][0]
        acclx = opt_action[3][1]

        ctrl_cmd = AckermannDriveStamped()
        ctrl_cmd.header.stamp = rospy.Time.now()
        
        
        if acclx > 0.0:
            desired_vel = local_vel[0] + acclx*self.dt             
            desired_vel = np.clip(desired_vel,0.8, 1.0)        
        else:                        
            desired_vel = local_vel[0] + acclx*self.dt 
            desired_vel = np.clip(desired_vel,0.0, 1.0)        
            if desired_vel < 0.5:
                desired_vel = 0.0
        

        ctrl_cmd.drive.speed = desired_vel

        # if self.prev_vehicleCmd is not None:
        #     # limit steering change by the maximum delta rate for preventing sudden manuever. 
        #     diff_steering = (delta-self.vehicleCmd.steering)
        #     diff_steering = np.clip(diff_steering,-self.VehicleModel.delta_rate_max*self.dt,self.VehicleModel.delta_rate_max*self.dt)
        #     target_steering = (self.vehicleCmd.steering + diff_steering)
        # else:
        
        target_steering = delta
        ctrl_cmd.drive.steering_angle = -1*target_steering  
        
        self.control_pub.publish(ctrl_cmd)
        self.prev_vehicleCmd = ctrl_cmd
        
        end = time.time()        
        print("time: {:.5f}".format( end-start))

###################################################################################

def main():
    rospy.init_node("mppi_ctrl")    

    MPPIWarpper()

if __name__ == "__main__":
    main()




 
    


