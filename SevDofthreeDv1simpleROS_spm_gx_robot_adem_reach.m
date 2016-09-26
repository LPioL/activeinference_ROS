function [g] = SevDofthreeDv1simpleROS_spm_gx_robot_adem_reach(x,v,a,P)
% returns the prediction for a two-joint arm (with action)
% FORMAT [g] = spm_gx_adem_reach(x,v,a,P)
%
% x    - hidden states
%   x(1) - joint angle
%   x(2) - joint angle
%   x(3) - angular velocity
%   x(4) - angular velocity
% v    - causal states
%   v(1) - target location (x)
%   v(2) - target location (y)
%   v(3) - force (cue strength)
% P    - parameters
% a    - action
% P    - parameters
%__________________________________________________________________________
% Copyright (C) 2008 Wellcome Trust Centre for Neuroimaging
 
% Karl Friston
% $Id: spm_gx_adem_reach.m 3893 2010-05-17 18:28:52Z karl $

% stretch (angular) and visual (positional) information (target & arm)
%==========================================================================

% x = full(x);
% pub = rospublisher('/r_arm_controller/command',rostype.trajectory_msgs_JointTrajectory);
% message = rosmessage('trajectory_msgs/JointTrajectory');
% message.JointNames = {'r_shoulder_pan_joint';'r_shoulder_lift_joint';'r_upper_arm_roll_joint';
%     'r_elbow_flex_joint';'r_forearm_roll_joint';'r_wrist_flex_joint';'r_wrist_roll_joint'} ;
% point=rosmessage('trajectory_msgs/JointTrajectoryPoint');
% message.Points=point;
% pos = x(1:7);
% velo = x(8:14);
% %acc = [0.5;0.5;0;0;0;0;0];
% message.Points.Positions = pos;
% message.Points.Velocities = velo;
% message.Points.Accelerations = a;
% send(pub,message)
% pause (10)
% clear pub message


g = SevDofthreeDv1simpleROS_spm_gx_robot_dem_reach(x,v,P);