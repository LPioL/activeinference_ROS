function [f]= SevDofthreeDv1simpleROS_spm_fx_robot_adem_reach(x,v,a,P)
% returns the flow for a two-joint arm (with action)
% FORMAT [f]= spm_fx_adem_reach(x,v,a,P)
%
% x    - hidden states
%   x(1) - joint angle
%   x(2) - joint angle
%   x(3) - angular velocity
%   x(4) - angular velocity
% v    - cue locations and strength
% a    - action (forces) (x,y)
% P    - parameters
%__________________________________________________________________________
% Copyright (C) 2008 Wellcome Trust Centre for Neuroimaging
 
% Karl Friston
% $Id: spm_fx_adem_reach.m 3893 2010-05-17 18:28:52Z karl $

% evaluate positions
%--------------------------------------------------------------------------
% 



a = full(a);
x = full(x);
% pub = rospublisher('/r_arm_controller/command',rostype.trajectory_msgs_JointTrajectory);
% message = rosmessage('trajectory_msgs/JointTrajectory');
% message.JointNames = {'r_shoulder_pan_joint';'r_shoulder_lift_joint';'r_upper_arm_roll_joint';
%     'r_elbow_flex_joint';'r_forearm_roll_joint';'r_wrist_flex_joint';'r_wrist_roll_joint'} ;
% point=rosmessage('trajectory_msgs/JointTrajectoryPoint');
% message.Points=point;
% pos = [0;x(1);0;x(2);0;0;0];
% velo =  [0;x(3);0;x(4);0;0;0];
% %acc = [0.5;0.5;0;0;0;0;0];
% message.Points.Positions = pos;
% message.Points.Velocities = velo;
% message.Points.Accelerations =  [0;a(1);0;a(2);0;0;0];
% send(pub,message)
% pause (10)
% clear pub message
%[X,velocity,effort] = JointStates();

% 
%      f  = [(x(1)+a(1));     % velocities
%            (x(2)+a(2));
%            (x(3)+a(3));  % accelerations
%            (x(4)+a(4))
%       ]
% %   
%      f  = [(a(1));  % accelerations
%         (a(2));
%         (a(3)); 
%         (a(4));
%         (a(5));  
%         (a(6));
%         (a(7));
%         a(8);     % velocities
%         a(9);
%         a(10);
%         a(11);
%         a(12);
%         a(13);
%         a(14);
%          
%       ];
%   
%   f  = [x(8);     % velocities
%         x(9);
%         x(10);
%         x(11);
%         x(12);
%         x(13);
%         x(14);
%         (a(1));  % accelerations
%         (a(2));
%         (a(3));
%         (a(4));
%         (a(5));
%         (a(6));
%         (a(7));];



k3=4;   % viscosity
k4 =3.21;
visco = 2;
elasticity=1.5; % elasticity
m1=4;
m2=3;

% 
% 
%       f  = [x(8);     % velocities
%         x(9);
%         x(10);
%         x(11);
%         x(12);
%         x(13);
%         x(14);
%        (a(1)-k6*k3*x(8)-k5*k3*(x(1)-pi/2))/10;     % velocities
%        (a(2)-k6*k3*x(9)-k5*k3*(x(2)-pi/2))/10;
%        (a(3)-k6*k3*x(10)-k5*k3*(x(3)-pi/2))/10;
%        (a(4)-k6*k4*x(11)-k5*k4*(x(4)-pi/2))/10;
%        (a(5)-k6*k4*x(12)-k5*k4*(x(5)-pi/2))/10;  
%        (a(6)-k6*k4*x(13)-k5*k4*(x(6)-pi/2))/10;
%        (a(7)-k6*k4*x(14)-k5*k4*(x(7)-pi/2))/10];
   
%    
%          f  = [x(8);     % velocities
%         x(9);
%         x(10);
%         x(11);
%         x(12);
%         x(13);
%         x(14);
%        (a(1)-visco*k3*x(8)-elasticity*(x(1)-pi/2))/m;     
%        (a(2)-visco*k3*x(9)-elasticity*(x(2)-pi/2))/m;
%        (a(3)-visco*k3*x(10)-elasticity*(x(3)-pi/2))/m;
%        (a(4)-visco*k4*x(11)-elasticity*(x(4)-pi/2))/m;
%        (a(5)-visco*k4*x(12)-elasticity*(x(5)-pi/2))/m;  
%        (a(6)-visco*k4*x(13)-elasticity*(x(6)-pi/2))/m;
%        (a(7)-visco*k4*x(14)-elasticity*(x(7)-pi/2))/m]
    
%     f  = [x(8);     % velocities
%         x(9);
%         x(10);
%         x(11);
%         x(12);
%         x(13);
%         x(14);
%         (a(1)-k3*x(8));  % accelerations
%         (a(2)-k3*x(9));
%         (a(3)-k3*x(10)); 
%         (a(4)-k4*x(11));
%         (a(5)-k4*x(12));  
%         (a(6)-k4*x(13));
%         (a(7)-k4*x(14))];  
    
   
    
    
%              f  = [x(8);     % velocities
%         x(9);
%         x(10);
%         x(11);
%         x(12);
%         x(13);
%         x(14);
%        (a(1)-visco*k3*x(8)-elasticity*(x(1)-pi/2))/m1;     
%        (a(2)-visco*k3*x(9)-elasticity*(x(2)-pi/2))/m1;
%        (a(3)-visco*k3*x(10)-elasticity*(x(3)-pi/2))/m1;
%        (a(4)-visco*k4*x(11)-elasticity*(x(4)-pi/2))/m2;
%        (a(5)-visco*k4*x(12)-elasticity*(x(5)-pi/2))/m2;  
%        (a(6)-visco*k4*x(13)-elasticity*(x(6)-pi/2))/m2;
%        (a(7)-visco*k4*x(14)-elasticity*(x(7)-pi/2))/m2];
   
   
                f  = [x(8);     % velocities
        x(9);
        x(10);
        x(11);
        x(12);
        x(13);
        x(14);
       (a(1)-visco*k3*x(8)-elasticity*(x(1)))/m1;     
       (a(2)-visco*k3*x(9)-elasticity*(x(2)))/m1;
       (a(3)-visco*k3*x(10)-elasticity*(x(3)))/m1;
       (a(4)-visco*k4*x(11)-elasticity*(x(4)))/m2;
       (a(5)-visco*k4*x(12)-elasticity*(x(5)))/m2;  
       (a(6)-visco*k4*x(13)-elasticity*(x(6)))/m2;
       (a(7)-visco*k4*x(14)-elasticity*(x(7)))/m2];
   
   
   
   
   
   
    