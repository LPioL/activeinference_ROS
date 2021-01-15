function  MoveArmPR2(x)

%% Initial position of the arm and its command          
pub = rospublisher('/r_arm_controller/command',rostype.trajectory_msgs_JointTrajectory);
message = rosmessage('trajectory_msgs/JointTrajectory');
message.JointNames = {'r_shoulder_pan_joint';'r_shoulder_lift_joint';'r_upper_arm_roll_joint';
    'r_elbow_flex_joint';'r_forearm_roll_joint';'r_wrist_flex_joint';'r_wrist_roll_joint'} ;
point=rosmessage('trajectory_msgs/JointTrajectoryPoint');
message.Points=point;

% Joints angles and velocities
velo = 3*ones(7,1);
acc = 3*ones(7,1);


% Message for ROS
message.Points.Positions = x;
message.Points.Velocities = velo;
message.Points.Accelerations = acc;
send(pub,message)
pause (1)
clear pub message

end

