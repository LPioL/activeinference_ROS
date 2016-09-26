function [g]= SevDofthreeDv1simpleROS_spm_gx_robot_dem_reach(x,v,P)
% returns the prediction for a two-joint arm
% FORMAT [g]= spm_gx_dem_reach(x,v,P)
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


% evaluate positions


% stretch (angular) and visual (positional) information (target & arm)
%==========================================================================

x = full(x);

%% Get_fk method for visual input of the position of the end effector
try
    gazebo = ExampleHelperGazeboCommunicator();
    models = getSpawnedModels(gazebo);
    pr2 = ExampleHelperGazeboSpawnedModel('pr2',gazebo);
    testclient = rossvcclient('/pr2_right_arm_kinematics/get_fk');
    

catch
    try
    rosshutdown
    pause(2)
    
    rosinit
    pause(2)
    gazebo = ExampleHelperGazeboCommunicator();
    pause(2)
    
    models = getSpawnedModels(gazebo);
    pr2 = ExampleHelperGazeboSpawnedModel('pr2',gazebo);
    testclient = rossvcclient('/pr2_right_arm_kinematics/get_fk');
    catch
    rosshutdown
    pause(2)
    
    rosinit
    pause(2)
    gazebo = ExampleHelperGazeboCommunicator();
    pause(2)
    
    models = getSpawnedModels(gazebo);
    pr2 = ExampleHelperGazeboSpawnedModel('pr2',gazebo);
    testclient = rossvcclient('/pr2_right_arm_kinematics/get_fk');
    end
end
[pr2Links, pr2Joints] = getComponents(pr2);

testreq = rosmessage(testclient);
testreq.FkLinkNames = {'r_wrist_roll_link'};
testreq.Header.FrameId = 'base_footprint';
testreq.RobotState.JointState.Name = pr2Joints(30:36);

% For broken articulation
%x(4)=-0.4;

%vel = x(8:14);

testreq.RobotState.JointState.Position=x(1:7);
%testreq.RobotState.JointState.Velocity =vel;
sumresp = call(testclient,testreq,'Timeout',200);
X=sumresp.PoseStamped.Pose.Position.X;
Y=sumresp.PoseStamped.Pose.Position.Y;
Z=sumresp.PoseStamped.Pose.Position.Z;

Pose = [X;Y;Z];%+rand;% + [0.1;0;0.1];

clear gazebo testclient


%% Mapping
g  = [ x(1:7); v; [Pose(1);Pose(2);Pose(3)]];




