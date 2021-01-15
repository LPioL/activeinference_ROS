% ROS and Friston reaching
% Leo Lopez
close all
clear all

% PR2 robot (7 DoFs on one arm)
for trial = 1:5
    


rosshutdown;
rosinit;


%--------------------------------------------------------------------------


%% Initial position of the arm and its command          

posinit=[0;0.75;0;-0.75;0;0;0];
MoveArmPR2(posinit);

%% Computation of the target into 3D space
postarget=-0.25*ones(7,1);
T = PosEndEffectorPR2(postarget);% + [0.1;0;0.1];
 


% Recognition model (linear for expediency)
%==========================================================================
M         = struct;
M(1).E.s  = 1/2;                              % smoothness
M(1).E.n  = 4;                                % order of 
M(1).E.d  = 2;                                % generalised motion
 
% level 1: Displacement dynamics and mapping to sensory/proprioception
%--------------------------------------------------------------------------
M(1).f  = 'ROS_spm_fx_robot_dem_reach_v2';                 % plant dynamics
M(1).g  = 'ROS_spm_gx_robot_dem_reach';                 % prediction
 
M(1).x  = [posinit; 0; 0;0;0;0;0;0];
M(1).V  = exp(8);                             % error precision
M(1).W  = exp(8);                             % error precision
 
% level 2: with non-informative priors on movement
%--------------------------------------------------------------------------
M(2).v  = [0; 0;0];                          % inputs
M(2).V  = exp(0);
 
% generative model
%==========================================================================
G       = M;
 
% first level
%--------------------------------------------------------------------------
G(1).f  = 'ROS_spm_fx_robot_adem_reach';
G(1).g  = 'ROS_spm_gx_robot_adem_reach';
G(1).V  = exp(16);                            % error precision
G(1).W  = exp(16);                            % error precision
G(1).U  = exp(8);                            % gain for action
 
% second level
%--------------------------------------------------------------------------
G(2).v  = [0; 0;0];                          % inputs
G(2).a  = [0; 0;0;0;0;0;0];              % action
G(2).V  = exp(16);
 
 
% generate and invert
%==========================================================================
N       = 40;                                 % length of data sequence
C       = sparse(3,N);
C(1,:)  = C(1,:) + T(1);                        % desired x
C(2,:)  = C(2,:) + T(2);                        % desired y
C(3,:)  = C(3,:) + T(3);                        % cue strength

M(2).v  = C(:,1);
 
 
DEM.G   = G;
DEM.M   = M;
DEM.C   = C;
DEM.U   = sparse(3,N);

% % Test nosiy information
% enable precision optimization
%--------------------------------------------------------------------------
 
Q{1}       = diag([1 1 1 1 1 1 1 0 0 0 0 0 0]);
Q{2}       = diag([0 0 0 0 0 0 0 0 0 0 1 1 1]);
M(1).E.nE  = 4;
M(1).E.nM  = 8;
M(1).V     = diag(exp([0 0 0 0 0 0 0 8 8 8 0 0 0]));               % error precision
M(1).Q     = Q;   

% Vision noise
G(1).V     = diag(exp([8 8 8 8 8 8 8 16 16 16 4 4 4]));            % error precision
M(1).hE    = [8 4];
DEM.G = G;
DEM.M = M;
DEM   = ROS_spm_ADEM(DEM, postarget);


% overlay true values
%--------------------------------------------------------------------------
% Graphics
%==========================================================================
spm_figure('GetWin','Graphics');
clf
 
subplot(2,1,1)
spm_dem_reach_plot(DEM)
title('trajectory','FontSize',16)
 
subplot(2,1,2)
spm_dem_reach_movie(DEM)
title('click on finger for movie','FontSize',16)


spm_DEM_qU(DEM.qU,DEM.pU)
 
 
% Graphics
%==========================================================================
spm_figure('GetWin','Figure 1');
clf
 
subplot(2,1,1)
spm_dem_reach_plot(DEM)
title('trajectory','FontSize',16)
 

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
    
 
 
 
commands = full(DEM.pU.x{1});
for i =1:size (commands,2)
    
    % In order to show the trajectory in a 'smooth' way
    pub = rospublisher('/r_arm_controller/command',rostype.trajectory_msgs_JointTrajectory);
    message = rosmessage('trajectory_msgs/JointTrajectory');
    message.JointNames = {'r_shoulder_pan_joint';'r_shoulder_lift_joint';'r_upper_arm_roll_joint';
        'r_elbow_flex_joint';'r_forearm_roll_joint';'r_wrist_flex_joint';'r_wrist_roll_joint'} ;
    point=rosmessage('trajectory_msgs/JointTrajectoryPoint');
    message.Points=point;
    %acc = [0.5;0.5;0;0;0;0;0];
    message.Points.Positions = commands(1:7,i);
    message.Points.Velocities = commands(8:14,i);
    send(pub,message)
    pause (1)
    clear pub message
   
    % Get_fk method
    [pr2Links, pr2Joints] = getComponents(pr2);
    testreq = rosmessage(testclient);
    testreq.Header.FrameId = 'base_footprint';
    testreq.RobotState.JointState.Name = pr2Joints(30:36);
    %testreq.RobotState.MultiDOFJointState.JointNames = cell(pr2Joints(30:36));
    
    pos=commands(1:7,i);
    
    testreq.RobotState.JointState.Position=pos;
    testreq = rosmessage(testclient);
    testreq.FkLinkNames = {'r_wrist_roll_link'};
    testreq.Header.FrameId = 'base_footprint';
    testreq.RobotState.JointState.Name = pr2Joints(30:36);
    

    testreq.RobotState.JointState.Position=commands(1:7,i);
    sumresp = call(testclient,testreq,'Timeout',200);
    X=sumresp.PoseStamped.Pose.Position.X;
    Y=sumresp.PoseStamped.Pose.Position.Y;
    Z=sumresp.PoseStamped.Pose.Position.Z;

PoseEndEffector(:,i) = [X;Y;Z];
    
        

end
NB=19;
figure(5)
    plot3(PoseEndEffector(1,:), PoseEndEffector(2,:), PoseEndEffector(3,:),'-r')
hold on

plot3(T(1),T(2),T(3),'O','MarkerSize',20,'MarkerFaceColor','g','MarkerEdgeColor','g');
save (strcat('proprio',int2str(trial+NB)),'PoseEndEffector')
save (strcat('proprioDEM',int2str(trial+NB)),'DEM')
results = figure (3);
spm_DEM_qU(DEM.qU,DEM.pU)
saveas(results,strcat('proprioResults',int2str(trial+NB),'.png'))
close all
clear DEM

end
return
 
 
 
