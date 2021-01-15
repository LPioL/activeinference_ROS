function [f]= ROS_spm_fx_robot_dem_reach(x,v,P)
% returns the flow for a two-joint arm
% FORMAT [f]= spm_fx_dem_reach(x,v,P)
%
% x    - hidden states
%   x(1) - joint angle
%   x(2) - joint angle
%   x(3) - angular velocity
%   x(4) - angular velocity
% v    - causal states
%   v(1) - target location (x)
%   v(2) - target location (y)
%   v(3) - cue strength
% P    - parameters
%__________________________________________________________________________



x = full(x);

%% TARGET
Target=[v(1);v(2); v(3)];


%% Computation of the positions of the joints
LinkNames={'r_shoulder_pan_link';'r_shoulder_lift_link';'r_upper_arm_roll_link';
    'r_elbow_flex_link';'r_forearm_roll_link';'r_wrist_flex_link';'r_wrist_roll_link'};

try
    gazebo = ExampleHelperGazeboCommunicator();
    models = getSpawnedModels(gazebo);
    pr2 = ExampleHelperGazeboSpawnedModel('pr2',gazebo);
    testclient = rossvcclient('/pr2_right_arm_kinematics/get_fk');

catch
    rosshutdown
    rosinit
    pause(2)
    gazebo = ExampleHelperGazeboCommunicator();
    models = getSpawnedModels(gazebo);
    pr2 = ExampleHelperGazeboSpawnedModel('pr2',gazebo);
    testclient = rossvcclient('/pr2_right_arm_kinematics/get_fk');

end
[pr2Links, pr2Joints] = getComponents(pr2);
testreq = rosmessage(testclient);
testreq.Header.FrameId = 'base_footprint';
testreq.RobotState.JointState.Name = pr2Joints(30:36);


testreq.RobotState.JointState.Position=x(1:7);

for i = 1: size(LinkNames,1)

    
    testreq.FkLinkNames = LinkNames(i);
    
    sumresp = call(testclient,testreq,'Timeout',100);
    
    X=sumresp.PoseStamped.Pose.Position.X;
    Y=sumresp.PoseStamped.Pose.Position.Y;
    Z=sumresp.PoseStamped.Pose.Position.Z;
    ALL(:,i) = [X;Y;Z];
    X1=sumresp.PoseStamped.Pose.Orientation.X;
    Y1=sumresp.PoseStamped.Pose.Orientation.Y;
    Z1=sumresp.PoseStamped.Pose.Orientation.Z;
    W=sumresp.PoseStamped.Pose.Orientation.W;
    RotMat(:,:,i) = quat2rotm([X1;Y1;Z1;W]');
end


% A bit of noise in order to avoid the createUnitVector problem
for i = size(ALL,2):-1:1
   ALL(:,i) = ALL(:,i)+ 0.0000000001*rand;
end

PoseEndEffector = ALL(:,end);


% Computation of the error and then the needed accelerations on order to reach the target
k=2.5;
k2= 1/750;
acc = zeros(size(ALL,2),1);
vec_phy = ( Target - PoseEndEffector);



for i = size(ALL,2)-1:-1:1
    
    PoseLink_i = ALL(1:3,i);
    
    if i == size(ALL,2)
        PoseLink_iplus1 = PoseEndEffector;
    else
        PoseLink_iplus1 = ALL(1:3,i+1);
    end
    
    % la normale au plan passant par trois points : la cible et les deux points extrêmes du segment i du bras
    normale_plan_3pts = createUnitVector(cross(PoseLink_iplus1 - PoseLink_i, PoseLink_iplus1 - Target) );
    
    % la normale du plan perpendiculaire au segment i
    vec_segment_i = createUnitVector(PoseLink_iplus1 - PoseLink_i);
    
    % vecteur unitaire perpendiculaire au segment i et appartenant au plan_3pts

    vec_ortho =RotMat(:,:,i) * [0;0;-1];
    
    % part de vec_phy qui doit être corrigée par l'articulation i
    acc(i) =  k*dot(vec_phy,vec_ortho);
    pos(i) =  k2*dot(vec_phy,vec_ortho);
    
    
end


visco = 1;
m1=4;
m2=3;

k3=4;
k4 = 3.21;

%% Flow as an elastic
%==========================================================================

  f  = [x(8);     
        x(9);
        x(10);
        x(11);
        x(12);
        x(13);
        x(14);
       (acc(1)-visco*k3*x(8))/m1;     
       (acc(2)-visco*k3*x(9))/m1;
       (acc(3)-visco*k3*x(10))/m1;
       (acc(4)-visco*k4*x(11))/m2;
       (acc(5)-visco*k4*x(12))/m2;  
       (acc(6)-visco*k4*x(13))/m2;
       (acc(7)-visco*k4*x(14))/m2;
       ];
















