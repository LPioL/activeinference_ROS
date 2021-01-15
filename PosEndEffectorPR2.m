function T = PosEndEffectorPR2( posTarget )

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

% POSTARGET
vel = zeros(7,1);


testreq.RobotState.JointState.Position=posTarget;
testreq.RobotState.JointState.Velocity =vel;
sumresp = call(testclient,testreq,'Timeout',200);
X=sumresp.PoseStamped.Pose.Position.X;
Y=sumresp.PoseStamped.Pose.Position.Y;
Z=sumresp.PoseStamped.Pose.Position.Z;

T = [X;Y;Z];


end

