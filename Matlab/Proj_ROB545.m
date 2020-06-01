% Motion Planning with RRT for a Robot Manipulator
% Plan a grasping motion for a Kinova Jaco Assistive Robotics Arm using the rapidly-exploring random tree (RRT) algorithm. This example uses a plannerRRTStar object to sample states and plan the robot motion. Provided example helpers illustrate how to define custom state spaces and state validation for motion planning applications.
% Load a Kinova Jaco model from the robot library. This particular model includes the three-finger gripper.
clc
clear 
close all

kin = loadrobot('kinovaJacoJ2S7S300');
num_of_env = 6;
num_of_itr_per_combo = 10;
num_of_planner = 2; 
time_to_execute = zeros(num_of_env, num_of_planner);
visualize = 0; % To display motion 1 or else 0
rrt_special = 0; % 0 for normal RRT* that stops when it gets to goal or 1 to contiue optimizing
rrt_itr = 800;
execution_time = zeros(num_of_env, num_of_planner, num_of_itr_per_combo);

for a = 1:1:num_of_env
    env = a;
    for b = 1:1:num_of_planner
        average_time = 0;
        for c = 1:1:num_of_itr_per_combo
            tic;
            fprintf('Number of iterations: %f.\n',c)
            plannerdec = b;
            
            floor = collisionBox(1, 1, 0.01);
            tabletop1 = collisionBox(0.4,1,0.02);
            tabletop1.Pose = trvec2tform([0.3,0,0.6]);
            tabletop2 = collisionBox(0.6,0.2,0.02);
            tabletop2.Pose = trvec2tform([-0.2,0.4,0.5]);
            can = collisionCylinder(0.03,0.16);
            can.Pose = trvec2tform([0.3,0.0,0.7]);
            %worldCollisionArray = {floor tabletop1 tabletop2 can};

            if env == 1
                disp('Environment 1 - 1 can, no wall, no roof')

            elseif env == 2
                disp('Environment 2 - 1 can, side walls, no roof')
                sidewall1 = collisionBox(0.4,0.03,0.3);
                sidewall1.Pose = trvec2tform([0.3,-0.25,0.76]);
                sidewall2 = collisionBox(0.4,0.03,0.3);
                sidewall2.Pose = trvec2tform([0.3,0.25,0.76]);
                %worldCollisionArray = {floor tabletop1 tabletop2 can sidewall1 sidewall2};

            elseif env == 3
                disp('Environment 3 - 1 can, side walls, roof')
                floor = collisionBox(1, 1, 0.01);
                sidewall1 = collisionBox(0.4,0.03,0.3);
                sidewall1.Pose = trvec2tform([0.3,-0.25,0.76]);
                sidewall2 = collisionBox(0.4,0.03,0.3);
                sidewall2.Pose = trvec2tform([0.3,0.25,0.76]);
                roof = collisionBox(0.4,0.53,0.02);
                roof.Pose = trvec2tform([0.3,0,0.92]);
                %worldCollisionArray = {floor tabletop1 tabletop2 can sidewall1 sidewall2 roof};

            elseif env == 4
                disp('Environment 4 - 3 cans, no walls, no roof')
                can2 = collisionCylinder(0.03,0.16);
                can2.Pose = trvec2tform([0.2,-0.15,0.69]);
                can3 = collisionCylinder(0.03,0.16);
                can3.Pose = trvec2tform([0.2,0.15,0.69]);
                %worldCollisionArray = {floor tabletop1 tabletop2 can can2 can3};

            elseif env == 5
                disp('Environment 5 - 3 cans, side walls, no roof')
                can2 = collisionCylinder(0.03,0.16);
                can2.Pose = trvec2tform([0.2,-0.15,0.69]);
                can3 = collisionCylinder(0.03,0.16);
                can3.Pose = trvec2tform([0.2,0.15,0.69]);
                sidewall1 = collisionBox(0.4,0.03,0.3);
                sidewall1.Pose = trvec2tform([0.3,-0.25,0.76]);
                sidewall2 = collisionBox(0.4,0.03,0.3);
                sidewall2.Pose = trvec2tform([0.3,0.25,0.76]);
                %worldCollisionArray = {floor tabletop1 tabletop2 can can2 can3 sidewall1 sidewall2};

            elseif env == 6
                disp('Environment 6 - 3 cans, side walls, roof')
                can2 = collisionCylinder(0.03,0.16);
                can2.Pose = trvec2tform([0.2,-0.15,0.69]);
                can3 = collisionCylinder(0.03,0.16);
                can3.Pose = trvec2tform([0.2,0.15,0.69]);
                sidewall1 = collisionBox(0.4,0.03,0.3);
                sidewall1.Pose = trvec2tform([0.3,-0.25,0.76]);
                sidewall2 = collisionBox(0.4,0.03,0.3);
                sidewall2.Pose = trvec2tform([0.3,0.25,0.76]);
                roof = collisionBox(0.4,0.53,0.02);
                roof.Pose = trvec2tform([0.3,0,0.92]);
                %worldCollisionArray = {floor tabletop1 tabletop2 can can2 can3 sidewall1 sidewall2 roof};
            end

            %exampleHelperVisualizeCollisionEnvironment(worldCollisionArray);
 
% Customize The State Validator
% The customized state validator, ExampleHelperValidatorRigidBodyTree, provides rigid body collision checking between the robot and the environment. This validator checks sampled configurations and the planner should discard invalid states.            
            
            ss = ExampleHelperRigidBodyTreeStateSpace(kin);
            ss.EndEffector = 'j2s7s300_end_effector';

            % Define the workspace goal region (WGR)
            % This WGR tells the planner that the can shall be grasped from
            % the side and the actual grasp height may wiggle at most 1 cm.

            % This is the orientation offset between the end-effector in grasping pose and the can frame
            R = [0 0 1; 1 0 0; 0 1 0];
           
            % Add obstacles in the environment
            Tw_0 = can.Pose;
            Te_w = rotm2tform(R);
            bounds = [0 0;       % x
                     0 0;       % y
                     0 0.01;    % z
                     0 0;       % R
                     0 0;       % P
                    -pi pi];    % Y
            setWorkspaceGoalRegion(ss,Tw_0,Te_w,bounds);
            
            sv = ExampleHelperValidatorRigidBodyTree(ss);

            addFixedObstacle(sv,tabletop1, 'tabletop1', [71 161 214]/256);
            addFixedObstacle(sv,tabletop2, 'tabletop2', [71 161 214]/256);
            addFixedObstacle(sv,can, 'can', 'r');
            addFixedObstacle(sv,floor, 'floor', [1,0.5,0]);
                           
            if env == 2 
                addFixedObstacle(sv,sidewall1, 'sidewall1', 'g');
                addFixedObstacle(sv,sidewall2, 'sidewall2', 'g');
            elseif env == 3
                addFixedObstacle(sv,sidewall1, 'sidewall1', 'g');
                addFixedObstacle(sv,sidewall2, 'sidewall2', 'g');
                addFixedObstacle(sv,roof, 'roof', [1,0.5,0]);
            elseif env == 4
                addFixedObstacle(sv,can2, 'can2', 'r');
                addFixedObstacle(sv,can3, 'can3', 'r');
            elseif env == 5
                addFixedObstacle(sv,can2, 'can2', 'r');
                addFixedObstacle(sv,can3, 'can3', 'r');
                addFixedObstacle(sv,sidewall1, 'sidewall1', 'g');
                addFixedObstacle(sv,sidewall2, 'sidewall2', 'g');
            elseif env == 6
                addFixedObstacle(sv,can2, 'can2', 'r');
                addFixedObstacle(sv,can3, 'can3', 'r');
                addFixedObstacle(sv,sidewall1, 'sidewall1', 'g');
                addFixedObstacle(sv,sidewall2, 'sidewall2', 'g');
                addFixedObstacle(sv,roof, 'roof', [1,0.5,0]);
            end
              
            % Skip collision checking for certain bodies for performance
            skipCollisionCheck(sv,'root'); % root will never touch any obstacles
            skipCollisionCheck(sv,'j2s7s300_link_base'); % base will never touch any obstacles
            skipCollisionCheck(sv,'j2s7s300_end_effector'); % this is a virtual frame
            
            % Set the validation distance
            sv.ValidationDistance = 0.01;

            % Set random seeds for repeatable results
            rng(0,'twister') % 0
            
            % Compute the reference goal configuration. Note this is applicable only when goal bias is larger than 0. 
            Te_0ref = Tw_0*Te_w; % Reference end-effector pose in world coordinates, derived from WGR
            ik = inverseKinematics('RigidBodyTree',kin);
            refGoalConfig = ik(ss.EndEffector,Te_0ref,ones(1,6),homeConfiguration(ss.RigidBodyTree));
            
            % Compute the initial configuration (end-effector is initially under the table)
            T = Te_0ref;
            T(1,4) = 0.3;
            T(2,4) = 0.0;
            T(3,4) = 0.4;
            initConfig = ik(ss.EndEffector,T,ones(1,6),homeConfiguration(ss.RigidBodyTree));
                        
            % Create the planner from previously created state space and state validator
            if plannerdec == 1
                planner = plannerRRT(ss,sv);
                disp('RRT planner')
            elseif plannerdec == 2
                planner = plannerRRTStar(ss,sv);
                planner.ContinueAfterGoalReached = false;
                disp('RRT* planner')
                if rrt_special == 1
                    % Allow further optimization
                    planner.ContinueAfterGoalReached = true;
                    % Reduce max iterations
                    planner.MaxIterations = rrt_itr;
                end
            end
            % If a node in the tree falls in the WGR, a path is considered found.
            planner.GoalReachedFcn = @exampleHelperIsStateInWorkspaceGoalRegion;
            
            % Set the max connection distance.
            planner.MaxConnectionDistance = 0.5;
            
            % With WGR, there is no need to specify a particular goal configuration (use
            % initConfig to hold the place).
            % As a result, one can set GoalBias to zero.
            planner.GoalBias = 0;
            [pthObj,solnInfo] = plan(planner,initConfig, initConfig);

            interpolate(pthObj,100);
            newPathObj = exampleHelperPathSmoothing(pthObj,sv);
            interpolate(newPathObj,200);
     
            %figure
            states = newPathObj.States;
             
            % Draw the robot.
            ax = show(kin,states(1,:));
            zlim(ax, [-0.03, 1.4])
            xlim(ax, [-1, 1])
            ylim(ax, [-1, 1])
            
            % Render the environment.
            hold on
            showObstacles(sv, ax);
            view(146, 33)
            camzoom(1.5)
           
            % Show the motion.
            if visualize == 1
                for i = 2:length(states)
                    show(kin,states(i,:),'PreservePlot',false,'Frames','off','Parent',ax);
                    drawnow
                end
            else
                i = length(states);
            end
            q = states(i,:);
             
            % Grab the can.
            q = exampleHelperEndEffectorGrab(sv,'can',q, ax);
% Set a target psotion for the can on the other tabletop.
            targetPos = [-0.15,0.35,0.51];
            exampleHelperDrawHorizontalCircle(targetPos,0.02,'y',ax);

% Plan The Move Motion
% During the move motion, keep the cylinder can level at all time to avoid spill. Specify an additional constraint on the interim manipulator configurations for the RRT planner. Turn the constraint on by setting the UseConstrainedSampling property to true.
            Tw_0 = trvec2tform(targetPos+[0,0,0.08]); 
            Te_w = rotm2tform(R);
            bounds =  [0 0;       % x
                       0 0;       % y
                       0 0;       % z
                       0 0;       % R
                       0 0;       % P
                      -pi pi];    % Y
            setWorkspaceGoalRegion(ss,Tw_0,Te_w,bounds);
            ss.UseConstrainedSampling = true;
            planner.MaxConnectionDistance = 0.05;
            [pthObj2, solnInfo2] = plan(planner,q,q);
            interpolate(pthObj2,100);
            newPathObj2 = exampleHelperPathSmoothing(pthObj2,sv);
            interpolate(newPathObj2,200);
            states = newPathObj2.States;
             
            view(ax, 152,45)
            if visualize == 1
                for i = 2:length(states)
                    show(kin,states(i,:),'PreservePlot',false,'Frames','off','Parent',ax);
                    drawnow
                end
            else
                i = length(states);
            end
            q = states(i,:);
             
            % let go of the can
            q = exampleHelperEndEffectorRelease(sv,q,ax);
            time_to_complete_single_itr = toc
            average_time = average_time + time_to_complete_single_itr;
            execution_time(a,b,c) = time_to_complete_single_itr;
            close all
            file_name = sprintf('Test_4_InitObj env%d planner%d itr%d.csv' , a, b, c);
            file_name2 = sprintf('Test_4_ObjGoal env%d planner%d itr%d.csv' , a, b, c);
            writematrix(newPathObj.States, file_name)
            writematrix(newPathObj2.States, file_name2)
            save(['Test_4_solinfo_env_' num2str(a) '_planner_' num2str(b) '_itr_' num2str(c) '.mat'],'solnInfo')
            save(['Test_4_solinfo2_env_' num2str(a) '_planner_' num2str(b) '_itr_' num2str(c) '.mat'],'solnInfo2')
        end
        time_to_execute(a, b) = average_time/num_of_itr_per_combo;
    end
end
save('Test_4_env-1-6_planner-1-2_avg-10_without-display.mat', 'time_to_execute')
save('Test_4_individual-execution-time_env-1-6_planner-1_without-display.mat', 'execution_time') 

% References
% [1] D. Berenson, S. Srinivasa, D. Ferguson, A. Collet, and J. Kuffner, "Manipulation Planning with Workspace Goal Regions", in Proceedings of IEEE International Conference on Robotics and Automation, 2009, pp.1397-1403
% [2] D. Berenson, S. Srinivasa, and J. Kuffner, "Task Space Regions: A Framework for Pose-Constrained Manipulation Planning", International Journal of Robotics Research, Vol. 30, No. 12 (2011): 1435-1460
% [3] P. Chen, and Y. Hwang, "SANDROS: A Dynamic Graph Search Algorithm for Motion Planning", IEEE Transaction on Robotics and Automation, Vol. 14 No. 3 (1998): 390-403
% 
% Copyright 2019-2020 The MathWorks, Inc.


