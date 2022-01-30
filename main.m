close all
addpath('mex\')
addpath('yumiMatlab\')
addpath('utilities\')
addpath('PSMIOC\')
robot = Robot('yumi');


dataDirs = Utils.getAllFileNames('data','contain','.');
for i = 1:6%length(dataDirs) 7 is broken
    Qfiles = Utils.getAllFileNames(dataDirs{i},'contains','.tsv');
    for f = 1:length(Qfiles)
        processExercise(Qfiles{f}, robot);
    end
end


% dataDirs = Utils.getAllFileNames('data','contain','.');
% for i = 1:1%length(dataDirs) 7 is broken
%     files = Utils.getAllFileNames(dataDirs{i},'contains','.tsv');
%     for f = 1:length(files)
%         processExercise(files{f}, robot);
%     end
% end

%
% for i = 1:length(dataDirs)
%     Utils.removeFilesPattern([dataDirs{i} '/'], '*.mat')
% end

%%
close all
robot = Robot('yumi');
dataDirs = Utils.getAllFileNames('data','exclude','.');

for i = 1:length(dataDirs)
    Qfiles = Utils.getAllFileNames(dataDirs{i}, 'contains', 'Q', 'exclude','inds');
    for f = 1:length(Qfiles)
        load(Qfiles{f});
        complete = robot.trajectoryBuilder(Q, Qfiles{f});
        if ~complete
            keyboard
        end
    end
end

%% build demos
dataDirs = Utils.getAllFileNames('data','exclude','.');
exercises = {'forwardRaise', 'lateralRaise', 'shoulderPress', 'internalExternalRotation'};
sides = {'L0','R0'};
close all
Utils.removeDir('demonstrations')
Utils.makeDirIfMissing('demonstrations')
for k = 1:2
    side = sides{k};
    for i = 1:length(dataDirs)
        Qfiles = Utils.getAllFileNames(dataDirs{i}, 'filter', @(x) ~(contains(x, 'Q') && contains(x, side)), 'exclude','inds');
        Xfiles = Utils.getAllFileNames(dataDirs{i}, 'filter', @(x) ~(contains(x, 'X') && contains(x, side)), 'exclude','inds');
        for val = 1:length(exercises)
            figure(val + k*length(exercises))
            indFiles = Utils.getAllFileNames(dataDirs{i}, 'filter',@(x) ~(contains(x, exercises{val}) && contains(x, 'inds') && contains(x, side)) );

            for f = 1:length(Qfiles)
                tmp = load(Qfiles{f});
                Q = tmp.Q;
                tmp = load(Xfiles{f});
                X = tmp.X;
                complete = robot.trajectoryBuilder(Q, Qfiles{f});
                if complete
                    tmp = load(indFiles{f});
                    inds = tmp.inds;
                    plot(X(inds, :)); hold on
                    D = X(inds, :);
                    demoFiles = Utils.getAllFileNames('demonstrations/', 'filter',@(x) ~(contains(x, exercises{val}) &&  contains(x, side) ) );
                    demoName = ['demonstrations/' exercises{val} '_' side '_' num2str(length(demoFiles))]; 
                    save(demoName, 'D')
                    title([exercises{val} '_' side(1)])
%                 else
%                     break
                end
            end
%             if ~complete
%                 break
%             end
        end
    end
end

%% test
% addpath('mex\')
% robot = Robot('yumi');
% jointAngles = robot.getJoints();
% robot.setJoints(jointAngles*0+0);
% J = robot.getJacobian()
% T = robot.getTransform(5)
%
% jointVel = jointAngles*0;
% while 1
%     jointAngles = robot.getJoints();
%     jointVel = jointVel +  .1*(rand(length(jointAngles),1) -.5);
%     jointAngles = jointAngles + jointVel*.1;
% robot.setJoints(jointAngles );
% robot.plotObject()
% pause(0.00001)
%end
%% controller

system('yumi_ws\runSim.bat &')
rosshutdown()
rosinit()
jointSub = rossubscriber('/joint_states','DataFormat','struct');
jointVelRPub = rospublisher('/joint_velocity_command_R','DataFormat','struct');
jointVelLPub = rospublisher('/joint_velocity_command_L','DataFormat','struct');
msg = rosmessage('std_msgs/Float32MultiArray','DataFormat','struct');
%%
robot = Robot('yumi');
dt = 1/30;
for i = 1:100
    jointSub.LatestMessage.Position'
    robot.setJoints(jointSub.LatestMessage.Position);
    [JeR, JwR, JeL, JwL] = robot.getElbowWristJacobians();
    elbowTransR = robot.getTransform(5);
    vec = .1*([1; 0; .5] - elbowTransR(1:3,end));
    vel = pinv(JeR)*vec;
    msg.Data = single(vel(1:end/2));
    jointVelRPub.send(msg);
    msg.Data = single(vel(end/2+1:end));
    jointVelLPub.send(msg);
    %     Q = robot.getJoints();
    %     Q = Q + vel*dt;


    robot.plotObject()
    pause(dt)
end
msg.Data = single(vel(end/2+1:end)*0);
jointVelRPub.send(msg);
jointVelLPub.send(msg);

%% build

% mex -g D:\NSFExercise\cpp\mexFiles\process_txt.cpp