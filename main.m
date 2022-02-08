%% convert all .tsv files to joint and point trajectories Q, X in robot correspondance
if false
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

if false
load("data\Subject 11\Subject 11 Order2R0001Q.mat")
load("data\Subject 11\Subject 11 Order2R0001Q_forwardRaise_inds.mat")
Q = Q(inds,:)
robot.trajectoryBuilder(Q,'vs')

end

%% manually parse beginning and end for each exercise
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

%% build demos: populate demonstrations folder
dataDirs = Utils.getAllFileNames('data','exclude','.');
exercises = {'forwardRaise', 'lateralRaise', 'shoulderPress', 'internalExternalRotation'};
phases = {'up', 'down'};
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
                    title([exercises{val} '\_' side(1)])
                end
            end
        end
    end
end

%% controller

% system('yumi_ws\runSim.bat &')
% pause(5)
rosshutdown()
rosinit()
jointSub = rossubscriber('/joint_states','DataFormat','struct');
jointVelRPub = rospublisher('/joint_velocity_command_R','DataFormat','struct');
jointVelLPub = rospublisher('/joint_velocity_command_L','DataFormat','struct');
msg = rosmessage('std_msgs/Float32MultiArray','DataFormat','struct');
end
%% go to start
addpath('mex\')
addpath('yumiMatlab\')
addpath('utilities\')
addpath('PSMIOC\')
robot = Robot('yumi');
timeScale = 1/1;
phases = {'up', 'down'};
exercises = {'forwardRaise', 'lateralRaise', 'shoulderPress', 'internalExternalRotation'};
side = 'R0';
jointInds = [1:7 9:15];
for e = 3
        phase = phases{1};
        robot = Robot('yumi');
        psm = PSM.load(['PSMIOC/models/psm_'  exercises{e} '_' phase '_' side]);

        dt = 1/20;
        x = getX(robot);
        while norm(x-psm.nominalStart) > .1
            tic
            %     jointSub.LatestMessage.Position'
            robot.setJoints(jointSub.receive().Position(jointInds));
            x = getX(robot);
            vec = -sign(x-psm.nominalStart).*(2*abs(x-psm.nominalStart));
            %     vec = vec.*(abs(x-psm.nominalStart) > .05);
            Jall = robot.getAllJacobians();
            J = cat(1, Jall{5}, Jall{7});
            %     vel = .3*pinv(J,.1)*vec;
            A = cat(1, J, (.01+.5*norm(x-psm.nominalStart))*eye(size(J,2)));
            b = cat(1, vec, (.01+.5*norm(x-psm.nominalStart))*(robot.home - robot.getJoints()) );
            vel =  lsqlin(A,b);
            vel = min(max(vel, -.2), .2);
            msg.Data = single(vel(1:end/2));
            jointVelRPub.send(msg);
            msg.Data = single(vel(end/2+1:end));
            jointVelLPub.send(msg);
            %     Q = robot.getJoints();
            %     Q = Q + vel*dt;


            %     robot.plotObject()
            pause(toc -dt)
        end
        msg.Data = single(vel(end/2+1:end)*0);
        jointVelRPub.send(msg);
        jointVelLPub.send(msg);
        %% PSM
        robot.setJoints(jointSub.receive().Position(jointInds));
        x = getX(robot);
 close all
 qCell = cell(2,1);
 qdCell = cell(2,1);
    for p = 1:length(phases)
        phase = phases{p};
       
        psm = PSM.load(['PSMIOC/models/psm_'  exercises{e} '_' phase '_' side]);

        psm.q = x;
        psm.qd = x*0;

        Qi = arrayfun(@(a) a, x,'UniformOutput',false);
        Qf = cell(length(x), 1);
        Qdi = arrayfun(@(a) 0*a, x, 'UniformOutput',false);
        Qdf = arrayfun(@(a) 0*a, x, 'UniformOutput',false);
        W = psm.W;
        psm.setupBasisFncs(psm.basisConfig, Qi, Qf)
        psm.W = W;
        boundaryConfig = struct( ...
            'startPos', true,...
            'endPos', false,...
            'startDir', false,...
            'endDir', false,...
            'startKE', true, ...
            'endKE', true);
        psm.setupBoundaryConds(boundaryConfig, Qi, Qdi, Qf, Qdf);

        psm.buildOptimize();
        psm.optimizePSM();

        invV = inv(psm.V);
        q = [];
        qd = [];
        while invV(1,:)*psm.q < psm.cp(end)
            psm.step(dt*timeScale)
            q = [q;psm.q'];
            qd = [qd;psm.qd'];
        end
        figure
        plot(q)
        qCell{p} = q;
        qdCell{p} = qd*timeScale;
        x = q(end,:)';
    end
        %% track trajectory
    for p = 1:length(phases)
        phase = phases{p};
        q = qCell{p};
        qd = qdCell{p};
        for i = 1:size(q,1)
            tic
            %     jointSub.LatestMessage.Position'
            robot.setJoints(jointSub.receive().Position(jointInds));
            x = getX(robot);

            vec = qd(i,:)' - 1*(x-q(i,:)');
            Jall = robot.getAllJacobians();
            J = cat(1, Jall{5}, Jall{7});
            %     vel = .3*pinv(J,.1)*vec;
            A = cat(1, J, .05*eye(size(J,2)));
            b = cat(1, vec, .05*(robot.home - robot.getJoints()) );
            vel =  lsqlin(A,b);
            
            Q = robot.getJoints();
            bool = vel > 0 & Q >= robot.jointMaximums-.1;
            vel(bool) = vel(bool).*(robot.jointMaximums(bool) - Q(bool))/.1;
            bool = vel < 0 & Q <= robot.jointMinimums+.1;
            vel(bool) = vel(bool).*(Q(bool) - robot.jointMaximums(bool))/.1;
 
            msg.Data = single(vel(1:end/2));
            jointVelRPub.send(msg);
%             msg.Data = single(vel(end/2+1:end));
%             jointVelLPub.send(msg);
            %     Q = robot.getJoints();
            %     Q = Q + vel*dt;
            %     robot.plotObject()
             dt - toc
            pause(dt - toc)
           
        end

        for i =1:10
            msg.Data = single(vel(end/2+1:end)*0);
            jointVelRPub.send(msg);
            jointVelLPub.send(msg);
        end
    end
end
%% test

robot = Robot('yumi');
J1 = robot.getJacobian();
Jall = robot.getAllJacobians();
J1(1:3,:)
Jall{10}

J1(end-5:end-3,:)
Jall{end}

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

%% build cpp

% mex -g D:\NSFExercise\cpp\mexFiles\process_txt.cpp