% rosshutdown()
% % rosinit('192.168.1.5')
% % rosinit('10.22.0.237');
% rosinit('10.22.0.158');
% client = rossvcclient('/getTransformations');
% clientPos = rossvcclient('/setQ');
% msg = rosmessage(client);
% msgPos = rosmessage(clientPos);
function Q = algorithm1(DeR, DwR, DeL, DwL, robot)
% close all
% addpath('mex\')
% robot = Robot('yumi');

stepSize = .1;
R = cell(14, 1);
P = cell(14, 1);
lambda1 = 0.001;
% lambda2 = 0.01;

z  = [0 0 1]';
it = 0;

beR = 1; bwR =1; beL = 1; bwL =1;
% rest = robot.home;
% rest(5:7) = 0;
% rest(14:16) = 0;
% jointInds = [1:7 10:16];
jointInds = 1:14;

while norm(beR)+norm(bwR)+norm(beL)+norm(bwL) > 0.005 && it < 5000
    it = it + 1;

    Q = robot.getJoints(); % ensures joint limits are enforced
    robot.setJoints(Q)

    for i = 1:7
        % right
        T = robot.getTransform(i+1);
        R{i} = T(1:3, 1:3);
        P{i} = T(1:3, end);
        % left
        T = robot.getTransform(i+1+9);
        R{i+7} = T(1:3, 1:3);
        P{i+7} = T(1:3, end);
    end

    JeR = zeros(3, 7);
    for j = 1:2
        Rj = R{j};
        JeR(:,j) = cross(Rj(:,3),P{3}-P{j});
    end
    JeL = zeros(3, 7);
    for j = 1:2
        Rj = R{j+7};
        JeL(:,j) = cross(Rj(:,3), P{3+7}-P{j+7});
    end
    
    beR = (DeR./norm(DeR)-R{3}*z);
    beL = (DeL./norm(DeL)-R{3+7}*z);

    JwR = zeros(3, 7);
    for j = 1:4
        Rj = R{j};
        JwR(:,j) = cross(Rj(:,3),P{5}-P{j});
    end

    JwL = zeros(3, 7);
    for j = 1:4
        Rj = R{j+7};
        JwL(:,j) = cross(Rj(:,3),P{5+7}-P{j+7});
    end
    
    bwR = DwR./norm(DwR)-R{5}*z;
    bwL = DwL./norm(DwL)-R{5+7}*z;

%     val = -sign(Q-rest);
% .5*(norm(beR)+ norm(bwR))*lambda2*val(1:7)
%         .5*(norm(be)+ norm(bw))*lambda2*eye(7) ...


    A = cat(1, ...
        [JeR zeros(size(JeR))], ...
        .05*[JwR zeros(size(JwR))], ...
        [zeros(size(JeL)) JeL], ...
        .05*[zeros(size(JwL)) JwL], ...
        lambda1*eye(14)...
        );
    b = cat(1, ...
        beR, ...
        .05*bwR, ...
        beL, ...
        .05*bwL, ...
        lambda1*zeros(14,1)...
        );

    deltaJoint = stepSize*lsqr(A, b);
    Q(jointInds) = Q(jointInds)+ deltaJoint;
    robot.setJoints(Q);
    norm(beR)+norm(bwR)+norm(beL)+norm(bwL)

end

    if rand(1) > .99
        robot.plotObject()
        pause(0.001)
    end


end