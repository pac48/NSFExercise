function processExercise(fileName, robot)
tmp  = process_txt(fileName);
names = cellfun(@(x) x.name, tmp, 'UniformOutput', false);
data = cellfun(@(s) [[s.x]' [s.y]' [s.z]'], tmp, 'UniformOutput', false);
tmp = cat(1, names', data');
tmp = reshape(tmp, 1,[]);
dataStruct = struct(tmp{:});
robot.setJoints(robot.home);

if ~isempty(tmp)

    % rotate the frame so the hips are in the postive x direction
    xT = (dataStruct.L_IAS(1,:)+dataStruct.R_IAS(1,:))/2-(dataStruct.L_IPS(1,:)+dataStruct.R_IPS(1,:))/2;
    xT(3)=0;
    xT=xT./norm(xT);
    theta = acos(dot(xT,[1 0 0]'));
    theta = theta*sign(sum(cross([1 0 0], xT)));
    f = fields(dataStruct);
    for i =3:length(f)
        data = dataStruct.(f{i});
        R = [cos(theta) -sin(theta) 0;sin(theta) cos(theta) 0; 0 0 1];
        dataStruct.(f{i}) = (R*data')';
    end

    % get vectors for left and right wrist
    % right markers
    wristR = (dataStruct.R_RSP+dataStruct.R_USP)/2-dataStruct.R_HLE;
    for i =1:size(wristR,1)
        wristR(i,:)=wristR(i,:)./norm(wristR(i,:));
    end
    elbowR = dataStruct.R_HLE-dataStruct.R_SAE;
    for i =1:size(elbowR,1)
        elbowR(i,:)=elbowR(i,:)./norm(elbowR(i,:));
    end
    % left markers
    wristL = (dataStruct.L_RSP+dataStruct.L_USP)/2-dataStruct.L_HLE;
    for i =1:size(wristL,1)
        wristL(i,:) = wristL(i,:)./norm(wristL(i,:));
    end
    elbowL = dataStruct.L_HLE-dataStruct.L_SAE;
    for i =1:size(elbowL,1)
        elbowL(i,:) = elbowL(i,:)./norm(elbowL(i,:));
    end

    Q = zeros(size(elbowR, 1), robot.numJoints);
    X = zeros(size(elbowR, 1), 3*robot.numBodies);
    for i =1:size(elbowR,1)
         Q(i, :) = algorithm1(elbowR(i,:)', wristR(i,:)', elbowL(i,:)', wristL(i,:)', robot);
         for j = 1:robot.numBodies
             Tj = robot.getTransform(j);
             X(i, (1:3)+(j-1)*3) = Tj(1:3,end);
         end
    end
   

%     wr = (dataStruct.R_RSP+dataStruct.R_USP)/2 - dataStruct.R_SAE;
% scatter3(wr(:,1), wr(:,2), wr(:,3))
% hold on
% eb = dataStruct.R_HLE-dataStruct.R_SAE;
% scatter3(eb(:,1), eb(:,2), eb(:,3))

    fileName = fileName(1:find(fileName == '.',99)-1);
    save([fileName 'X.mat'], 'X')
    save([fileName 'Q.mat'], 'Q')

end

end