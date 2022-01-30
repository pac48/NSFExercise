classdef Robot < handle

    properties(Constant, Access=private)
        INIT = 0
        DESTROY = 1
        SETJOINTS = 2
        GETJOINTS = 3
        GETJACOB = 4;
        GETTRANS = 5
        GETOPPOS = 6;
    end

    properties(Access=private)
        robotPtr
        verts
        vertsNormals
        vertsNames
        body2VertMap
        startInd
        directory
        side
        t
        hSpinner
        hSlider
        fig
    end

    properties
        numJoints
        numBodies
        jointNames
        bodyNames
        numEndEffector
        jointMinimums
        jointMaximums
        home = [(46.0 / 180.0) * 3.14;
            (-121.0 / 180.0) * 3.14;
            (-87.0 / 180.0) * 3.14;
            (4.0 / 180.0) * 3.14;
            (44.0 / 180.0) * 3.14*0;
            (54.0 / 180.0) * 3.14*0;
            (-3.0 / 180.0) * 3.14*0;
            -(46.0 / 180.0) * 3.14;
            (-121.0 / 180.0) * 3.14;
            -(-87.0 / 180.0) * 3.14;
            (4.0 / 180.0) * 3.14;
            -(44.0 / 180.0) * 3.14*0;
            (54.0 / 180.0) * 3.14*0;
            (-3.0 / 180.0) * 3.14*0;
            ];
    end

    methods
        function obj = Robot(name)
            assert(strcmp(name, 'yumi'), 'name must be a char array of a supported robot. Supported robots: yumi')
            p = which('Robot.m');
            directory = p(1:max(find(p=='\',9999)));  
            [obj.robotPtr, obj.numJoints, obj.numBodies, obj.jointNames,...
                obj.bodyNames,obj.numEndEffector, obj.jointMinimums, obj.jointMaximums] ...
                = robot_mex(obj.INIT, [directory name]);
            obj.jointMinimums(4) = obj.jointMinimums(4) +.7;
            obj.jointMinimums(3) = obj.jointMinimums(3);

            if exist([directory 'obj.mat']) == 2
                tmp = load([directory 'obj.mat']);
                obj.verts = tmp.verts;
                obj.vertsNames = tmp.vertsNames;
                obj.vertsNormals = tmp.vertsNormals;
            else
                [points, verts, vertsNormals, verticesTextureCoordinates, vertsNames] = object_loader([directory name '.obj']);
                vertsNames = cellfun(@(x) x(find(x =='_',1)+1:end), vertsNames,'UniformOutput',false);
                save([directory 'obj'], 'verts', 'vertsNames','vertsNormals');
                obj.verts = verts;
                obj.vertsNormals = vertsNormals;
                obj.vertsNames = vertsNames;
            end

            obj.body2VertMap = containers.Map();
            for b = 1:length(obj.bodyNames)
                tmp = cellfun(@(x) strcmp(obj.bodyNames{b}, x), obj.vertsNames);
                obj.body2VertMap(obj.bodyNames{b}) = find(tmp,1);
            end

            obj.setJoints(obj.home)
            obj.startInd = 1;

        end
        function delete(obj)
            robot_mex(obj.DESTROY, obj.robotPtr);
        end
        function setJoints(obj, jointAngles)
            jointAngles = reshape(jointAngles, [], 1);
            jointAngles = min(jointAngles, obj.jointMaximums);
            jointAngles = max(jointAngles, obj.jointMinimums);
            robot_mex(obj.SETJOINTS, obj.robotPtr, jointAngles);
        end
        function jointAgnles = getJoints(obj)
            jointAgnles = robot_mex(obj.GETJOINTS, obj.robotPtr);
        end
        function J = getJacobian(obj)
            J = robot_mex(obj.GETJACOB, obj.robotPtr);
        end
        function T = getTransform(obj, index)
            T = robot_mex(obj.GETTRANS, obj.robotPtr, index);
        end
        function operationalPosition = getOperationalPosition(obj, index)
            operationalPosition = robot_mex(obj.GETOPPOS, obj.robotPtr, index);
        end
        function [JeR, JwR, JeL, JwL] = getElbowWristJacobians(obj)
            R = cell(14, 1);
            P = cell(14, 1);
            for i = 1:7
                % right
                T = obj.getTransform(i+1);
                R{i} = T(1:3, 1:3);
                P{i} = T(1:3, end);
                % left
                T = obj.getTransform(i+1+10);
                R{i+7} = T(1:3, 1:3);
                P{i+7} = T(1:3, end);
            end

            JeR = zeros(3, obj.numJoints);
            for j = 1:2
                Rj = R{j};
                JeR(:,j) = cross(Rj(:,3),P{3}-P{j});
            end
            JwR = zeros(3, obj.numJoints);
            for j = 1:4
                Rj = R{j};
                JwR(:,j) = cross(Rj(:,3),P{5}-P{j});
            end

            JeL = zeros(3, obj.numJoints);
            for j = 1:2
                Rj = R{j+7};
                JeL(:,j + obj.numJoints/2) = cross(Rj(:,3), P{3+7}-P{j+7});
            end
            JwL = zeros(3, obj.numJoints);
            for j = 1:4
                Rj = R{j+7};
                JwL(:,j + obj.numJoints/2) = cross(Rj(:,3),P{5+7}-P{j+7});
            end

        end
        function plotObject(obj)
            % mainly used for debugging
            vNew = [];
            color = [];
            lighDir = [-1; -.5; -1];
            lighDir = lighDir./norm(lighDir);
            for i = 1:length(obj.bodyNames)
                ind = obj.body2VertMap(obj.bodyNames{i});
                tmp = obj.verts{ind};
                tmp2 = obj.vertsNormals{ind};
                T = obj.getTransform(i);
                vNew = cat(1, vNew, cat(2, tmp, ones(size(tmp,1), 1))*T');
                vNormNew = tmp2*T(1:3,1:3)';
                color = cat(1, color, max(-vNormNew*lighDir, .3));
            end
            vNew = vNew(:,1:3);
            T = reshape(1:size(vNew, 1), 3, [])';

            trisurf(T, vNew(:, 1), vNew(:, 2),  vNew(:, 3), color)
            axis equal
            ylim([-.7 .7])
            xlim([-.5 .9])
            zlim([-0 1.2])
            view(120, 15)
            colormap gray
            shading interp
        end
        function indexCallback(obj, Q, sld, sld2)
            sld2.Value = sld.Value;
            numFrames = size(Q, 1);
            index = min(max(floor(sld.Value), 1), numFrames);
            obj.setJoints(Q(index, :))
            obj.plotObject()
        end
        function startCallback(obj)
            obj.startInd = floor(obj.hSpinner.Value);
        end
        function stopCallback(obj, fileName)
            endInd = floor(obj.hSpinner.Value);
            inds = obj.startInd:endInd;
            exercises = {'forwardRaise', 'lateralRaise', 'shoulderPress', 'internalExternalRotation'};
            val = input('Which exercise: 1) forwardRaise 2) lateralRaise 3) shoulderPress 4) internalExternalRotation');
%             num = length(Utils.getAllFileNames(exercises{val}))+1;
            fileName = [obj.directory fileName '_' exercises{val} '_inds.mat'];
            save(fileName, 'inds')
        end
        function timerCallback(obj, Q)
            obj.hSpinner.Value = obj.hSpinner.Value + 8;
            obj.indexCallback(Q, obj.hSpinner, obj.hSlider)
        end
        function playCallback(obj, Q, sld)
            if (strcmp(sld.Text,'Play'))
                sld.Text = 'Pause';
                obj.t = timer('StartDelay', 0, 'Period', .1, 'ExecutionMode', 'fixedRate', 'TimerFcn', @(x,y) obj.timerCallback(Q));
                start(obj.t);
            else
                sld.Text = 'Play';
                stop(obj.t);
            end
        end
        function complete = trajectoryBuilder(obj, Q, file)
            obj.directory = file(1:max(find(file=='/',9999)));
%             obj.side = 'L';
%             if contains(file,'R')
%                 obj.side = 'R';
%             end
            fileName = Utils.getFileName(file);
            fileName = fileName (1:find(fileName=='.',1)-1);
            complete = obj.validateInds(fileName);
            if complete
                return
            end
            if isempty(obj.fig) || ~isgraphics(obj.fig)
                obj.fig = uifigure;
                obj.hSpinner = uispinner(obj.fig,'Position',[100, 100, 150, 25]);
                obj.hSlider = uislider(obj.fig, 'Limits',[1, size(Q,1)],'Position',[100, 60, 150, 25]);
                set(obj.hSpinner,'ValueChangedFcn', @(sld, event) obj.indexCallback(Q, sld, obj.hSlider))
                set(obj.hSlider,'ValueChangedFcn', @(sld, event) obj.indexCallback(Q, sld, obj.hSpinner))
                uibutton(obj.fig, 'ButtonPushedFcn', @(sld,event) obj.startCallback(),'Position',[100, 190, 150, 25], 'Text', 'Start');
                uibutton(obj.fig, 'ButtonPushedFcn', @(sld,event) obj.stopCallback(fileName),'Position',[100, 150, 150, 25], 'Text', 'End');
                uibutton(obj.fig, 'ButtonPushedFcn', @(sld,event) obj.playCallback(Q, sld),'Position',[100, 220, 150, 25], 'Text', 'Play');
            end

            %             t = timer('StartDelay', 4, 'Period', 4, 'TasksToExecute', 2, ...
            %                 'ExecutionMode', 'fixedRate','TimerFcn', );
        end
        function bool = validateInds(obj, fileName)
            exercises = {'forwardRaise', 'lateralRaise', 'shoulderPress', 'internalExternalRotation'};
            for val = 1:length(exercises)
                if exist([obj.directory fileName '_' exercises{val} '_inds.mat']) == 0
                    warning([fileName ' in directory ' obj.directory ' is not complete'] )
                    bool = false;
                    return
                end
            end
            disp([fileName ' in directory ' obj.directory ' is complete'] )
bool = true;
        end
        function trajectoryPlayback(obj, Q)
            for i = 1:25:size(Q,1)
                obj.setJoints(Q(i,:));
                obj.plotObject();
                pause(0.0001);
            end

        end
    end
end