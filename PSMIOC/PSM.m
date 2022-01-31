classdef PSM < handle
    % PSM

    properties
        numCuts
        numDims
        numRadialBasis
        basisWidth
        cp
        BOFs = {};
        BOFsE = {};
        parameters
        W = [];
        typeMap
        H
        f
        Aeq
        beq
        Ain
        bin
        AeqBound
        beqBound
        AinBound
        binBound
        bofCounter = 0
        qPrime1i
        qPrime1f
        numDemos = 0
        q
        qd
        A
        k
        c
        V
        basisConfig
        KETarget
        velLimit = [];
    end
    methods(Static)
        function psm = load(name)
            load(name);
            minD = cp(1);
            maxD = cp(end);
            psm = PSM(numCuts, minD, maxD, numDims, V);
            psm.W = W;
            psm.cp = cp;
            psm.basisConfig = basisConfig;
            psm.numDims = numDims;
            psm.V = V;
            psm.numRadialBasis = numRadialBasis;
            psm.basisWidth = basisWidth;
        end
        function mVal = radialBasisI(qPrime, qPrime0, x, width)
            %             https://en.wikipedia.org/wiki/Error_function
            mVal = -sqrt(width*pi)*.5*(erfc(sqrt(1/width)*(qPrime - x) ) - erfc(sqrt(1/width)*(qPrime0 - x)));
        end
        function mVal = radialBasis(qPrime, x, width)
            mVal = exp(-1/width*(qPrime - x).^2);
        end
        function mVal = radialBasisD(qPrime, x, width)
            mVal = -(2/width*(qPrime - x)).*exp(-1/width*(qPrime - x).^2);
        end
        function mVal = radialBasisDD(qPrime, x, width)
            mVal = (4/width^2*(qPrime - x).^2 - 2/width).*exp(-1/width*(qPrime - x).^2);
        end
    end

    methods
        function obj = PSM(numCuts,minD,maxD,numDims, V)
            obj.cp = linspace(minD,maxD,numCuts);
%             obj.cp(1) = obj.cp(1)-1;
%             obj.cp(end) = obj.cp(end)+1;
            obj.numCuts = numCuts;
            obj.numDims = numDims;
            obj.typeMap = containers.Map('KeyType','char','ValueType','any');
            obj.q = zeros(numDims,1);
            obj.qd = zeros(numDims,1);
            obj.V = V;
            obj.numRadialBasis = 5;
            obj.basisWidth = 2;
        end

        function save(obj,name)
            W = obj.W;
            cp = obj.cp;
            basisConfig = obj.basisConfig;
            numDims = obj.numDims;
            numCuts = obj.numCuts;
            V = obj.V;
            numRadialBasis = obj.numRadialBasis;
            basisWidth = obj.basisWidth;
            save(name,'W','cp','basisConfig','numDims','numCuts','V','numRadialBasis', 'basisWidth')
        end

        function step(obj,dt)
            %             assert(0, 'this is broken')
            qPrime = inv(obj.V)*obj.q;
            qdPrime = inv(obj.V)*obj.qd;
            %             cut = BasisObjFnc.getJ(obj.cp,qPrime(1));
            %             ci = obj.c(cut);
            %             ki = obj.k(cut);

            %             qdhat1 = 0;
            %             qdhat1Next = 0;
            %             cut2 = BasisObjFnc.getJ(obj.cp,obj.qPrime1f(1));
            %             if  cut+1 <= cut2
            %                 for i = 2:cut+1
            %                     qdhat1Next = qdhat1Next+(obj.cp(i)-obj.cp(i-1))*obj.c(i-1)+...
            %                         .5*(obj.cp(i)^2-obj.cp(i-1)^2)*obj.k(i-1);
            %                 end
            %             else
            %                 for i = 2:cut
            %                     qdhat1Next = qdhat1Next+(obj.cp(i)-obj.cp(i-1))*obj.c(i-1)+...
            %                         .5*(obj.cp(i)^2-obj.cp(i-1)^2)*obj.k(i-1);
            %                 end
            %                 qdhat1Next = qdhat1Next+(obj.qPrime1f(1)-obj.cp(cut))*ci+...
            %                     .5*(obj.qPrime1f(1)^2-obj.cp(cut)^2)*ki;
            %             end
            %             qdhat1Next = sqrt(2*qdhat1Next);
            %
            %             for i = 2:cut
            %                 qdhat1 = qdhat1+(obj.cp(i)-obj.cp(i-1))*obj.c(i-1)+...
            %                     .5*(obj.cp(i)^2-obj.cp(i-1)^2)*obj.k(i-1);
            %             end
            %             qdhat1 = sqrt(2*(qdhat1+(qPrime(1)-obj.cp(cut))*ci+...
            %                 .5*(qPrime(1)^2-obj.cp(cut)^2)*ki));
            %             qdhat1 = real(qdhat1);
            %             qdhat1 = qdPrime(1);
            %             if  cut+1 <= cut2
            %                 ci = 0*0.00001+(.5*(qdhat1Next^2-qdhat1^2)-.5*(obj.cp(cut+1)^2-qPrime(1)^2)*ki)/(obj.cp(cut+1)-qPrime(1));
            %             else
            %                 ci = 0*0.00001+(.5*(qdhat1Next^2-qdhat1^2)-.5*(obj.qPrime1f(1)^2-qPrime(1)^2)*ki)/(obj.qPrime1f(1)-qPrime(1));
            %             end
            %             qddhat1 = ki*qPrime(1)+ci;
            %             qddhat = zeros(obj.numDims,1);
            %             qddhat(1) = qddhat1;
            %             d0 = zeros(obj.numDims,1);
            %             d0(1) = qPrime(1);
            %             d1 = zeros(obj.numDims,1);
            %             d1(1) = 1;
            %             dd1 = zeros(obj.numDims,1);
            %             for i = 2:length(qddhat)
            %                 coefs = obj.A(cut,:,i-1);
            %                 for d = 1:length(coefs)
            %                     d0(i) = d0(i)+coefs(d)*qPrime(1)^(d-1);
            %                 end
            %
            %                 for d = 2:length(coefs)
            %                     d1(i) = d1(i)+(d-1)*coefs(d)*qPrime(1)^(d-2);
            %                 end
            %
            %                 for d = 3:length(coefs)
            %                     dd1(i) = dd1(i)+(d-1)*(d-2)*coefs(d)*qPrime(1)^(d-3);
            %                 end
            %                 qddhat(i) = dd1(i)*qdhat1^2+d1(i)*qddhat1;
            %             end
            %             qdd = (obj.V*(qddhat+20*(d1*qdhat1-qdPrime) + 40*sqrt(20)*(d0-qPrime)));

            for d = 1
                m = KineticEnergy(obj, d);
                val = 0;
                for i = 1:length(obj.parameters)
                    if ~isnumeric(m{i})
                        val = val + m{i}(qPrime(1))*obj.parameters(i);
                    end
                end
                qdPrime(d) = max(sqrt(abs(2*val)), .01); % KE = .5*v^2  v = sqrt(2KE)
            end

            for d = 2:obj.numDims
                m = Position(obj, d);
                val = 0;
                for i = 1:length(obj.parameters)
                    if ~isnumeric(m{i})
                        val = val + m{i}(qPrime(1))*obj.parameters(i);
                    end
                end
                m = Direction(obj, d);
                valD = 0;
                for i = 1:length(obj.parameters)
                    if ~isnumeric(m{i})
                        valD = valD + m{i}(qPrime(1))*obj.parameters(i);
                    end
                end
                qdPrime(d) = valD*qPrime(1) + -100*(qPrime(d) - val);
            end
            obj.qd = obj.V*qdPrime;
            %             obj.qd = obj.qd+qdd*dt;
            obj.q = obj.q+obj.qd*dt;
        end

        function [QPrimei, QdPrimei,QPrimef,QdPrimef] = convertToPrime(obj,Qi, Qdi,Qf,Qdf)
            QPrimei = Qi;
            QPrimef = Qf;
            QdPrimei = Qdi;
            QdPrimef = Qdf;
            for d = 1:size(Qi,2)
                tmp = [Qi{:,d}]';
                if length(tmp) < 2
                    continue
                end
                tmp = inv(obj.V)*tmp;
                for i = 1:size(Qi,1)
                    QPrimei{i,d} = tmp(i);
                end
            end
            for d = 1:size(Qf,2)
                tmp = [Qf{:,d}]';
                if isempty(tmp)
                    continue
                end
                tmp = inv(obj.V)*tmp;
                for i = 1:size(Qf,1)
                    QPrimef{i,d} = tmp(i);
                end
            end
            for d = 1:size(Qdi,2)
                tmp = [Qdi{:,d}]';
                if isempty(tmp)
                    continue
                end
                tmp = inv(obj.V)*tmp;
                for i = 1:size(Qdi,1)
                    QdPrimei{i,d} = tmp(i);
                end
            end
            for d = 1:size(Qdf,2)
                tmp = [Qdf{:,d}]';
                if isempty(tmp)
                    continue
                end
                tmp = inv(obj.V)*tmp;
                for i = 1:size(Qdf,1)
                    QdPrimef{i,d} = tmp(i);
                end
            end
        end

        function setupBoundaryConds(obj,boundaryConfig, Qi, Qdi, Qf, Qdf)
            % set up PSM basis boundary condtions
            [QPrimei, QdPrimei,QPrimef,QdPrimef] = convertToPrime(obj,Qi, Qdi,Qf,Qdf);
            obj.qPrime1f = QPrimef{1,1};
            obj.qPrime1i = QPrimei{1,1};
            obj.AeqBound = cell(size(QPrimei,2),1);
            obj.beqBound = cell(size(QPrimei,2),1);
            obj.AinBound = cell(size(QPrimei,2),1);
            obj.binBound = cell(size(QPrimei,2),1);
            for d = 1:size(QPrimei,2)
                qPrimei = QPrimei(:,d);
                qPrimef = QPrimef(:,d);
                qdPrimei = QdPrimei(:,d);
                qdPrimef = QdPrimef(:,d);
%                 g = @(qPrime) qPrime*0;
%                 obj.addBoundaryConstraintIn(m,g, qPrimei{1},d);
                if ~isempty(qdPrimei{1}) && boundaryConfig.('startKE')
                    m = obj.KineticEnergy(1);
                    KEi = 0.5*qdPrimei{1}^2;
                    g = @(qPrime) qPrime*0+KEi;
                    obj.addBoundaryConstraint(m, g, qPrimei{1},d)
                end
                m = obj.KineticEnergy(1);
                g = @(qPrime) qPrime*0;
                obj.addBoundaryConstraintIn(m,g, qPrimef{1},d);
                if ~isempty(qdPrimef{1}) && boundaryConfig.('endKE')
                    KEf = 0.5*qdPrimef{1}^2;
                    g = @(qPrime) qPrime*0+KEf;
                    m = obj.KineticEnergy(1);
                    obj.addBoundaryConstraint(m, g, qPrimef{1},d)
                end
                for dim = 2:obj.numDims
                    if ~isempty(qPrimei{dim}) && boundaryConfig.('startPos')
                        g = @(qPrime) qPrime*0+qPrimei{dim};
                        m = obj.Position(dim);
                        obj.addBoundaryConstraint(m, g, qPrimei{1},d)
                    end
                    if ~isempty(qdPrimei{dim})  && boundaryConfig.('startDir')
                        val = qdPrimei{dim}./qdPrimei{1};
                        g = @(qPrime) qPrime*0+val;
                        m = obj.Direction(dim);
                        obj.addBoundaryConstraint(m, g, qPrimei{1},d)
                    end
                    if ~isempty(qPrimef{dim}) && boundaryConfig.('endPos')
                        g = @(qPrime) qPrime*0+qPrimef{dim};
                        m = obj.Position(dim);
                        obj.addBoundaryConstraint(m, g, qPrimef{1},d)
                    end
                    if ~isempty(qdPrimef{dim}) && boundaryConfig.('endDir')
                        val = qdPrimef{dim}./qdPrimef{1};
                        g = @(qPrime) qPrime*0+val;
                        m = obj.Direction(dim);
                        obj.addBoundaryConstraint(m, g, qPrimef{1},d)
                    end
                end
            end
        end

        function addBoundaryConstraint(obj, m, g, qPrime1,demo)
            r = zeros(1,length(m));
            for i = 1:length(m)
                if ~isnumeric(m{i}) && ~isempty(qPrime1)
                    f1 = m{i};
                    r(i) = f1(qPrime1);
                end
            end
            b = g(qPrime1);
            obj.AeqBound{demo} = [obj.AeqBound{demo};r];
            obj.beqBound{demo} = [obj.beqBound{demo};b];
        end

        function addBoundaryConstraintIn(obj, m, g, qPrime1,demo)
            r = zeros(1,length(m));
            for i = 1:length(m)
                if ~isnumeric(m{i}) && ~isempty(qPrime1)
                    f1 = m{i};
                    r(i) = f1(qPrime1);
                end
            end
            b = g(qPrime1);
            obj.AinBound{demo} = [obj.AinBound{demo};-r];
            obj.binBound{demo} = [obj.binBound{demo};b];
        end

        function setupBasisFncs(obj, basisConfig, Qi, Qf)
            % set up PSM basis functions. QPrimei is a vector of starting
            % qPrime points and QPrimef is a vector of ending
            % qPrime points
            %             obj.posTarget = posTarget;
            %             obj.dirTarget = dirTarget;
            %             obj.curvTarget = curvTarget;
            %             obj.effortTarget = effortTarget;
            %             obj.KETarget = KETarget;
            [QPrimei, ~, QPrimef, ~] = obj.convertToPrime(Qi, [], Qf, []);
            obj.basisConfig = basisConfig;
            fNames = fieldnames(basisConfig);
            for demoInd = 1:size(QPrimei,2)
                qPrimei = QPrimei{1, demoInd};
                qPrimef = QPrimef{1, demoInd};
                for fi = 1:length(fNames)
                    fName = fNames{fi};
                    [dims, targets] = basisConfig.(fName);
                    for ti = 1:length(targets)
                        target = targets(ti);
                        for d = 1:length(dims)
                            for cut = 1:obj.numCuts-1

                                switch fName

                                    case 'KETarget'
                                        assert(dims(d)==1, 'KE can only be used for dim 1')
                                        m = obj.KineticEnergy(dims(d));
                                        v = ConstTargetFnc(target);
                                        obj.addBOF(m, v, cut, qPrimei, qPrimef,['KE, cut :' num2str(cut), ' target:' num2str(target)], ['KE' num2str(dims(d))], demoInd);
                                    case 'effortTarget'
                                        assert(dims(d)==1, 'Effort can only be used for dim 1')
                                        m = obj.Effort(dims(d));
                                        v = ConstTargetFnc(target);
                                        obj.addBOF(m,v, cut, qPrimei, qPrimef,['effort, cut :' num2str(cut), ' target:' num2str(target)], ['Effort' num2str(dims(d))], demoInd);
                                    case 'posTarget'
                                        assert(dims(d) > 1, 'Position can only be used for dim > 1')
                                        m = obj.Position(dims(d));
                                        v = ConstTargetFnc(target);
                                        obj.addBOF(m,v, cut, qPrimei, qPrimef, ['position, cut :' num2str(cut), ' target:' num2str(target)], ['Position' num2str(dims(d))], demoInd);
                                    case 'dirTarget'
                                        assert(dims(d) > 1, 'Direction can only be used for dim > 1')
                                        m = obj.Direction(dims(d));
                                        v = ConstTargetFnc(target);
                                        obj.addBOF(m,v, cut, qPrimei, qPrimef, ['direction, cut :' num2str(cut), ' target:' num2str(target)], ['Direction' num2str(dims(d))], demoInd);
                                    case 'curvTarget'
                                        assert(dims(d) > 1, 'Curvature can only be used for dim > 1')
                                        m = obj.Curvature(dims(d));
                                        v = ConstTargetFnc(target);
                                        obj.addBOF(m,v, cut, qPrimei, qPrimef, ['curvature, cut :' num2str(cut), ' target:' num2str(target)], ['Curvature' num2str(dims(d))], demoInd);
                                end


                                % %                             % effort
                                % %                             if target <= length(effortTarget)
                                % %                                 m = obj.Effort(cut, 1);
                                % %                                 v = ConstTargetFnc(target);
                                % %                                 bofE = obj.addBOF(m,v, cut,['effort' ': ' num2str(cut)],d);
                                % %                             end
                                % %                             % KE
                                % %                             if target <= length(KETarget)
                                % %                                 mk = obj.KineticEnergy(cut, 1);
                                % %                                 vk = ConstTargetFnc(target);
                                % %                                 bofK = obj.addBOF(mk,vk, cut,['KE' ': ' num2str(cut)],d);
                                % %                             end
                                % %                             % constraints
                                % %                             if cut > 1
                                % %                                 for v = [0 1]
                                % %                                     offset = .5*(obj.cp(cut+1)-obj.cp(cut));
                                % %                                     g = @(qPrime) qPrime*0;
                                % %                                     mk = obj.KineticEnergy(cut, 1);
                                % %                                     bofK.addConstraintIn(mk, g, obj.cp(cut)+0.000001+offset*v)
                                % %                                     if ~isempty(obj.velLimit)
                                % %                                         g = @(qPrime) qPrime*0+obj.velLimit;
                                % %                                         bofK.addConstraintIn2(mk, g, obj.cp(cut)+0.000001+offset*v)
                                % %                                     end
                                % %                                 end
                                % %                             end
                                % %                             for target = 1:2
                                % %                                 % position
                                % %                                 if target <= length(posTarget)
                                % %                                     mp = obj.Position(cut, dim);
                                % %                                     vp = ConstTargetFnc(obj.cp, posTarget(target), cut);
                                % %                                     bofP = obj.addBOF(mp,vp, cut,['pos_' num2str(dim) ': ' num2str(cut)],d);
                                % %                                 end
                                % %                                 % direction
                                % %                                 if target <= length(dirTarget)
                                % %                                     md = obj.Direction(cut, dim);
                                % %                                     vd = ConstTargetFnc(obj.cp, dirTarget(target), cut);
                                % %                                     bofD = obj.addBOF(md,vd, cut,['dir_' num2str(dim)  ': ' num2str(cut)],d);
                                % %                                 end
                                % %                                 % curvature
                                % %                                 if target <= length(curvTarget)
                                % %                                     mc = obj.Curvature(cut, dim);
                                % %                                     vc = ConstTargetFnc(obj.cp, curvTarget(target), cut);
                                % %                                     bofC = obj.addBOF(mc,vc, cut,['curv_' num2str(dim) ': ' num2str(cut)],d);
                                % %                                 end
                                % %                             end

                            end
                        end
                    end
                end
            end
        end

        function setupBasisFncsE(obj, basisConfig, T, Demos, Demosd, Demosdd)
            % setup basis functions for expert
            obj.numDemos = length(Demos);
            for demoInd = 1:length(Demos)
                demo = inv(obj.V)*Demos{demoInd};
                demod = inv(obj.V)*Demosd{demoInd};
                demodd = inv(obj.V)*Demosdd{demoInd};
                t = T{demoInd };
                qPrimei = demo(1,1);
                qPrimef = demo(1,end);

                fNames = fieldnames(basisConfig);
                for fi = 1:length(fNames)
                    fName = fNames{fi};
                    [dims, targets] = basisConfig.(fName);
                    for ti = 1:length(targets)
                        target = targets(ti);
                        for d = 1:length(dims)
                            for cut = 1:obj.numCuts-1

                                switch fName

                                    case 'KETarget'
                                        assert(dims(d)==1, 'KE can only be used for dim 1')
                                        g = obj.KineticEnergyE(t, demo, demod, demodd, dims(d));
                                        v = ConstTargetFnc(target);
                                        obj.addBOFE(g, v, cut, qPrimei, qPrimef, ['KE, cut :' num2str(cut), ' target:' num2str(target)], ['KE' num2str(dims(d))], demoInd );
                                    case 'effortTarget'
                                        assert(dims(d)==1, 'Effort can only be used for dim 1')
                                        g = obj.EffortE(t, demo, demod, demodd, dims(d));
                                        v = ConstTargetFnc(target);
                                        obj.addBOFE(g, v, cut, qPrimei, qPrimef, ['effort, cut :' num2str(cut), ' target:' num2str(target)], ['Effort' num2str(dims(d))], demoInd);
                                    case 'posTarget'
                                        assert(dims(d) > 1, 'Position can only be used for dim > 1')
                                        g = obj.PositionE(t, demo, demod, demodd, dims(d));
                                        v = ConstTargetFnc(target);
                                        obj.addBOFE(g,v, cut, qPrimei, qPrimef, ['position, cut :' num2str(cut), ' target:' num2str(target)], ['Position' num2str(dims(d))], demoInd);
                                    case 'dirTarget'
                                        assert(dims(d) > 1, 'Direction can only be used for dim > 1')
                                        g = obj.DirectionE(t, demo, demod, demodd, dims(d));
                                        v = ConstTargetFnc(target);
                                        obj.addBOFE(g,v, cut, qPrimei, qPrimef, ['direction, cut :' num2str(cut), ' target:' num2str(target)], ['Direction' num2str(dims(d))], demoInd);
                                    case 'curvTarget'
                                        assert(dims(d) > 1, 'Curvature can only be used for dim > 1')
                                        g = obj.CurvatureE(t, demo, demod, demodd, dims(d));
                                        v = ConstTargetFnc(target);
                                        obj.addBOFE(g,v, cut, qPrimei, qPrimef,['curvature, cut :' num2str(cut), ' target:' num2str(target)], ['Curvature' num2str(dims(d))], demoInd);
                                end
                            end
                        end
                    end
                end

                %                 for cut = 1:obj.numCuts-1
                %                     for target = 1:3
                %                         % effort
                %                         if target <= length(effortTarget)
                %                             v = ConstTargetFnc(effortTarget(target));
                %                             g = obj.EffortE(t,demo,cut, 1);
                %                             bofEE = obj.addBOFE(g,v, qPrimei,qPrimef,['effort' ': ' num2str(effortTarget(target))],d);
                %                         end
                %                         % KE
                %                         if target <= length(KETarget)
                %                             vk = ConstTargetFnc(KETarget(target));
                %                             gk = obj.KineticEnergyE(t,demo,cut, 1);
                %                             bofKE = obj.addBOFE(gk,vk, qPrimei,qPrimef,['KE' ': ' num2str(KETarget(target))],d);
                %                         end
                %                     end
                %                     for dim = 2:obj.numDims
                %                         for target = 1:2
                %                             % position
                %                             if target <= length(posTarget)
                %                                 vp = ConstTargetFnc(obj.cp, posTarget(target), cut);
                %                                 gp = obj.PositionE(t, demo,cut, dim);
                %                                 bofPE = obj.addBOFE(gp,vp, qPrimei,qPrimef,['pos_' num2str(dim) ': ' num2str(posTarget(target))],d);
                %                             end
                %                             % direction
                %                             if target <= length(dirTarget)
                %                                 vd = ConstTargetFnc(obj.cp, dirTarget(target), cut);
                %                                 gd = obj.DirectionE(t,demo,cut, dim);
                %                                 bofDE = obj.addBOFE(gd,vd, qPrimei,qPrimef,['dir_' num2str(dim) ': ' num2str(dirTarget(target))],d);
                %                             end
                %                             % curvature
                %                             if target <= length(curvTarget)
                %                                 vc = ConstTargetFnc(obj.cp, curvTarget(target), cut);
                %                                 gc = obj.CurvatureE(t,demo,cut, dim);
                %                                 bofCE = obj.addBOFE(gc,vc, qPrimei,qPrimef,['curv_' num2str(dim)  ': ' num2str(curvTarget(target))],d);
                %                             end
                %                         end
                %                     end
                %                 end
            end
        end

        function buildOptimize(obj)
            obj.Aeq = {};
            obj.beq = {};
            obj.Ain = {};
            obj.bin = {};
            obj.c = {};
            for d = 1:size(obj.BOFs,2)
                bofs = obj.BOFs(:,d);
                Aeqd = [];
                beqd = [];
                Aind = [];
                bind = [];
                for i = 1:length(bofs)
                    bofs{i}.build();
                    Aeqd = [Aeqd;bofs{i}.Aeq];
                    beqd = [beqd;bofs{i}.beq];
                    Aind = [Aind;bofs{i}.Ain];
                    bind = [bind;bofs{i}.bin];
                end
                obj.Aeq{d} = Aeqd;
                obj.beq{d} = beqd;
                obj.Ain{d} = Aind;
                obj.bin{d} = bind;
            end
            for d = 1:size(obj.BOFsE,2)
                bofsE = obj.BOFsE(:,d);
                for i = 1:length(bofsE)
                    bofsE{i}.build();
                end
            end
        end

        function val = optimizePSM(obj)
            obj.parameters = [];
            options = optimoptions('quadprog','MaxIterations',2000);
            for d = 1:size(obj.BOFs,2)
                Aind =  [];
                bind =  [];
                Aeqd =  [];
                beqd =  [];
                obj.H = zeros(size(obj.BOFs{1}.H));
                obj.f = 0;
                bofs = obj.BOFs(:,d);
                tmp = {};
                for i = 1:length(bofs)
                    tmp{i} = bofs{i}.H;
                end
                obj.H = getH(tmp, obj.W);
                obj.H = obj.H + .0000000001*eye(size(obj.H,1));
                for i = 1:length(bofs)
                    obj.f = obj.f+obj.W(i)*bofs{i}.f;
                end
                %                     Aeqd = [obj.Aeq{d}];
                %                     beqd = [obj.beq{d}];
                %                 Aind = [obj.Ain{d}];
                %                 bind = [obj.bin{d}];
                if length(obj.AeqBound)>=d
                    Aeqd = [Aeqd;obj.AeqBound{d}];
                    beqd = [beqd;obj.beqBound{d}];
                    %                     Aind = [Aind;obj.AinBound{d}];
                    %                     bind = [bind;obj.binBound{d}];
                end
                [x,val] = quadprog(2*obj.H, obj.f,Aind,bind,Aeqd,beqd,[],[],[],options);
                obj.parameters = [obj.parameters x];
            end
            %             obj.A = zeros(obj.numCuts-1, obj.numRadialBasis, obj.numDims);
            %             obj.k = zeros(obj.numCuts-1, 1);
            %             obj.c = zeros(obj.numCuts-1, 1);
            %             Af = reshape(obj.A, [], 1);
            %             x = obj.parameters;  % [k;c;Af];
            %             obj.k = x(1:length(obj.k),1);
            %             obj.c = x((1:length(obj.c))+length(obj.k),1);
            %             obj.A = reshape(x(length(obj.k)+length(obj.c)+1:end,1), obj.numCuts-1, obj.numRadialBasis, obj.numDims-1);
            obj.A = reshape(x, obj.numRadialBasis+1, obj.numDims);
        end

        function e = evaluate(obj,Plot)
            eJ = [];
            eC = [];
            qPrime1 = linspace(obj.qPrime1i,obj.qPrime1f,100);
            keys = flip(obj.typeMap.keys);
            for k = 1:length(keys)
                key = keys{k};
                if contains(key,'_expert')
                    continue
                end
                dJT = 0;
                dcET = 0;
                bofsKey = obj.typeMap(key);
                bofsKeyE = obj.typeMap([key '_expert']);
                if exist('Plot') == 1
                    figure(k)
                    subplot(1,2,2);hold on
                    title(key)
                end
                for i = 1:length(bofsKey{1})
                    bof = bofsKey{1}{i};
                    Wind = bof.Wind;
                    bofE = bofsKeyE{i};
                    Hi = obj.W(Wind)*bof.H;
                    fi = obj.W(Wind)*bof.f;
                    ci = obj.W(Wind)*bof.c;
                    x = obj.parameters(:,1);
                    J = x'*Hi*x+fi'*x+ci;
                    dc = bofE.dc;
                    dHi = obj.W(Wind)*bof.dH;
                    dfi = obj.W(Wind)*bof.df;
                    dci = obj.W(Wind)*bof.dc;
                    dJ = zeros(size(dfi,3),1);
                    dcE = zeros(size(dfi,3),1);
                    for j = 1:length(dJ)
                        dJ(j) = x'*dHi(:,:,j)*x+dfi(:,1,j)'*x+dci(j);
                        dcE(j) = obj.W(Wind)*dc(j);
                    end
                    dJT = dJT+ dJ;
                    dcET = dcET+ dcE;
                    eJ = [eJ; J];
                    eC = [eC;obj.W(Wind)*bofE.c];
                end
                if exist('Plot') == 1
                    plot(qPrime1, dcET,'Color','k','LineWidth',2);hold on
                    plot(qPrime1, dJT,'Color','b','LineWidth',2)
                    ylabel('$$\sum_{i} w_i \frac{d}{dq''_1}J_i^{be}$$','Interpreter','latex','FontSize',16)
                    xlabel('$$q''_1 $$','Interpreter','latex','FontSize',16)
                    axis tight
                    set(gcf,'Color','w')
                end
            end
            e = sum(eC)/sum(eJ);
        end

        function optimize(obj)
            options = optimoptions('fmincon','SpecifyObjectiveGradient',true,'MaxFunctionEvaluations',3E4);
            obj.W = fmincon(@(w) obj.objective(w), obj.W, [],[],[],[],obj.W*0, obj.W*0+1, [], options); %@(w) cont(w)
            obj.objective(obj.W)
        end

        function [val,gradT] = objective(obj,w)
            obj.W = w;
            obj.optimizePSM();
            val = 0;
            gradT = zeros(length(obj.BOFs{1,1}),1);
            for d = 1:size(obj.BOFs,2)
                bofs = obj.BOFs(:,d);
                bofsE = obj.BOFsE(:,d);
                grad = zeros(length(bofs),1);
                for i = 1:length(bofs)
                    bof = bofs{i};
                    Hi = bof.H;
                    fi = bof.f;
                    ci = bof.c;
                    x = obj.parameters(:,d);
                    J = x'*Hi*x+fi'*x+ci;
                    c = bofsE{i}.c;
                    try
                    grad(i) = J-c;
                    catch
                    keyboard
                    end
                    val = val+obj.W(i)*J-obj.W(i)*c;
                end
                gradT = gradT+grad;
            end
            gradT = -gradT;
            val = -val;
            
%             val = val + 0*sum(w.^2); % normalization
%             gradT = gradT + 0*2*w;
            

            val
            
        end

        function bof = addBOF(obj, m, v, cut, qPrimei, qPrime1f, description, type, d)

            
            bof = BasisObjFnc(obj.cp, m, v, cut, max(obj.cp(cut), qPrimei), min(obj.cp(cut+1), qPrime1f));
            
            bof.type = type;
            bof.description = description;
            if size(obj.BOFs,2) < d
                obj.bofCounter = 0;
            end
            obj.bofCounter = obj.bofCounter+1;
            obj.BOFs{obj.bofCounter, d} = bof;
            if ~obj.typeMap.isKey(bof.type)
                obj.typeMap(bof.type) = {{}};
            end
            cells = obj.typeMap(bof.type);
            if length(cells) < d
                cells{d}{1} = bof;
            else
                cells{d}{length(cells{d})+1} = bof;
            end
            obj.typeMap(bof.type) = cells;
            if d == 1
                obj.W = [obj.W;1];
                bof.Wind = length(obj.W);
            end
        end

        function bofE = addBOFE(obj, g, v, cut, qPrime1i, qPrime1f, description, type, d)
            % need to add expert function right after adding BOFs
            type = [type '_expert'];
            bofE = BasisObjFncE(g, v, max(obj.cp(cut), qPrime1i), min(obj.cp(cut+1), qPrime1f));
            if isempty(obj.BOFsE)
                obj.BOFsE{1,obj.numDemos} = [];
            end
            v = obj.BOFsE(:,d);
            obj.BOFsE{length([v{:}])+1, d} = bofE;
            bofE.type = type;
            bofE.description = description;
            if ~obj.typeMap.isKey(bofE.type)
                obj.typeMap(type) = {};
            end
            cells = obj.typeMap(bofE.type);
            cells{length(cells)+1} = bofE;
            obj.typeMap(bofE.type) = cells;
        end

        function [A, functionsM] = preBasis(obj)
            A = zeros(obj.numRadialBasis+1, obj.numDims);
            %             k = zeros(obj.numCuts-1, 1);
            %             c = zeros(obj.numCuts-1, 1);
            functionsM = {};
        end

        function m = postBasis(obj, A, functionsM)
            m = BasisFnc(reshape(A, [], 1), functionsM);
        end

        function [demo, demod, demodd] = preBasisE(obj, t, demo)
            dt = t(2)-t(1);
            demod = demo;
            for i = 1:size(demo,1)
                demod(i,:) = d_dt(demo(i,:),dt);
            end
            demodd = demo;
            for i = 1:size(demo,1)
                demodd(i,:) = d_dt(demod(i,:),dt);
            end
        end

        function g = KineticEnergyE(obj, t, demo, demod, demodd, dim)
%             [demo, demod, demodd] = obj.preBasisE(t,demo);
            V = .5*demod(1,:).^2;
            g = @(qPrime1) interp1(demo(1,:), V, qPrime1,'nearest','extrap');
            %             g = BasisFncE(obj.cp, g, cut);
        end

        function g = EffortE(obj, t, demo, demod, demodd, dim)
%             [demo, demod, demodd] = obj.preBasisE(t,demo);
            g = @(qPrime1) interp1(demo(1,:), demodd(1,:), qPrime1, 'nearest','extrap');
            %             g = BasisFncE(obj.cp, g, cut);
        end

        function g = PositionE(obj, t, demo, demod, demodd, dim)
%             [demo, demod, demodd] = obj.preBasisE(t,demo);
            g = @(qPrime1) interp1(demo(1,:), demo(dim,:), qPrime1,'nearest','extrap');
            %             g = BasisFncE(obj.cp, g, cut);
        end

        function g = DirectionE(obj, t, demo, demod, demodd, dim)
%             [demo, demod, demodd] = obj.preBasisE(t,demo);
%             d = linspace(demo(1,1),demo(1,end),200);
%             D = interp1(demo(1,:), demo(dim,:), d,'makima','extrap');
%             dD = d(2) - d(1);
%             Dd = d_dt(D,dD);
            g = @(qPrime1) interp1(demo(1,:), demod(dim,:)./demod(1,:), qPrime1,'nearest','extrap');
            %             g = BasisFncE(obj.cp, g, cut);
        end

        function g = CurvatureE(obj, t, demo, demod, demodd, dim)
%             [demo, demod, demodd] = obj.preBasisE(t,demo);
            d = linspace(demo(1,1),demo(1,end),200);
            D = interp1(demo(1,:), demo(dim,:), d,'pchip','extrap');
            dD = d(2) - d(1);
            Dd = d_dt(D,dD);
            Ddd = d_dt(Dd,dD);

% demo(dim,:)./demod(1,:)
% f(q)
% f'(q)*q' = demod(dim,:)./demod(1,:)
% f''(q)*q'^2 + f'(q)*q'' 
% DddPrime = 
            g = @(qPrime1) interp1(d, Ddd, qPrime1,'nearest','extrap');
            %             g = BasisFncE(obj.cp, g, cut);
        end

        function m = Position(obj, dim)
            [A, functionsM] = obj.preBasis();
            X = linspace(obj.cp(1), obj.cp(end), obj.numRadialBasis);
            for i = 1 : obj.numRadialBasis
                A(i, dim) = length(functionsM)+1;
                functionsM{A(i, dim)} = @(qPrime1) PSM.radialBasis(qPrime1, -X(i), obj.basisWidth);
            end
             A(obj.numRadialBasis+1, dim) = length(functionsM)+1;
            functionsM{A(obj.numRadialBasis+1, dim)} = @(qPrime1) 1 + 0*qPrime1;
            m = obj.postBasis(A,functionsM);
        end

        function m = Direction(obj, dim)
            [A, functionsM] = obj.preBasis();
            X = linspace(obj.cp(1), obj.cp(end), obj.numRadialBasis);
            for i = 1 : obj.numRadialBasis
                A(i, dim) = length(functionsM)+1;
                functionsM{A(i, dim)} = @(qPrime1) PSM.radialBasisD(qPrime1, -X(i), obj.basisWidth);
%                     functionsM{A(i, dim)} = @(qPrime1) PSM.radialBasis(qPrime1, -X(i), obj.basisWidth);
            end
            m = obj.postBasis(A,functionsM);
        end

        function m = Curvature(obj, dim)
            [A, functionsM] = obj.preBasis();
            X = linspace(obj.cp(1), obj.cp(end), obj.numRadialBasis);
            for i = 1 : obj.numRadialBasis
                A(i, dim) = length(functionsM)+1;
                functionsM{A(i, dim)} = @(qPrime1) PSM.radialBasisDD(qPrime1, -X(i), obj.basisWidth);
%                 functionsM{A(i, dim)} = @(qPrime1) PSM.radialBasis(qPrime1, -X(i), obj.basisWidth);
            end
            m = obj.postBasis(A,functionsM);
        end

        function m = Effort(obj, dim)
            [A, functionsM] = obj.preBasis();
            X = linspace(obj.cp(1), obj.cp(end), obj.numRadialBasis);
            for i = 1 : obj.numRadialBasis
                A(i, dim) = length(functionsM)+1;
                functionsM{A(i, dim)} = @(qPrime1) PSM.radialBasis(qPrime1, -X(i), obj.basisWidth);
            end
            m = obj.postBasis(A,functionsM);
        end

        function m = KineticEnergy(obj, dim)
            [A, functionsM] = obj.preBasis();
            X = linspace(obj.cp(1), obj.cp(end), obj.numRadialBasis);
            for i = 1 : obj.numRadialBasis
                A(i, dim) = length(functionsM)+1;
%                 functionsM{A(i, dim)} = @(qPrime1) PSM.radialBasisI(qPrime1, obj.cp(1), -X(i), obj.basisWidth);
                  functionsM{A(i, dim)} = @(qPrime1) PSM.radialBasis(qPrime1, -X(i), obj.basisWidth);
            end
            A(obj.numRadialBasis+1, dim) = length(functionsM)+1;
            functionsM{A(obj.numRadialBasis+1, dim)} = @(qPrime1) 1 + 0*qPrime1;
            
            m = obj.postBasis(A,functionsM);

            %             mAll = {};
            %             for i = 1:cut
            %                 [A, functionsM] = obj.preBasis();
            %                 k(i,dim) = length(functionsM)+1;
            %                 functionsM{k(i,dim)} = @(qPrime1) .5*(qPrime1.^2-obj.cp(i)^2);
            %                 c(i,dim) = length(functionsM)+1;
            %                 functionsM{c(i,dim)} = @(qPrime1) qPrime1-obj.cp(i);
            %                 m = obj.postBasis(i, A, k,c,functionsM);
            %                 mAll{length(mAll)+1} = {m};
            %             end
            %             for i =1:length(m)
            %                 val = obj.mCombine(mAll,i,cut);
            %                 m{i} = val;
            %             end
        end

        function f = mCombine(obj, mAll,i,cut)
            f = 0;
            for j = 1:length(mAll)
                mj = mAll{j}{1};
                mji = mj{i};
                if ~isnumeric(mji)
                    if j < cut
                        mji = @(qPrime1) qPrime1*0+mji(obj.cp(j+1)-0.000001).*(qPrime1 > obj.cp(cut) ).*(qPrime1 < obj.cp(cut+1) );
                    end
                    if isnumeric(f) && f == 0
                        f = @(qPrime1) mji(qPrime1);
                    else
                        f = @(qPrime1) f(qPrime1)+mji(qPrime1);
                    end
                end
            end
        end


        function plotBasisFcns(obj,qi,qf)
            types = flip(obj.typeMap.keys);
            for t = 1:length(types)
                if contains(types{t},'expert')
                    type = types{t};
                    tmp = find(strcmp(types,type(1:end-7)));
                    figure(tmp)
                    subplot(1,2,1)
                    bofsE = obj.typeMap(types{t});
                    for i = 1:length(bofsE)
                        qPrime1 = linspace(bofsE{i}.qPrime1i, bofsE{i}.qPrime1f, 200)';
                        valv = 0;
                        g = bofsE{i}.g;
                        valv = valv+g(qPrime1);
                        %                         bool = (bofsE{i}.qPrime1i <= qPrime1) & (qPrime1 <=  bofsE{i}.qPrime1f);
                        plot(qPrime1, valv,'LineWidth',2,'Color','k');hold on
                    end
                else
                    figure(t)
                    bofs = obj.typeMap(types{t});
                    for d = 1:length(bofs)
                        bofd = bofs{d};
                        x = obj.parameters(:,d);
                        for i = 1:length(bofd)
                            qPrime1 = linspace(bofd{i}.qPrime1i, bofd{i}.qPrime1f, 200)';
                            m = bofd{i}.m;
                            valf = 0;
                            for j =1:length(m)
                                if ~isnumeric(m{j})
                                    fj = m{j};
                                    valf = valf+fj(qPrime1)*x(j);
                                end
                            end
                            %                             bool = (bofd{1}.qPrime1i <= qPrime1) & (qPrime1 <=  bofd{1}.qPrime1f);
                            %                             plot(qPrime1(valf~=0 & bool),valf(valf~=0 & bool),'LineWidth',2,'Color','b');hold on
                            plot(qPrime1, valf,'LineWidth',2,'Color','b');hold on
                            xlabel('$$q''_1 $$','Interpreter','latex','FontSize',16)
                            axis tight
                            set(gcf,'Color','w')
                        end
                    end
                    title(types{t})
                end
            end
        end
    end
end

