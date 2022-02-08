%% train models
close all
numDims = 6;
numDemos = 4;
side = 'R0';

exercises = {'forwardRaise', 'lateralRaise', 'shoulderPress', 'internalExternalRotation'};

phases = {'up', 'down'};
for e = 4
    for p = 2:length(phases)
        phase = phases{p};
        [Demos, Demosd, Demosdd, T,Qi, Qf, Qdi, Qdf, minqPrime1, maxqPrime1,V] = getDemos(e, [1:numDemos], side, phase);
%             [Demos,Demosd, Demosdd, T,Qi,Qf,Qdi,Qdf] = chunkDemos(2, Demos, Demosd, Demosdd, T,Qi,Qf,Qdi,Qdf);
        %      [Demos,Demosd, Demosdd, T,Qi,Qf,Qdi,Qdf] = chunkDemos(2, Demos, Demosd, Demosdd, T,Qi,Qf,Qdi,Qdf);

        numCuts = 3;
        posTarget = [-2 : .2: 2]; % values for position target functions
        dirTarget = [-1 :.5 :1]; % values for direction target functions
        curvTarget = []; % values for curvature target functions
        effortTarget = []; % values for effort target functions
        KETarget = [0: .2: 2]; % values for kinetic energy target functions
        psm = PSM(numCuts, minqPrime1, maxqPrime1, numDims, V);

        basisConfig = struct( ...
            'posTarget', {2:numDims, posTarget},...
            'dirTarget', {2:numDims, dirTarget},...
            'curvTarget', {2:numDims, curvTarget},...
            'effortTarget', {1, effortTarget},...
            'KETarget', {1, KETarget});


        psm.setupBasisFncs(basisConfig, Qi, Qf);

        boundaryConfig = struct( ...
            'startPos', false,...
            'endPos', false,...
            'startDir', false,...
            'endDir', false,...
            'startKE', false, ...
            'endKE', false);

        psm.setupBoundaryConds(boundaryConfig, Qi, Qdi,Qf,Qdf)
        psm.setupBasisFncsE(basisConfig, T, Demos, Demosd, Demosdd)
        psm.buildOptimize();
        psm.W = 1*ones(length(psm.W),1);
        psm.optimize();
        psm.optimizePSM();
        close all
        %     qPrimei = psm.cp(1);
        %     qPrimef = psm.cp(end);
        psm.plotBasisFcns();
        psm.save(['PSMIOC/models/psm_' exercises{e} '_' phase '_' side]);
    end
end
%% Experiment 1: exercise reproduction
close all
for bool = [0]
    % bool = 0: without velocity limit
    % bool = 0: with velocity limit
    for e = 4 % forward raise
        psm = PSM.load(['PSMIOC/models/psm_'  exercises{e} '_' phase '_' side]);
        [Demos, Demosd, Demosdd,T,Qi,Qf,Qdi,Qdf,minqPrime1,maxqPrime1,V] = getDemos(e, [1:numDemos], side, phase);
        figure(66+bool)
        for d2 = 1:length(Demos)
            D = Demos{d2};
            t = T{d2};
            for d = 1:psm.numDims
                subplot(2,3,d)
                plot(t ,D(d,:),'Color','k','LineWidth',1);hold on
            end
        end
        for d3 = 1:numDemos
            psm = PSM.load(['PSMIOC/models/psm_'  exercises{e} '_' phase '_' side]);
            if bool
                psm.velLimit = 1;
            end
            W = psm.W;
            [Demos, Demosd, Demosdd,T,Qi,Qf,Qdi,Qdf,minqPrime1,maxqPrime1,V] = getDemos(e, d3, side, phase);
            psm.setupBasisFncsE(psm.basisConfig, T, Demos, Demosd, Demosdd)
            %             Qi{1} = Qi{1}+.1;
            %             Qi{1+3} = Qi{1+3}+.1;
            qpi = Qi;
            qdi = Qdi;
            qpf = Qf;
            qdf = Qdf;
            Qf = qpf;
            Qdf = qdf;
            psm.setupBasisFncs(psm.basisConfig, Qi, Qf)
            psm.W = W;
            for i = 0
                val = inv(psm.V)*[Qi{:}]';
                for j = 2:length(Qi)
                    val(j) = val(j)+i;
                end
                val = psm.V*val;
                for j = 1:length(Qi)
                    qpi{j} = val(j);
                end
                boundaryConfig = struct( ...
                    'startPos', false,...
                    'endPos', false,...
                    'startDir', false,...
                    'endDir', false,...
                    'startKE', true, ...
                    'endKE', true);
                psm.setupBoundaryConds(boundaryConfig, qpi, qdi,qpf,qdf)

                for j = 1:length(qpi)
                    psm.q(j) = qpi{j};
                    psm.qd(j) = qdi{j};
                end

                psm.buildOptimize();
                psm.optimizePSM();
                qPrimei = psm.qPrime1i;
                qPrimef = psm.qPrime1f;
                q = [];
                qd = [];
                dt = 0.01;
                invV = inv(psm.V);
                while invV(1,:)*psm.q < psm.qPrime1f
                    psm.step(dt)
                    q = [q;psm.q'];
                    qd = [qd;psm.qd'];
                end
                figure(66+bool)
                for d = 1:psm.numDims
                    subplot(2,3,d)
                    plot((0:length(q(:,d))-1)*dt ,q(:,d),'Color','b','LineWidth',2);hold on
                    axis tight
                end
            end
        end
        set(gcf,'Color','w')
    end
end
%% Experiment 2: patient evaluation
% close all
% table = zeros(4,8,3);
% for e = 1:4
%     load(['models/psm_' num2str(e)])
%     for j = [0 4]
%         for i = (4:6)
%             psmTest = PSM.load(['models/psm_' num2str(e)]);
%             W = psmTest.W;
%             psmTest.W = [];
%             [Demos,T,Qi,Qf,Qdi,Qdf,minqPrime1,maxqPrime1,V] = getDemos(e+j, i,numDims,1);
%             psmTest.setupBasisFncs(posTarget, dirTarget, curvTarget, effortTarget, KETarget, Qi,Qf)
%             Qf = cell(size(Qf,1),size(Qf,2));
%             Qdf = cell(size(Qdf,1),size(Qdf,2));
%             psmTest.setupBoundaryConds(Qi, Qdi,Qf,Qdf)
%             psmTest.setupBasisFncsE(posTarget, dirTarget, curvTarget, effortTarget, KETarget, T, Demos)
%             psmTest.W = W;
%             qPrimei = cp(1);
%             qPrimef = cp(end);
%             psmTest.buildOptimize();
%             psmTest.optimizePSM();
%             if j == 4 && e == 2 && i==4
%                 psmTest.plotBasisFcns(minqPrime1,maxqPrime1);
%                 error = psmTest.evaluate('plot')
%             else
%                 error = psmTest.evaluate()
%             end
%             table(e,e+j,i-3) = error;
%         end
%     end
% end
% figure
% plotTable(log(table*10))
% set(gcf,'Color','w')
% sum(mean(table,3),1)