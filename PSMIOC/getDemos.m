function [Demos,Demosd, Demosdd, T,Qi,Qf,Qdi,Qdf,minqPrime1,maxqPrime1,V] = getDemos(e, demos, numDims, numDemos)
Demos = {}; % joint trajectories for exercise e
Demosd = {}; % joint velocities for exercise e
Demosdd = {}; % joint accelerationsfor exercise e
T = {}; % times for exercise e
Qi = {}; % intial positons
Qf = {}; % final positions
Qdi = {}; % intial velocities
Qdf = {}; % final velocities
Qi{numDims,numDemos} = [];
Qf{numDims,numDemos} = [];
Qdi{numDims,numDemos} = [];
Qdf{numDims,numDemos} = [];
minqPrime1 = 999; % minimum q'_1 in datas
maxqPrime1 = -999; % maximum q'_1 in data
for d = 1:length(demos)
    [t, D, Dd, Ddd, Dprime,Ddprime,Dddprime, dt, V] = loadData(e,demos(d));
    if e > 4
        [~, ~, ~, ~, ~, ~, ~, ~, V] = loadData(e-4,demos(d));
        [t, D, Dd, Ddd, Dprime,Ddprime,Dddprime, dt, V] = loadData(e,demos(d),V);
    end
    if Dprime(1,1) < minqPrime1
        minqPrime1 =  Dprime(1,1);
    end
    if Dprime(1,end) > maxqPrime1
        maxqPrime1 =  Dprime(1,end);
    end

    Demos{d} = D;
    Demosd{d} = Dd;
    Demosdd{d} = Ddd;
    for i = 1:numDims
        Qi{i,d} = D(i,1);
        Qdi{i,d} = Dd(i,1);
        Qf{i, d} = D(i,end);
        Qdf{i,d} = Dd(i,end);
    end
    T{d} = t;
    %     figure(100)
    %     plot(demo(1,:),demo(2:end,:));hold on
    %     figure(101)
    %     plot(Dprime(1,:),Dprime(2:end,:));hold on
    %     figure(102)
    %     plot(Dprime(1,:),.5*Ddprime(1,:).^2);hold on
        figure(102)
        for s =1:4
            subplot(2,2,s)
            plot(t,D(s,:));hold on
        end
        figure(103)
        for s =1:4
            subplot(2,2,s)
            plot(Dprime(1,:), Dprime(s,:));hold on
        end
        figure(104)
        for s =1:4
            subplot(2,2,s)
            plot(Dprime(1,:), Ddprime(s,:)./Ddprime(1,:) );hold on
        end

%         why
% figure
% plot(t, d_dt(Dprime(1,:), dt)); hold on
% plot(t, Ddprime(1,:));
% 
% figure
% plot(t, d_dt(Dprime(2,:), dt)); hold on
% plot(t, Ddprime(2,:));

end
end