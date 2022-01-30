function [t,D,Dd, Ddd,DPrime,DdPrime,DddPrime, dt,V] = loadData(e, r, Vin)
% e 1-4 corresponds to correct exercises and 5-8 corresponds to incorrect
% exercises. Pre-processing steps occur in this function.
% offset = 0;
% if (e==4 || e==8)
%     offset = 2;
% end
% if e<=4
%     load(['data/TPR Therapist Weights000' num2str(e+offset) 'Q.mat'])
% else
%     load(['data/TPR Therapist Weights Bad Compensation000' num2str(e-4+offset) 'Q.mat'])
% end'

exercises = {'forwardRaise', 'lateralRaise', 'shoulderPress', 'internalExternalRotation'};

dataDirs = Utils.getAllFileNames('data','contain','.');
AllFiles = {};
for i = 1:length(dataDirs)
    files = Utils.getAllFileNames(dataDirs{i}, 'filter', @(x) ~(contains(x,'.mat') && contains(x,exercises{e})) );
    AllFiles = cat(1, AllFiles, files);
end




% % figure(5 + floor(1000*rand(1)))
% % pause(.1)
% % robot = Robot('yumi');
% % robot.trajectoryPlayback(D')

% get V
D1 = [];
for i = 1:length(AllFiles)
    load(AllFiles{i})
    D1 = cat(2, D1, D');
end
D = D';
D1 = D1(1:4, :);
D = D(1:4, :);
if exist('Vin') ~= 1
    %     V = getV(D1-D1(:,end));%; % this is the eigenvector matrix for a signle demonstration
    V = getV(D1-mean(D1,2));%; % this is the eigenvector matrix for a signle demonstration
else
    V = Vin;
end
% V = eye(4);

load(AllFiles{r})
D = D';
dt = 1/300;
time = size(D,2)*dt;
dataLength = 500;
dt = time/dataLength;

D = D(1:4, :);
t = (1:size(D,2))*dt;
figure(102)
for s =1:4
    subplot(2,2,s)
    plot(t,D(s,:), "Color",'k');hold on
end
Dd = zeros(size(D));
Ddd =  zeros(size(D));
for i =1:size(D,1)
    Dd(i,:) = d_dt(D(i,:),dt);
    Ddd(i,:) = d_dt(Dd(i,:),dt);
end
DPrime = inv(V)*D;
DdPrime = inv(V)*Dd;
DddPrime = inv(V)*Ddd;

% why
numBasis = 30;
width = (t(end) - t(1))/1;
X = linspace(t(1), t(end), numBasis);
A = zeros(size(DPrime,2), numBasis);
AI = zeros(size(DPrime,2), numBasis);
AD = zeros(size(DPrime,2), numBasis);
for i = 1:numBasis
    AI(:, i) = PSM.radialBasisI(t, 0, X(i), width);
end
for i = 1:numBasis
    A(:, i) = PSM.radialBasis(t, X(i), width);
end
for i = 1:numBasis
    AD(:, i) = PSM.radialBasisD(t, X(i), width);
end

for i = 1:numBasis
    A(:, i) = PSM.radialBasis(t, X(i), width);
end

minVel = 2;
Ain = [];
bin = [];
Aeq = zeros(3, numBasis);
for i = 1:numBasis
    Aeq(1, i) = PSM.radialBasis(t(end), X(i), width);
    Aeq(2, i) = PSM.radialBasis(t(end), X(i), width);
    Aeq(3, i) = PSM.radialBasisI(t(end), 0, X(i), width);
end
beq = [minVel;minVel;DPrime(1,end)-DPrime(1,1)];

while 1
    w = lsqlin(A, DdPrime(1,:), Ain, bin, Aeq, beq);
    DdPrime1 = A*w;
    minInd = find(DdPrime1 == min(DdPrime1), 1);
    if (DdPrime1(minInd) > minVel - 0.05)
        break;
    else
        bin = cat(1, bin, -minVel);
        row = zeros(1, numBasis);
        for i = 1:numBasis
            row(i) = PSM.radialBasis(t(minInd), X(i), width);
        end
        Ain = cat(1, Ain, -row);
    end

end
DPrime1 = AI*w;
DPrime1 = DPrime1-DPrime1(1) + DPrime(1,1);

% figure
% plot(DdPrime(1,:)); hold on
% plot(DdPrime1);
% figure
% plot(DPrime(1,:)); hold on
% plot(DPrime1);

DPrime(1,:) = DPrime1;
DdPrime(1,:) = DdPrime1;
DddPrime(1,:) = AD*w;

width = (DPrime(1,end) - DPrime(1,1))/1;
X = linspace(DPrime(1,1), DPrime(1,end), numBasis);
A = zeros(size(DPrime,2), numBasis);
AD = zeros(size(DPrime,2), numBasis);
ADD = zeros(size(DPrime,2), numBasis);
DPrime1Lin = linspace(DPrime1(1), DPrime1(end), length(DPrime1));
% DdPrime1Lin = interp1(DPrime1, DdPrime(1,:), DPrime1Lin);
for i = 1:numBasis
    A(:, i) = PSM.radialBasis(t, X(i), width);
end
for i = 1:numBasis
    AD(:, i) = PSM.radialBasisD(t, X(i), width);
end
for i = 1:numBasis
    ADD(:, i) = PSM.radialBasisDD(t, X(i), width);
end


for d = 2:size(DPrime,1)
    Aeq = zeros(2, numBasis);
    for i = 1:numBasis
        Aeq(1, i) = PSM.radialBasis(t(1), X(i), width);
        Aeq(2, i) = PSM.radialBasis(t(end), X(i), width);
        Aeq(3, i) = PSM.radialBasisD(t(1), X(i), width);
        Aeq(4, i) = PSM.radialBasisD(t(end), X(i), width);
    end
    beq = [DPrime(d,1); DPrime(d,end);0;0];

%     DPrime_target = interp1(DPrime1, DPrime(d,:), DPrime1Lin');
    DPrime_target = DPrime(d,:)';
%     DdPrime_target = DdPrime(d,:)./DdPrime(1,:);
    % figure
    % plot(DPrime1Lin, DPrime_target); hold on
    % plot(DPrime1, DPrime(d,:))
    w = lsqlin(cat(1, A, 0*ADD, 0*AD), cat(1, DPrime_target, 0*DPrime(d,:)', 0*DPrime(d,:)'), [], [], Aeq, beq);
    DPrime_d = A*w;
    DdPrime_d = AD*w;
    DddPrime_d = ADD*w;

%         figure
%         plot(t, DPrime_d); hold on
%         plot(t, DPrime(d,:))
% 
%         figure
%         plot(t, DdPrime_d) ; hold on
%         plot(t, DdPrime(d,:))
% 
%          figure
%         plot(t, DdPrime_d./DdPrime(1,:)'); hold on
%         plot(t, DdPrime(d,:)./DdPrime(1,:)) 
        

    DPrime_t  = interp1(DPrime1Lin, DPrime_d , DPrime1);
    DdPrime_t  = interp1(DPrime1Lin, DdPrime_d , DPrime1);
    DddPrime_t  = interp1(DPrime1Lin, DddPrime_d , DPrime1);

     

    DPrime(d,:) = DPrime_d;
    DdPrime(d,:) = DdPrime_d;
    DddPrime(d,:) = DddPrime_d;
end

% for i =1:size(D,1)
%     DdPrime(i,:) = d_dt(DPrime(i,:),dt);
%     DddPrime(i,:) = d_dt(DdPrime(i,:),dt);
% end

% figure(1)
% plot(D'); hold on
D = V*DPrime;
% plot(D');
Dd = V*DdPrime;
Ddd = V*DddPrime;
%
%
% n = length(t);
% h = .06;
% for i = 1:size(DPrime,1)
%     ri = ksr(t,DPrime(i,:),h,n);
% end
% D = V*DPrime;
% Dd = 0*D;
% Ddd = 0*D;
% DdPrime = 0*DPrime;
% DddPrime = 0*DPrime;
% for i =1:size(D,1)
%     Dd(i,:) = d_dt(D(i,:),dt);
%     DdPrime(i,:) = d_dt(DPrime(i,:),dt);
%     Ddd(i,:) = d_dt(Dd(i,:),dt);
%     DddPrime(i,:) = d_dt(DdPrime(i,:),dt);
% end
% if DPrime(1,end)-DPrime(1,1)<0
%     DPrime(1,:) = -DPrime(1,:);
% end
% for i = 1:size(D,1)
%     DdPrime(i,:) = d_dt(DPrime(i,:),dt);
%     DddPrime(i,:) = d_dt(DdPrime(i,:),dt);
% end
%
%
% % return
%
% offset = floor(size(DdPrime,2)/2);
% % crop motion once velocity falls below a threshold
% ind2 = offset+find( (.5*DdPrime(1,offset:end).^2) < 0.1,1)-1;
% ind1 = find(.5*DdPrime(1,1:end).^2 > 0.1,1);
% if isempty(ind1) || true
%     ind1 = 2;
% end
% if isempty(ind2) || true
%     ind2 = size(DPrime,2)-1;
% end
%
% tmp = ind1:ind2;
% DPrime = DPrime(:,tmp);
% DdPrime = DdPrime(:,tmp);
% DddPrime = DddPrime(:,tmp);
% D = D(:,tmp);
% Dd = Dd(:,tmp);
% Ddd = Ddd(:,tmp);
% t = t(:,tmp);
% t = t-t(1);
% n = length(t);
% h = .05;
% ri = ksr(t,DPrime(1,:),h,n);
% DPrime(1,:) = ri.f;% smooth(DPrime(i,:),.2); % smooth data
% h = .05;%.2;
% for i = 2:size(DPrime,1)
%     ri = ksr(DPrime(1,:),DPrime(i,:),h,n);
%     DPrime(i,:) = ri.f;% smooth(DPrime(i,:),.2); % smooth data
% end
% for i = 1:size(D,1)
%     DdPrime(i,:) = d_dt(DPrime(i,:),dt);
%     DddPrime(i,:) = d_dt(DdPrime(i,:),dt);
% end
% D = V*DPrime;
% Dd = V*DdPrime;
% Ddd = V*DddPrime;
% offset = floor(size(DdPrime,2)/2);
% % crop motion again once velocity falls below a threshold
% ind2 = offset+find( (.5*DdPrime(1,offset:end).^2) < 0.1,1)-1;
% ind1 = find(.5*DdPrime(1,1:end).^2 > 0.1,1);
% if isempty(ind1) || true
%     ind1 = 2;
% end
% if isempty(ind2) || true
%     ind2 = size(DPrime,2)-1;
% end
% tmp = ind1:ind2;
% DPrime = DPrime(:,tmp);
% DdPrime = DdPrime(:,tmp);
% DddPrime = DddPrime(:,tmp);
% D = D(:,tmp);
% Dd = Dd(:,tmp);
% Ddd = Ddd(:,tmp);
% t = t(:,tmp);
% t = t-t(1);
end