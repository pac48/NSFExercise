function [t,D,Dd, Ddd,DPrime,DdPrime,DddPrime, dt,V] = loadData(e, r, side, phase, Vin)
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
% side = 'R0';
% phase = 'up';
bodyInds =  [(5-1)*3 + (1:3) (7-1)*3+(1:3)];
numDims = 6;
dataDirs = Utils.getAllFileNames('data','contain','.');
allD = {};
for i = 1:length(dataDirs)
    Xfiles = Utils.getAllFileNames(dataDirs{i}, 'filter', @(x) ~(contains(x,'.mat') && contains(x, side) && contains(x, 'X') && ~contains(x,exercises{e})) );
    indFiles = Utils.getAllFileNames(dataDirs{i}, 'filter', @(x) ~(contains(x,'.mat') && contains(x, side) && contains(x,exercises{e})) );
    assert(length(Xfiles) >= length(indFiles), 'there are problems');
    for j = 1:length(indFiles)
        nameIndX = find(Xfiles{j} =='X', 1)-1;
        nameIndI = find(indFiles{j} =='Q', 1)-1;
        assert(strcmp(Xfiles{j}(1:nameIndX), indFiles{j}(1:nameIndI) ))
        tmp = load(Xfiles{j});
        X = tmp.X;
        tmp = load(indFiles{j});
        inds = tmp.inds;
        %         t = linspace(0, 1, size(X,1))';
        allD = cat(1, allD, X(inds, bodyInds) );
    end
end




% % figure(5 + floor(1000*rand(1)))
% % pause(.1)
% % robot = Robot('yumi');
% % robot.trajectoryPlayback(D')

% get V
directory = ['PSMIOC/inds/'];
Dall = [];
for i = 1:length(allD)
    if exist([directory exercises{e} '_' num2str(i) '_' phase '_' side '.mat']) == 0
        if strcmp(phase, 'up')
            disp(['please select the up start then up end for motion ' num2str(i) ])
        else
            disp(['please select the down start then down end for motion ' num2str(i) ])
        end
        plot(allD{i})
        [ind1, ~] = ginput(1);
        [ind2, ~] = ginput(1);
        ind1 = max(floor(ind1),1);
        ind2 = min(floor(ind2), size(allD{i},1));
        inds = ind1:ind2;
        save([directory exercises{e} '_' num2str(i) '_' phase '_' side '.mat'], 'inds')
    else
        tmp = load([directory exercises{e} '_' num2str(i) '_' phase '_' side '.mat']);
        inds = tmp.inds;
    end
    Dall = cat(2, Dall, allD{i}(inds,:)');
    %     for p = 1:6
    %         subplot(2,3,p)
    %         plot(allD{i}(inds, p)); hold on
    %     end
    allD{i} = allD{i}(inds,:);
end

% D1 = D1(1:4, :);
% D = D(1:4, :);
if exist('Vin') ~= 1
    %     V = getV(D1-D1(:,end));%; % this is the eigenvector matrix for a signle demonstration
    %     V = getV(Dall-mean(Dall,2));%; % this is the eigenvector matrix for a signle demonstration
% %     if exist(['PSMIOC/V/' exercises{e} '_V_' phase '.mat']) == 0 %|| true
        V = getV2(Dall, allD);
%         save(['PSMIOC/V/' exercises{e} '_V_' phase '.mat'],'V')
% %     else
% %         tmp = load(['PSMIOC/V/' exercises{e} '_V_' phase '.mat']);
% %         V = tmp.V;
% %     end
else
    V = Vin;
end
% V = eye(4);

% tmp = load([directory exercises{e} '_' num2str(r) '_' phase '.mat']);
% inds = tmp.inds;
D = allD{r}';%(inds, :)';
avgInds = floor(mean(cellfun(@(x) length(x), allD)));
Dnew = zeros(size(D,1), avgInds);
for i = 1:size(D,1)
    Dnew(i,:) = interp1(D(i, :), linspace(1, size(D,2), avgInds));
end
D = Dnew;
dt = 1/250;
t = (1:size(D,2))*dt;
figure(102)
for s =1:6
    subplot(2,3,s)
    plot(t, D(s,:), "Color",'k');hold on
end


DPrime = inv(V)*D;
DPrimeEnd = zeros(length(allD), size(D,1));
DPrimeStart = zeros(length(allD), size(D,1));
for i = 1:length(allD)
    DPrimei = inv(V)*(allD{i}');
    DPrimeEnd(i,:) = DPrimei(:,end);
    DPrimeStart(i,:) = DPrimei(:,1);
end
DPrimestartAvg = mean(DPrimeStart);
DPrimeendAvg = mean(DPrimeEnd);
start = DPrime(:,1);
DPrime = DPrime - DPrime(:, 1);
DPrime = DPrime./DPrime(1, end);
DPrime = DPrime.*(DPrimeendAvg(1) - DPrimestartAvg(1));
DPrime(1,:) = DPrime(1,:) + DPrimestartAvg(1);
DPrime(2:end,:) = DPrime(2:end,:) + start(2:end); 

DdPrime = 0*DPrime;
for i =1:size(DdPrime,1)
    DdPrime(i,:) = d_dt(DPrime(i,:),dt);
end

% D = V*DPrime;
% Dd = zeros(size(D));
% Ddd =  zeros(size(D));
% for i =1:size(D,1)
%     Dd(i,:) = d_dt(D(i,:),dt);
%     Ddd(i,:) = d_dt(Dd(i,:),dt);
% end


% DdPrime = inv(V)*Dd;
% DddPrime = inv(V)*Ddd;
% DPrime = DPrime(1:numDims, :);
% DdPrime = DdPrime(1:numDims, :);
% DddPrime = DddPrime(1:numDims, :);

% why
numBasis = 30;
% width = (t(end) - t(1))/1;
X = linspace(t(1)-1, t(end)+1, numBasis);
    function [AI, A, AD, ADD] = getAVals(width)
        A = zeros(size(DPrime,2), numBasis+1);
        AI = zeros(size(DPrime,2), numBasis+1);
        AD = zeros(size(DPrime,2), numBasis+1);
        ADD = zeros(size(DPrime,2), numBasis+1);
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
            ADD(:, i) = PSM.radialBasisDD(t, X(i), width);
        end

        A(:, end) = 1;
        AI(:, end) = t;
    end


[AI, A, AD, ADD] = getAVals((t(end) - t(1))/.1);
minVel = .1;
Ain = [];
bin = [];
Aeq = [A(1,:)
    A(end,:)
    %     AD(1,:)
    %     AD(end,:)
    ];

beq = [DPrime(1,1); DPrime(1,end)];

while 1
    w = lsqlin(A, DPrime(1,:), Ain, bin, Aeq, beq);
    DdPrime1 = AD*w;
    minInd = find(DdPrime1 == min(DdPrime1), 1);
    if (DdPrime1(minInd) > minVel - 0.01)
        break;
    else
        bin = cat(1, bin, -minVel);
        row = AD(minInd, :);
        Ain = cat(1, Ain, -row);
    end

end

DPrime(1,:) = A*w;
DdPrime(1,:) = AD*w;
DddPrime(1,:) = ADD*w;



[AI, A, AD, ADD] = getAVals((t(end) - t(1))/20);

Aeq = [A(1,:)
    A(end,:)
    %     AD(1,:)
    %     AD(end,:)
    ];

for d = 2:size(DPrime,1)


    beq = [DPrime(d,1); DPrime(d,end);
        %         0;0
        ];

%     DPrime_target = DPrime(d,:)';
    w = lsqlin(cat(1, A, .1*AD, .01*ADD), cat(1, DPrime(d,:)', .1*DdPrime(d,:)', 0*DdPrime(d,:)'), [], [], [], []);
    DPrime_d = A*w;
    DdPrime_d = AD*w;
    DddPrime_d = ADD*w;


% 
% %             figure
% %             plot(t, DPrime_d); hold on
% %             plot(t, DPrime(d,:)')
% %     %
    %         figure
    %         plot(t, DdPrime_d) ; hold on
    %         plot(t, DdPrime(d,:))
    %
    %          figure
    %         plot(t, DdPrime_d./DdPrime(1,:)'); hold on
    %         plot(t, DdPrime(d,:)./DdPrime(1,:))


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
D = V(:,1:numDims)*DPrime;
Dd = V(:,1:numDims)*DdPrime;
Ddd = V(:,1:numDims)*DddPrime;

end