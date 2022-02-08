function Uall = getV2(Q, Qcell)
% Qd = 0;
Qd  = [];
for i = 1:length(Qcell)
    Qd = cat(2, Qd, diff(Qcell{i},1)' );
end
% Qd = 

[~, ~, V] = svd(Q');
% Uall = V;
% return


S = (Q*Q')./size(Q,1)^2;

QStart = zeros(length(Qcell), size(Q,1));
QEnd = zeros(length(Qcell), size(Q,1));
for i =1:length(Qcell)
  QStart(i,:) =  Qcell{i}(1,:);
  QEnd(i,:) =  Qcell{i}(end,:);
end

% obj = @(u) -(u'*S*u) -100*sum(u'*Qd);


% A = [];
% b = [];
%     u0 = V(:,1);
% bestU = [];
% bestVal = 999999999;
% 
% for it = 0 %1:10
% 
%     u0 = fmincon(obj, u0, A, b,[],[],[],[],nonlcon);
%     
% %     u = u0./norm(u0);
%    
%     qd = u0'*Qd;
%     minInd = find(qd == min(qd));
%     if obj(u0) < bestVal %sum(qd > 0) > bestVal
% %          plot(u0'*Qd); hold on
%         bestVal = obj(u0);%sum(qd > 0);
%         bestU = u0;
%     end
% 
%     if qd(minInd) > 0
%         break
%     else
%         A = cat(1, A, -Qd(:, minInd)');
%         b = cat(1, b, -max(qd)/100);
%     end
% end


% bestU =  mean(QEnd - QStart)';
%  bestU = bestU./norm(bestU);

 bestU = [0 0 0 0 0 -1]';

obj = @(u)  -(u'*S*u);% + 100*sum(((QEnd*u - QStart*u) - 1).^2);

Uall = bestU';
for i = 2:size(Q,1)
    nonlcon = @(u) deal(0, [u'*u-1; Uall*u]);
     u0 = V(:,i);
    ui = fmincon(obj, u0, [],[],[],[],[],[],nonlcon);
    Uall = cat(1, Uall, ui');
end

% w = 
% X = [0 -w(3) w(2) ; w(3) 0 -w(1) ; -w(2) w(1) 0 ];
Uall = Uall';



% Qhat = inv(V)*Q;
% 
% if Qhat(1,1) > Qhat(1,end) % flip the eiganvectors if the first one is in the negative direction
%     V = -V;
% end

end