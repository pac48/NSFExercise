function m = BasisFnc(index, functionsM)
% m1 = cell(length(index),1);
m = cell(length(index),1);
% J = @(qPrime1) BasisObjFnc.getJ(cp, qPrime1);
for i = 1:length(index)
%     m1{i} = 0;
    m{i} = 0;
    if index(i) ~= 0
%         m1{i} = functionsM{index(i)};
%         m{i} = @(qPrime1) m1{i}(qPrime1).*(J(qPrime1) == cut);
          m{i} = functionsM{index(i)};% @(qPrime1) m1{i}(qPrime1);
    end
end
end