function V = getV(Q)

Qd = diff(Q,2);
sum(Qd,2)

[~, ~, V] = svd(Q');
Qhat = inv(V)*Q;
if Qhat(1,1) > Qhat(1,end) % flip the eiganvectors if the first one is in the negative direction
    V = -V;
end
end