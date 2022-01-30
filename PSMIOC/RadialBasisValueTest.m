close all
q = -4:0.001:3;
width = 1;
x = -2;


val = PSM.radialBasis(q, x, width) + PSM.radialBasis(q, x+1, width)+PSM.radialBasis(q, x+2, width);
valI = PSM.radialBasisI(q, q(1), x, width) + PSM.radialBasisI(q, q(1), x+1, width)+PSM.radialBasisI(q, q(1), x+2, width);
valD = PSM.radialBasisD(q, x, width) + PSM.radialBasisD(q, x+1, width)+PSM.radialBasisD(q, x+2, width);
valDD = PSM.radialBasisDD(q, x, width) + PSM.radialBasisDD(q, x+1, width)+PSM.radialBasisDD(q, x+2, width);


plot(val); hold on
plot(valI)
plot(cumtrapz(q, val) ,'LineStyle','--');


plot(diff(val)./(q(2)-q(1)) ,'LineStyle', '--'); hold on
plot(valD);


plot(diff(diff(val))./(q(2)-q(1))^2 ,'LineStyle', '--'); hold on
plot(valDD);