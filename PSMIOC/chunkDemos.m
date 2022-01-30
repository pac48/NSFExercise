function [DemosNew, DemosdNew, DemosddNew, TNew,QiNew,QfNew,QdiNew,QdfNew] = chunkDemos(numChuncks, Demos, Demosd, Demosdd, T,Qi,Qf,Qdi,Qdf)
DemosNew = Demos;
DemosdNew = Demosd;
DemosddNew = Demosdd;
TNew = T;
QiNew = Qi;
QfNew = Qf;
QdiNew = Qdi;
QdfNew = Qdf;
for d =1:length(Demos)
    demo = Demos{d};
    demod = Demosd{d};
    demodd = Demosdd{d};
    t = T{d};
    inds = floor(linspace(1, size(demo,2), numChuncks+1));
    for i = 1:numChuncks
        TNew = cat(2, TNew, {t(inds(i):inds(i+1))});
        DemosNew = cat(2, DemosNew, {demo(:, inds(i):inds(i+1))});
        DemosdNew = cat(2, DemosdNew, {demod(:, inds(i):inds(i+1))});
        DemosddNew = cat(2, DemosddNew, {demodd(:, inds(i):inds(i+1))});
        QiNew = cat(2, QiNew, num2cell(demo(:, inds(i))));
        QdiNew = cat(2, QdiNew, num2cell(demod(:, inds(i))));
        QfNew = cat(2, QfNew, num2cell(demo(:, inds(i+1))));
        QdfNew = cat(2, QdfNew, num2cell(demod(:, inds(i+1))));
    end
end

end