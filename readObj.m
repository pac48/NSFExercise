function obj = readObj(fname, connectionStruct)
% compile with
%   mex loadObj.cpp -R2018a

Utils.makeDirIfMissing(Directories.cachedDir);

cacheEnabled = false;
if ~cacheEnabled
    warning('cache disabled')
end

    function name = processName(name)
        if contains(name, '_M')
            ind = max(find(name =='_', 999));
            name = name(1:ind-1);
        end
    end

if Utils.findFile(Directories.cachedDir, [fname '.mat']) && cacheEnabled
    tmp = load([Directories.cachedDir '/' fname '.mat'], 'obj');
    obj = tmp.obj;
else

    [~, verticesPositions, verticesNormals, verticesTextureCoordinates, names]...
        = loadObj([Directories.objsDir '/' fname '.obj']);
    names = cellfun(@processName, names,'UniformOutput',false);
    obj = kinematics.object(fname);
    fNames = fieldnames(connectionStruct);
    for i = 1:length(fNames)
        fName = fNames{i};
        body = kinematics.createEmptyBody();
        body.name = fName;
        if contains(body.name, 'pedestal')
            continue %  ignore this
        end
        j = find(strcmp(names, fName), 1);
        if ~isempty(j)
            v = verticesPositions{j};
            vt = verticesTextureCoordinates{j};
            vn = verticesNormals{j};
            inds = reshape(1:size(v,1), 3, []);
            inds = reshape(inds, [], 1);
            body.v = v(inds,:);
            body.vt = vt(inds,:);
            body.vn = vn(inds,:);
            body.color = ones(size(vn,1), 3);
        end
        body.parentTransform = connectionStruct.(body.name).parentTransform;
        body.parentName = connectionStruct.(body.name).parentName;
        body.transformType = connectionStruct.(body.name).transformType;
        body.transformParameterization = connectionStruct.(body.name).transformParameterization;
        body.colorType = connectionStruct.(body.name).colorType;
        body.colorParameterization = connectionStruct.(body.name).colorParameterization;
            
        obj.addBody(body, strcmp(body.transformType, 'freeXYZ'));
    end
    obj.buildTree()
    save([Directories.cachedDir '/' fname '.mat'], 'obj')
end

end

