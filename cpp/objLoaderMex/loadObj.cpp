//
// Created by paulg on 11/17/2021.
//

#include "mex.h"
#include "OBJ_Loader.h"
#include "string"

using namespace std;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    int fileNameInd = 0;
    if (nrhs != 1) {
        mexErrMsgIdAndTxt("MATLAB:myComponent:inputError",
                          "File name must be the only input");
    }
    if (nlhs != 5) {
        mexErrMsgIdAndTxt("MATLAB:myComponent:inputError",
                          "must have 5 outputs");
    }
    if (!mxIsChar(prhs[fileNameInd])) {
        mexErrMsgIdAndTxt("MATLAB:mxcreatecharmatrixfromstr:invalidInputType",
                          "Input must be of type char.");
    }
    string fileName = mxArrayToString(prhs[fileNameInd]);

    // Initialize Loader
    objl::Loader Loader;

    // Load .obj File
    bool loadout = Loader.LoadFile(fileName);

    // Check to see if it loaded

    // If so continue
    if (!loadout) {
        mexErrMsgIdAndTxt("MATLAB:myComponent:inputError",
                          "Failed to load object");
    }
    int numMeshes = Loader.LoadedMeshes.size();
    plhs[0] = mxCreateCellMatrix(numMeshes, 1);
    plhs[1] = mxCreateCellMatrix(numMeshes, 1);
    plhs[2] = mxCreateCellMatrix(numMeshes, 1);
    plhs[3] = mxCreateCellMatrix(numMeshes, 1);
    plhs[4] = mxCreateCellMatrix(numMeshes, 1);
//    int totalNumVerts = 0;
//    int totalNumInds = 0;
//    for (int i = 0; i < numMeshes; i++) {
//        objl::Mesh curMesh = Loader.LoadedMeshes[i];
//        totalNumVerts += curMesh.Vertices.size();
//        totalNumInds += curMesh.Indices.size();
//    }
    for (int i = 0; i < numMeshes; i++) {
        objl::Mesh curMesh = Loader.LoadedMeshes[i];
        int numVerts = curMesh.Vertices.size();
        int numPoints = curMesh.Positions.size();
        int numInds = curMesh.Indices.size();
        // Go through each vertex and print its number,position, normal, and texture coordinate
        mxSingle *points = (mxSingle *) mxCalloc(3 * numPoints,
                                                            sizeof(mxSINGLE_CLASS));// new mxSingle[curMesh.Vertices.size()];
        mxSingle * verticesPositions = (mxSingle *) mxCalloc(3 * numVerts,
                                                            sizeof(mxSINGLE_CLASS));// new mxSingle[curMesh.Vertices.size()];
        mxSingle *verticesNormals = (mxSingle *) mxCalloc(3 * numVerts,
                                                          sizeof(mxSINGLE_CLASS));// new mxSingle[curMesh.Vertices.size()];
        mxSingle *verticesTextureCoordinates = (mxSingle *) mxCalloc(2 * numVerts,
                                                                     sizeof(mxSINGLE_CLASS));// new mxSingle[curMesh.Vertices.size()];;
        mxUint32 *faces = (mxUint32 *) mxCalloc(numInds,
                                                sizeof(mxUINT32_CLASS));// new mxSingle[curMesh.Vertices.size()];

        string objName = curMesh.MeshName;
        // Copy one of the loaded meshes to be our current mesh

        for (int j = 0; j < numPoints; j++) {
            points[j] = curMesh.Positions[j].X;
            points[j + numPoints] = curMesh.Positions[j].Y;
            points[j + 2 * numPoints] = curMesh.Positions[j].Z;
        }

        for (int j = 0; j < numVerts; j++) {
            verticesPositions[j] = curMesh.Vertices[j].Position.X;
            verticesPositions[j + numVerts] = curMesh.Vertices[j].Position.Y;
            verticesPositions[j + 2 * numVerts] = curMesh.Vertices[j].Position.Z;

            verticesNormals[j] = curMesh.Vertices[j].Normal.X;
            verticesNormals[j + numVerts] = curMesh.Vertices[j].Normal.Y;
            verticesNormals[j + 2 * numVerts] = curMesh.Vertices[j].Normal.Z;

            verticesTextureCoordinates[j] = curMesh.Vertices[j].TextureCoordinate.X;
            verticesTextureCoordinates[j + numVerts] = curMesh.Vertices[j].TextureCoordinate.Y;
        }
        // handle faces - all are triangles
        for (int j = 0; j < numInds; j += 3) {
            faces[j] = curMesh.Indices[j];
            faces[j + 1] = curMesh.Indices[j + 1];
            faces[j + 2] = curMesh.Indices[j + 2];
        }

        mxArray *tmp;
        tmp = mxCreateNumericMatrix(numPoints, 3, mxSINGLE_CLASS, mxREAL);
        mxSetData(tmp, points);
        mxSetCell(plhs[0], i, tmp);

        tmp = mxCreateNumericMatrix(numVerts, 3, mxSINGLE_CLASS, mxREAL);
        mxSetData(tmp, verticesPositions);
        mxSetCell(plhs[1], i, tmp);

        tmp = mxCreateNumericMatrix(numVerts, 3, mxSINGLE_CLASS, mxREAL);
        mxSetData(tmp, verticesNormals);
        mxSetCell(plhs[2], i, tmp);

        tmp = mxCreateNumericMatrix(numVerts, 2, mxSINGLE_CLASS, mxREAL);
        mxSetData(tmp, verticesTextureCoordinates);
        mxSetCell(plhs[3], i, tmp);

//        tmp = mxCreateNumericMatrix(3, numInds / 3, mxUINT32_CLASS, mxREAL);
//        mxSetUint32s(tmp, faces);
//        mxSetCell(plhs[3], i, tmp);

        mxSetCell(plhs[4], i, mxCreateString(objName.c_str()));

    }
}


//    plhs[0] = mxCreateNumericMatrix(totalNumVerts, 3, mxSINGLE_CLASS, mxREAL);
//    mxSetSingles(plhs[0], verticesPositions);
//
//    plhs[1] = mxCreateNumericMatrix(totalNumVerts, 3, mxSINGLE_CLASS, mxREAL);
//    mxSetSingles(plhs[1], verticesNormals);
//
//    plhs[2] = mxCreateNumericMatrix(totalNumVerts, 2, mxSINGLE_CLASS, mxREAL);
//    mxSetSingles(plhs[2], verticesTextureCoordinates);
//
//    plhs[3] = mxCreateNumericMatrix(3, totalNumInds/3, mxUINT32_CLASS, mxREAL);
//    mxSetUint32s(plhs[3], faces);
//
//
//    plhs[4] = mxCreateCellMatrix(numMeshes, 1);
//    for (int i=0; i < numMeshes; i++){
//        string name = objNames[i];
//        mxSetCell(plhs[4], i, mxCreateString(name.c_str()));

//    plhs[5] = mxCreateNumericMatrix(numMeshes, 1, mxUINT32_CLASS, mxREAL);
//    mxSetUint32s(plhs[5], vertEndInds);
////
//    plhs[6] = mxCreateNumericMatrix(numMeshes, 1, mxUINT32_CLASS, mxREAL);
//    mxSetUint32s(plhs[6], faceEndInds);

