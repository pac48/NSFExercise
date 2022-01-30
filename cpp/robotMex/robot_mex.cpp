//
// Created by paulg on 1/16/2022.
//

#define _USE_MATH_DEFINES
#include <cmath>
#include "mex.h"
#include "vector"
#include "rl/math/Transform.h"
#include "rl/math/Unit.h"
#include "rl/mdl/Dynamic.h"
#include "rl/mdl/Body.h"
#include <cstdio>
#include <string>
#include "rl/mdl/UrdfFactory.h"
#include "rl/mdl/Joint.h"

using namespace rl;
using namespace std;


const int INIT = 0;
const int DESTROY = 1;
const int SETJOINTS = 2;
const int GETJOINTS = 3;
const int GETJACOB = 4;
const int GETTRANS = 5;
const int GETOPPOS = 6;


mdl::Dynamic* getModelPtr(const mxArray* pa){
    double* pointer0 = mxGetPr(pa);
    mxAssert(pointer0 != NULL, "input must be a valid robot pointer\n");
    intptr_t pointer1 = (intptr_t) pointer0[0];
    return (mdl::Dynamic*) pointer1;
}

void setMatrixOutput(mxArray*& pa, const math::Matrix& M){
    pa = mxCreateDoubleMatrix(M.rows(), M.cols(), mxREAL);
    double* data_ptr = mxGetPr(pa);
    memcpy(data_ptr, M.data(), sizeof(double)*M.cols()*M.rows());
}

void setMatrixOutput(mxArray*& pa, const math::Transform& T){
    pa = mxCreateDoubleMatrix(4, 4, mxREAL);
    double* data_ptr = mxGetPr(pa);
    memcpy(data_ptr, T.data(), sizeof(double)*16);
}

void myAssert(int expression, const char *error_message){
    if (!expression){
        mexErrMsgTxt(error_message);
    }
}

void init(int nlhs, mxArray *plhs[],
          int nrhs, const mxArray *prhs[] ){

    myAssert(nrhs == 2, "Init function requires the name of the urdf file");
    myAssert(nlhs == 8, "expects 8 output argument to be assigned");

    char * filename =  mxArrayToString(prhs[1]);
    string name(filename);
    string urdf_file_name = name+".urdf";
    auto* model = new mdl::Model();
    rl::mdl::UrdfFactory urdf;
    urdf.load(urdf_file_name, model);
    mdl::Dynamic* dynamics = (mdl::Dynamic*) model;
    dynamics->forwardPosition();

    plhs[0] = mxCreateDoubleMatrix(1,1,mxREAL);
    double* robot_ptr = mxGetPr(plhs[0]);
    robot_ptr[0] = (intptr_t) dynamics;

    plhs[1] = mxCreateDoubleMatrix(1,1,mxREAL);
    double* numJoint = mxGetPr(plhs[1]);
    numJoint[0] = model->getJoints();

    plhs[2] = mxCreateDoubleMatrix(1,1,mxREAL);
    double* numBodies = mxGetPr(plhs[2]);
    numBodies[0] = dynamics->getBodies();

    mwSize numDims = 1;
    mwSize dims[1] = {(mwSize) model->getJoints()};


    plhs[3] = mxCreateCellArray(numDims, dims);
    for (int i = 0; i < model->getJoints(); i++){
        mxSetCell(plhs[3], i, mxCreateString(model->getJoint(i)->getName().c_str()));
    }

    dims[0] = dynamics->getBodies();
    plhs[4] = mxCreateCellArray(numDims, dims);
    for (int i = 0; i < model->getBodies(); i++){
        mxSetCell(plhs[4], i, mxCreateString(dynamics->getBody(i)->getName().c_str()));
    }

    plhs[5] = mxCreateDoubleMatrix(1,1,mxREAL);
    double* numEE = mxGetPr(plhs[5]);
    numEE[0] = dynamics->getOperationalDof();


    plhs[6] = mxCreateDoubleMatrix(model->getJoints(), 1, mxREAL);
    double* minJoints = mxGetPr(plhs[6]);
    for (int i = 0; i < model->getJoints(); i++){
        minJoints[i] = model->getJoint(i)->getMinimum()[0];
    }

    plhs[7] = mxCreateDoubleMatrix(model->getJoints(), 1, mxREAL);
    double* maxJoints = mxGetPr(plhs[7]);
    for (int i = 0; i < model->getJoints(); i++){
        maxJoints[i] = model->getJoint(i)->getMaximum()[0];
    }

}

void setJoints(int nlhs, mxArray *plhs[],
               int nrhs, const mxArray *prhs[]){

    myAssert(nrhs == 3, "three input argument expected");
    myAssert(nlhs == 0, "no output arguments expected");
    mdl::Dynamic* model = getModelPtr(prhs[1]);

    double* jointAngle = (double*) mxGetData(prhs[2]);
    const mwSize* dims = mxGetDimensions(prhs[2]);
    mwSize numDims = mxGetNumberOfDimensions(prhs[2]);

    auto curJoint = model->getPosition();
//    printf("number of dims is %d and value is %d", numDims, dims[0]);
    myAssert(numDims == 2 && dims[0] == curJoint.size(), "number of joints must be equal to length of input array");

    for (int i =0; i < curJoint.size(); i++){
        curJoint[i] = jointAngle[i];
    }
    model->setPosition(curJoint);
    model->forwardPosition();
}

void getJoints(int nlhs, mxArray *plhs[],
               int nrhs, const mxArray *prhs[]){

    myAssert(nrhs == 2, "two input argument expected");
    myAssert(nlhs == 1, "one output arguments expected");
    mdl::Dynamic* model = getModelPtr(prhs[1]);

    auto curJoint = model->getPosition();
    plhs[0] = mxCreateDoubleMatrix(curJoint.size(), 1, mxREAL);
    double* joints_ptr = mxGetPr(plhs[0]);

    for (int i =0; i < curJoint.size(); i++){
        joints_ptr[i] = curJoint[i];
    }

}

void getOperationalPosition(int nlhs, mxArray *plhs[],
               int nrhs, const mxArray *prhs[]){

    myAssert(nrhs == 3, "two input argument expected");
    myAssert(nlhs == 1, "one output arguments expected");
    mdl::Dynamic* model = getModelPtr(prhs[1]);

    int index = mxGetScalar(prhs[2])-1;
    myAssert(index < model->getOperationalDof() && index >= 0, "index out of bounds");
    rl::math::Transform T = model->getOperationalPosition(index);

    setMatrixOutput(plhs[0], T);

}

void getJacobian(int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[]){
    myAssert(nrhs == 2, "two input argument expected");
    myAssert(nlhs == 1, "one output arguments expected");
    mdl::Dynamic* model = getModelPtr(prhs[1]);
    int numEE = model->getOperationalDof();
    int numDof = model->getDof();
    math::Matrix J = math::Matrix(6*numEE, numDof);
    model->calculateJacobian(J);

    setMatrixOutput(plhs[0], J);
}

void getTransform(int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[]){
    myAssert(nrhs == 3, "two input argument expected");
    myAssert(nlhs == 1, "one output arguments expected");
    mdl::Dynamic* model = getModelPtr(prhs[1]);
    int index = mxGetScalar(prhs[2])-1;

    myAssert(index < model->getBodies() && index >= 0, "index out of bounds");
    setMatrixOutput(plhs[0], model->getBody(index)->t);

}

void destroy(int nlhs, mxArray *plhs[],
          int nrhs, const mxArray *prhs[] ){
    myAssert(nrhs == 2, "Only two input argument expected");
    myAssert(nlhs == 0, "no output arguments expected");
    mdl::Dynamic* model = getModelPtr(prhs[1]);
    printf("deleted robot object\n");

    delete model;
}

void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[] ) {


    myAssert(nrhs >= 1, "input must specify a function using an int");

    int val = mxGetScalar(prhs[0]);
    switch (val) {
        case INIT: init(nlhs, plhs, nrhs, prhs); break;
        case DESTROY: destroy(nlhs, plhs, nrhs, prhs); break;
        case SETJOINTS: setJoints(nlhs, plhs, nrhs, prhs); break;
        case GETJOINTS: getJoints(nlhs, plhs, nrhs, prhs); break;
        case GETJACOB: getJacobian(nlhs, plhs, nrhs, prhs); break;
        case GETTRANS: getTransform(nlhs, plhs, nrhs, prhs); break;
        case GETOPPOS: getOperationalPosition(nlhs, plhs, nrhs, prhs); break;
        default: break;
    }

}
