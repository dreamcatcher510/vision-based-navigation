/*
Copyright 2011. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

This file is part of libviso2.
Authors: Andreas Geiger

libviso2 is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or any later version.

libviso2 is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libviso2; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA
*/

#include "visualodometry.h"
#include <fstream>

//#include "highgui.h"
//#include "cv.h"

using namespace std;

VisualOdometry::VisualOdometry() {
  J         = 0;
  p_observe = 0;
  p_predict = 0;
  calib_f   = 1;
  calib_cu  = 0;
  calib_cv  = 0;
  calib_b   = 1;
  calib_p   = 0;
  srand (2);
}

VisualOdometry::~VisualOdometry() {
}

void VisualOdometry::setCalibration(myFLOAT f,myFLOAT cu,myFLOAT cv,myFLOAT b,myFLOAT p) {
  calib_f  = f;
  calib_cu = cu;
  calib_cv = cv;
  calib_b  = b;
  calib_p  = p;
}

void VisualOdometry::setCalibration(myFLOAT f,myFLOAT cu,myFLOAT cv,myFLOAT b,myFLOAT p,
						myFLOAT fu_1, myFLOAT fv_1, myFLOAT cu_1,myFLOAT cv_1,
						myFLOAT fu_2, myFLOAT fv_2, myFLOAT cu_2,myFLOAT cv_2,
						Matrix extri)
{
	calib_f  = f;
	calib_cu = cu;
	calib_cv = cv;
	calib_b  = b;
	calib_p  = p;

	calib_fu_1 = fu_1;
	calib_fv_1 = fv_1;
	calib_cu_1 = cu_1;
	calib_cv_1 = cv_1;
    calib_fu_2 = fu_2;
	calib_fv_2 = fv_2;
	calib_cu_2 = cu_2;
	calib_cv_2 = cv_2;
	//extrinsic = Matrix::Matrix(extri) ;
	extrinsic = extri;
}

bool VisualOdometry::update(vector<Matcher::p_match> p_matched,
                            myFLOAT deltaT,bool stereo,bool record_raw_odometry) {

  // return value
  bool success;

  // estimate egomotion from feature points
  if (stereo) {
    success = processStereo(p_matched,deltaT);
  } else {
    success = processMono(p_matched,deltaT);
    if (!success) // no success => not moving!
      return false;
  }

  // use this to avoid roll
  //param[2] = 0;

  //////////////////////
  // kalman filtering //
  //////////////////////

  // measurement
  if (success) KF_z = Matrix(6,1,param)/deltaT;
  else         KF_z = Matrix(6,1);

  // state transition matrix
  KF_A = Matrix::eye(12);
  for (int32_t i=0; i<6; i++)
    KF_A.val[i][i+6] = deltaT;

  // observation matrix
  KF_H = Matrix(6,12);
  for (int32_t i=0; i<6; i++)
    KF_H.val[i][i] = 1;

  // process noise
  KF_Q = Matrix::eye(12);
  KF_Q.setDiag(1e-9,0,2);
  KF_Q.setDiag(1e-8,3,5);
  KF_Q.setDiag(1e-0,6,8);
  KF_Q.setDiag(1e-0,9,11);

  // measurement noise
  KF_R = Matrix::eye(6);
  KF_R.setDiag(1e-2,0,2);
  KF_R.setDiag(1e-1,3,5);

  // do not rely on measurements if estimation went wrong
  if (!success)
    KF_R = KF_R*1e6;

  // first iteration
  if (KF_x.m==0) {

    // init state x and state covariance P
    KF_x = Matrix(12,1);
    KF_x.setMat(KF_z,0,0);
    KF_P = Matrix::eye(12);

  // other iterations
  } else {

    // prediction
    KF_x = KF_A*KF_x;
    KF_P = KF_A*KF_P*(~KF_A)+KF_Q;

    // kalman gain
    Matrix K = KF_P*(~KF_H)*Matrix::inv(KF_H*KF_P*(~KF_H)+KF_R);

    // correction
    KF_x = KF_x + K*(KF_z-KF_H*KF_x);
    KF_P = KF_P - K*KF_H*KF_P;
  }

  // re-set parameter vector
  (KF_x*deltaT).getData(param,0,0,5,0);

  // write raw odometry measurements (velocities) to file
  if (record_raw_odometry && success) {
    ofstream file;
    file.open("/home/geiger/odometry.txt",ios::out|ios::app);
    file << ~KF_z << endl;
    file.close();
  }

  // return true
  return true;
}

bool VisualOdometry::processMono(vector<Matcher::p_match> p_matched,myFLOAT deltaT) {

  // get number of matches
  int32_t N = p_matched.size();
  if (N<10)
    return false;

  // create calibration matrix
  myFLOAT K_data[9] = {calib_f,0,calib_cu,0,calib_f,calib_cv,0,0,1};
  Matrix K(3,3,K_data);

  // normalize feature points
  Matrix Tp,Tc;
  vector<Matcher::p_match> p_matched_normalized = p_matched;
  if (!monoNormalizeFeaturePoints(p_matched_normalized,Tp,Tc))
    return false;

  /////////////////////////////////////////////////////////////
  // F: initial RANSAC estimate
  Matrix E,F;
  inliers.clear();
  for (int32_t k=0;k<2000;k++) {

    // draw random sample set
    vector<int32_t> active = monoGetRandomSample(N,8);

    // estimate fundamental matrix and get inliers
    monoFundamentalMatrix(p_matched_normalized,active,F);
    vector<int32_t> inliers_curr = monoGetFundamentalInlier(p_matched_normalized,F,0.00001);

    // update model if we are better
    if (inliers_curr.size()>inliers.size())
      inliers = inliers_curr;
  }

  if (inliers.size()<10)
    return false;

  // F: final optimization (refinement)
  monoFundamentalMatrix(p_matched_normalized,inliers,F);

  // denormalise and extract essential matrix
  F = ~Tc*F*Tp;
  E = ~K*F*K;

  // compute 3d points X and R|t up to scale
  Matrix X,R,t;
  monoEtoRt(E,K,p_matched,inliers,X,R,t);

  // normalize 3d points and remove points behind image plane
  X = X/X.getMat(3,0,3,-1);
  vector<int32_t> pos_idx;
  for (int32_t i=0; i<X.n; i++)
    if (X.val[2][i]>0)
      pos_idx.push_back(i);
  Matrix X_plane = X.extractCols(pos_idx);

  // we need at least 10 points to proceed
  if (X_plane.n<10)
    return false;

  // get elements closer than median element
  myFLOAT median;
  smallerThanMedian(X_plane,median);

  // return on large median (litte motion)
  if (median>100)
    return false;

  // project features to 2d
  Matrix x_plane(2,X_plane.n);
  x_plane.setMat(X_plane.getMat(1,0,2,-1),0,0);

  Matrix n(2,1);
  myFLOAT pitch      = calib_p;
  n.val[0][0]      = cos(-pitch);
  n.val[1][0]      = sin(-pitch);
  Matrix  d        = ~n*x_plane;
  myFLOAT   sigma    = median/50.0;
  myFLOAT   weight   = 1.0/(2.0*sigma*sigma);
  myFLOAT   best_sum = 0;
  int32_t best_idx = 0;

  // find best plane
  for (int32_t i=0; i<x_plane.n; i++) {
    if (d.val[0][i]>median/100.0) {
      myFLOAT sum = 0;
      for (int32_t j=0; j<x_plane.n; j++) {
        myFLOAT dist = d.val[0][j]-d.val[0][i];
        sum += exp(-dist*dist*weight);
      }
      if (sum>best_sum) {
        best_sum = sum;
        best_idx = i;
      }
    }
  }
  t = t*calib_b/d.val[0][best_idx];

  // compute rotation angles
  myFLOAT ry = asin(R.val[0][2]);
  myFLOAT rx = asin(-R.val[1][2]/cos(ry));
  myFLOAT rz = asin(-R.val[0][1]/cos(ry));

  // set parameter vector
  param[0] = rx;
  param[1] = ry;
  param[2] = rz;
  param[3] = t.val[0][0];
  param[4] = t.val[1][0];
  param[5] = t.val[2][0];

  // parameter estimate succeeded?
  return true;
}

Matrix VisualOdometry::smallerThanMedian (Matrix &X,myFLOAT &median) {

  // set distance and index vector
  vector<myFLOAT> dist;
  vector<int32_t> idx;
  for (int32_t i=0; i<X.n; i++) {
    dist.push_back(fabs(X.val[0][i])+fabs(X.val[1][i])+fabs(X.val[2][i]));
    idx.push_back(i);
  }

  // sort elements
  sort(idx.begin(),idx.end(),idx_cmp<vector<myFLOAT>&>(dist));

  // get median
  int32_t num_elem_half = idx.size()/2;
  median = dist[idx[num_elem_half]];

  // create matrix containing elements closer than median
  Matrix X_small(4,num_elem_half+1);
  for (int32_t j=0; j<=num_elem_half; j++)
    for (int32_t i=0; i<4; i++)
      X_small.val[i][j] = X.val[i][idx[j]];
	return X_small;
}

bool VisualOdometry::monoNormalizeFeaturePoints(vector<Matcher::p_match> &p_matched,Matrix &Tp,Matrix &Tc) {

  // shift origins to centroids
  float cpu=0,cpv=0,ccu=0,ccv=0;
  for (vector<Matcher::p_match>::iterator it = p_matched.begin(); it!=p_matched.end(); it++) {
    cpu += it->u1p;
    cpv += it->v1p;
    ccu += it->u1c;
    ccv += it->v1c;
  }
  cpu /= (float)p_matched.size();
  cpv /= (float)p_matched.size();
  ccu /= (float)p_matched.size();
  ccv /= (float)p_matched.size();
  for (vector<Matcher::p_match>::iterator it = p_matched.begin(); it!=p_matched.end(); it++) {
    it->u1p -= cpu;
    it->v1p -= cpv;
    it->u1c -= ccu;
    it->v1c -= ccv;
  }

  // scale features such that mean distance from origin is sqrt(2)
  float sp=0,sc=0;
  for (vector<Matcher::p_match>::iterator it = p_matched.begin(); it!=p_matched.end(); it++) {
    sp += sqrt(it->u1p*it->u1p+it->v1p*it->v1p);
    sc += sqrt(it->u1c*it->u1c+it->v1c*it->v1c);
  }
  if (fabs(sp)<1e-10 || fabs(sc)<1e-10)
    return false;
  sp = sqrt(2.0)*(float)p_matched.size()/sp;
  sc = sqrt(2.0)*(float)p_matched.size()/sc;
  for (vector<Matcher::p_match>::iterator it = p_matched.begin(); it!=p_matched.end(); it++) {
    it->u1p *= sp;
    it->v1p *= sp;
    it->u1c *= sc;
    it->v1c *= sc;
  }

  // compute corresponding transformation matrices
  myFLOAT Tp_data[9] = {sp,0,-sp*cpu,0,sp,-sp*cpv,0,0,1};
  myFLOAT Tc_data[9] = {sc,0,-sc*ccu,0,sc,-sc*ccv,0,0,1};
  Tp = Matrix(3,3,Tp_data);
  Tc = Matrix(3,3,Tc_data);

  // return true on success
  return true;
}

vector<int32_t> VisualOdometry::monoGetRandomSample(int32_t N,int32_t num) {

  // init sample and totalset
  vector<int32_t> sample;
  vector<int32_t> totalset;

  // create vector containing all indices
  for (int32_t i=0; i<N; i++)
    totalset.push_back(i);

  // add numm indices to current sample
  sample.clear();
  for (int32_t i=0; i<num; i++) {
    int32_t j = rand()%totalset.size();
    sample.push_back(totalset[j]);
    totalset.erase(totalset.begin()+j);
  }

  // return sample
  return sample;
}

void VisualOdometry::monoFundamentalMatrix (const vector<Matcher::p_match> &p_matched,const vector<int32_t> &active,Matrix &F) {

  // number of active p_matched
  int32_t N = active.size();

  // create constraint matrix A
  Matrix A(N,9);
  for (int32_t i=0; i<N; i++) {
    Matcher::p_match m = p_matched[active[i]];
    A.val[i][0] = m.u1c*m.u1p;
    A.val[i][1] = m.u1c*m.v1p;
    A.val[i][2] = m.u1c;
    A.val[i][3] = m.v1c*m.u1p;
    A.val[i][4] = m.v1c*m.v1p;
    A.val[i][5] = m.v1c;
    A.val[i][6] = m.u1p;
    A.val[i][7] = m.v1p;
    A.val[i][8] = 1;
  }

  // compute singular value decomposition of A
  Matrix U,W,V;
  A.svd(U,W,V);

  // extract fundamental matrix from the column of V corresponding to the smallest singular value
  F = Matrix::reshape(V.getMat(0,8,8,8),3,3);

  // enforce rank 2
  F.svd(U,W,V);
  W.val[2][0] = 0;
  F = U*Matrix::diag(W)*~V;
}

vector<int32_t> VisualOdometry::monoGetFundamentalInlier (vector<Matcher::p_match> &p_matched,Matrix &F,myFLOAT tau) {

  // extract fundamental matrix
  myFLOAT f00 = F.val[0][0]; myFLOAT f01 = F.val[0][1]; myFLOAT f02 = F.val[0][2];
  myFLOAT f10 = F.val[1][0]; myFLOAT f11 = F.val[1][1]; myFLOAT f12 = F.val[1][2];
  myFLOAT f20 = F.val[2][0]; myFLOAT f21 = F.val[2][1]; myFLOAT f22 = F.val[2][2];

  // loop variables
  myFLOAT u1,v1,u2,v2;
  myFLOAT x2tFx1;
  myFLOAT Fx1u,Fx1v,Fx1w;
  myFLOAT Ftx2u,Ftx2v;

  // vector with inliers
  vector<int32_t> inliers;

  // for all matches do
  for (int32_t i=0; i<(int32_t)p_matched.size(); i++) {

    // extract matches
    u1 = p_matched[i].u1p;
    v1 = p_matched[i].v1p;
    u2 = p_matched[i].u1c;
    v2 = p_matched[i].v1c;

    // F*x1
    Fx1u = f00*u1+f01*v1+f02;
    Fx1v = f10*u1+f11*v1+f12;
    Fx1w = f20*u1+f21*v1+f22;

    // F'*x2
    Ftx2u = f00*u2+f10*v2+f20;
    Ftx2v = f01*u2+f11*v2+f21;

    // x2'*F*x1
    x2tFx1 = u2*Fx1u+v2*Fx1v+Fx1w;

    // sampson distance
    myFLOAT d = x2tFx1*x2tFx1 / (Fx1u*Fx1u+Fx1v*Fx1v+Ftx2u*Ftx2u+Ftx2v*Ftx2v);

    // check threshold
    if (fabs(d)<tau)
      inliers.push_back(i);
  }

  // return set of all inliers
  return inliers;
}

void VisualOdometry::monoEtoRt(Matrix &E,Matrix &K,vector<Matcher::p_match> &p_matched,vector<int32_t> &inliers,Matrix &X,Matrix &R,Matrix &t) {

  // hartley matrices
  myFLOAT W_data[9] = {0,-1,0,+1,0,0,0,0,1};
  myFLOAT Z_data[9] = {0,+1,0,-1,0,0,0,0,0};
  Matrix W(3,3,W_data);
  Matrix Z(3,3,Z_data);

  // extract T,R1,R2 (8 solutions)
  Matrix U,S,V;
  E.svd(U,S,V);
  Matrix T  = U*Z*~U;
  Matrix Ra = U*W*(~V);
  Matrix Rb = U*(~W)*(~V);

  // convert T to t
  t = Matrix(3,1);
  t.val[0][0] = T.val[2][1];
  t.val[1][0] = T.val[0][2];
  t.val[2][0] = T.val[1][0];

  // assure determinant to be positive
  if (Ra.det()<0) Ra = -Ra;
  if (Rb.det()<0) Rb = -Rb;

  // create vector containing all 4 solutions
  vector<Matrix> R_vec;
  vector<Matrix> t_vec;
  R_vec.push_back(Ra); t_vec.push_back( t);
  R_vec.push_back(Ra); t_vec.push_back(-t);
  R_vec.push_back(Rb); t_vec.push_back( t);
  R_vec.push_back(Rb); t_vec.push_back(-t);

  // try all 4 solutions
  Matrix X_curr;
  int32_t max_inliers = 0;
  for (int32_t i=0; i<4; i++) {
    int32_t num_inliers = monoTriangulateChieral(p_matched,K,R_vec[i],t_vec[i],X_curr);
    if (num_inliers>max_inliers) {
      max_inliers = num_inliers;
      X = X_curr;
      R = R_vec[i];
      t = t_vec[i];
    }
  }
}

int32_t VisualOdometry::monoTriangulateChieral (vector<Matcher::p_match> &p_matched,Matrix &K,Matrix &R,Matrix &t,Matrix &X) {

  // init 3d point matrix
  X = Matrix(4,p_matched.size());

  // projection matrices
  Matrix P1(3,4);
  Matrix P2(3,4);
  P1.setMat(K,0,0);
  P2.setMat(R,0,0);
  P2.setMat(t,0,3);
  P2 = K*P2;

  // triangulation via orthogonal regression
  Matrix J(4,4);
  Matrix U,S,V;
  for (int32_t i=0; i<p_matched.size(); i++) {
    for (int32_t j=0; j<4; j++) {
      J.val[0][j] = P1.val[2][j]*p_matched[i].u1p - P1.val[0][j];
      J.val[1][j] = P1.val[2][j]*p_matched[i].v1p - P1.val[1][j];
      J.val[2][j] = P2.val[2][j]*p_matched[i].u1c - P2.val[0][j];
      J.val[3][j] = P2.val[2][j]*p_matched[i].v1c - P2.val[1][j];
    }
    J.svd(U,S,V);
    X.setMat(V.getMat(0,3,3,3),0,i);
  }

  // compute inliers
  Matrix  AX1 = P1*X;
  Matrix  BX1 = P2*X;
  int32_t num = 0;
  for (int32_t i=0; i<X.n; i++)
    if (AX1.val[2][i]*X.val[3][i]>0 && BX1.val[2][i]*X.val[3][i]>0)
      num++;

  // return number of inliers
  return num;
}

bool VisualOdometry::processStereo (vector<Matcher::p_match> p_matched,myFLOAT deltaT) {

  // return value
  bool success = true;

  // compute minimum distance for RANSAC samples
  float width=0,height=0;
  for (vector<Matcher::p_match>::iterator it=p_matched.begin(); it!=p_matched.end(); it++) {
    if (it->u1c>width)  width  = it->u1c;
    if (it->v1c>height) height = it->v1c;
  }
  float min_dist = min(width,height)/3.0;

  // allocate dynamic memory
  int32_t N = p_matched.size();
  X         = new myFLOAT[N];
  Y         = new myFLOAT[N];
  Z         = new myFLOAT[N];
  J         = new myFLOAT[4*N*6];
  p_predict = new myFLOAT[4*N];
  p_observe = new myFLOAT[4*N];

 // /* ********************************************************************* */
 // /* triangulation 2D to 3D points begin
 // /* ********************************************************************* */
 // // triangulate 2D projections to 3D
 // // create 2d correspondences
 // CvMat *projPts_1 = cvCreateMat(1,N,CV_32FC2);
 // CvMat *projPts_2 = cvCreateMat(1,N,CV_32FC2);
 // for (int32_t i=0; i<N; i++) {
	//  CV_MAT_ELEM( *projPts_1, CvPoint2D32f, 0, i ).x = p_matched[i].u1p;
	//  CV_MAT_ELEM( *projPts_1, CvPoint2D32f, 0, i ).y = p_matched[i].v1p;
	//  CV_MAT_ELEM( *projPts_2, CvPoint2D32f, 0, i ).x = p_matched[i].u2p;
	//  CV_MAT_ELEM( *projPts_2, CvPoint2D32f, 0, i ).y = p_matched[i].v2p;
 // }

 // // define intrinsic and extrinsic
 // double _K[9] = { calib_f, 0, calib_cu,
	//				0, calib_f, calib_cv,
	//				0, 0, 1};
 // double _K_1[9] = { calib_fu_1, 0, calib_cu_1,
	//				0, calib_fv_1, calib_cv_1,
	//				0, 0, 1};
 // double _K_2[9] = { calib_fu_2, 0, calib_cu_2,
	//					0, calib_fv_2, calib_cv_2,
	//					0, 0, 1};
 // CvMat K = cvMat(3,3, CV_64F,_K);
 // CvMat K_1 = cvMat(3,3, CV_64FC1,_K_1);
 // CvMat K_2 = cvMat(3,3, CV_64FC1,_K_2);

 // //the camera calibration also delivers some distortion parameters
 // //double _camDist[5] = { 0.00177, -0.00396, -0.00055, -0.00014, 0};
 // //CvMat camDist = cvMat(5,1,CV_64F,_camDist);
 //  double camDist_a[5] = { 0.00252, -0.01432, -0.00003, 0.00041, 0.00000};
 //  CvMat camDist_1 = cvMat(5,1,CV_64F,camDist_a);
 //  double camDist_b[5] = { 0.00051, -0.00190, -0.00018, -0.00013, 0.00000};
 //  CvMat camDist_2 = cvMat(5,1,CV_64F,camDist_b);
 //
 // //define the extrinsic of stereo frame
 // //the first camera is easy, because we define ths view as our system origin
 // double _camExt_1[12] = { 1,0,0,0,
 //                         0,1,0,0,
 //                         0,0,1,0};
 // CvMat camExt_1 = cvMat(3,4,CV_64F,_camExt_1);
 // //the camera matrix of the second camera is then [R,T]
 // double _camExt_2[12] = { extrinsic.val[0][0], extrinsic.val[0][1], extrinsic.val[0][2], extrinsic.val[0][3],
	//						extrinsic.val[1][0], extrinsic.val[1][1], extrinsic.val[1][2], extrinsic.val[1][3],
	//						extrinsic.val[2][0], extrinsic.val[2][1], extrinsic.val[2][2], extrinsic.val[2][3]};
 // CvMat camExt_2 = cvMat(3,4,CV_64F,_camExt_2);

 // /**
 //    * 3D localization
 //    * now we have all parameters we also get from our stereo frame
 //    * - intrinsic and extrinsic of cameras
 //    * - coordinates of a point in both images
 //    *
 //    * so we calculate the 3d position of this point in our system
 //    */
 // //first we need do the inverse of the projective transformation to get
 // //the points coordinates in the cameras normal space
 // cvUndistortPoints(projPts_1,projPts_1,&K,&camDist_1);
 // cvUndistortPoints(projPts_2,projPts_2,&K,&camDist_2);

 // //be sure the point's are saved in right matrix format 2xN 1 channel
 // CvMat *_pointImg1 = cvCreateMat(2,N,CV_32FC1);
 // CvMat *_pointImg2 = cvCreateMat(2,N,CV_32FC1);
 // for (int32_t i=0; i<N; i++) {
	//  CV_MAT_ELEM( *_pointImg1, float, 0, i ) = CV_MAT_ELEM( *projPts_1, CvPoint2D32f, 0, i ).x;
	//  CV_MAT_ELEM( *_pointImg1, float, 1, i ) = CV_MAT_ELEM( *projPts_1, CvPoint2D32f, 0, i ).y;
	//  CV_MAT_ELEM( *_pointImg2, float, 0, i ) = CV_MAT_ELEM( *projPts_2, CvPoint2D32f, 0, i ).x;
	//  CV_MAT_ELEM( *_pointImg2, float, 1, i ) = CV_MAT_ELEM( *projPts_2, CvPoint2D32f, 0, i ).y;
 // }

 // //triangulate both projections to find real point position
 // //!!! all parameters must be double type
 // CvMat *point3D = cvCreateMat(4,N,CV_32F) ;
 // cvTriangulatePoints( &camExt_1, &camExt_2, _pointImg1, _pointImg2, point3D );

 // //to get the real position we need to do also a homogeneous division
 // for (int32_t i=0; i<N; i++) {
	//  CV_MAT_ELEM( *point3D, float, 0, i ) /= CV_MAT_ELEM( *point3D, float, 3, i );
	//  CV_MAT_ELEM( *point3D, float, 1, i ) /= CV_MAT_ELEM( *point3D, float, 3, i );
	//  CV_MAT_ELEM( *point3D, float, 2, i ) /= CV_MAT_ELEM( *point3D, float, 3, i );
 // }

 // //std::cout << point3D->data.fl[0] << " " << point3D->data.fl[1] << " " << point3D->data.fl[2] << std::endl;

 // // restore the 3D coordinates
 // for (int32_t i=0; i<N; i++) {
 //   X[i]    = CV_MAT_ELEM(*point3D, float, 0, i);
 //   Y[i]    = CV_MAT_ELEM(*point3D, float, 1, i);
 //   Z[i]    = CV_MAT_ELEM(*point3D, float, 2, i);
	//std::cout << CV_MAT_ELEM(*point3D, float, 3, i) << std::endl;
 // }

 // // release cvMat
 // cvReleaseData(projPts_1);
 // cvReleaseData(projPts_2);
 // cvReleaseData(&K);
 // cvReleaseData(&K_1);
 // cvReleaseData(&K_2);
 // cvReleaseData(_pointImg1);
 // cvReleaseData(_pointImg2);
 // //cvReleaseData(&camDist);
 // cvReleaseData(&camDist_1);
 // cvReleaseData(&camDist_2);
 // cvReleaseData(&camExt_1);
 // cvReleaseData(&camExt_2);
 // cvReleaseData(point3D);
 // /* ********************************************************************* */
 // /* triangulation 2D to 3D points finish
 // /* ********************************************************************* */

  // get number of matches
  if (N<6) {
    success = false;
    goto failed;
  }

  // project matches of previous image into 3d
  for (int32_t i=0; i<N; i++) {
    myFLOAT d = max(p_matched[i].u1p - p_matched[i].u2p,(float)1.0);
	//myFLOAT d = abs(p_matched[i].u1p - p_matched[i].u2p);	// disparity
    X[i]    = (p_matched[i].u1p-calib_cu)*calib_b/d;
    Y[i]    = (p_matched[i].v1p-calib_cv)*calib_b/d;
    Z[i]    = calib_f*calib_b/d;
  }

  // loop variables
  myFLOAT param_curr[6];

  // clear parameter and inlier vectors
  inliers.clear();
  for (int32_t i=0; i<6; i++)
      param[i] = 0;

  //////////////////////////////////
  // initial RANSAC estimate
  for (int32_t k=0;k<50;k++) {

    // draw random sample set
    vector<int32_t> active = stereoGetRandomSample(p_matched,min_dist);
    if (active.size()<3) {
      success = false;
      goto failed;
    }

    // clear parameter vector
    for (int32_t i=0; i<6; i++)
      param_curr[i] = 0;

    // perform bundle adjustment
    VisualOdometry::result result = UPDATED;
    int32_t iter=0;
    while (result==UPDATED) {
      result = stereoUpdateParameters(p_matched,active,param_curr,1,1e-6);
      if (iter++ > 20 || result==CONVERGED)
        break;
    }

    // overwrite best parameters if we have more inliers
    if (result!=FAILED) {
      vector<int32_t> inliers_curr = stereoGetInlier(p_matched,param_curr,4);
      if (inliers_curr.size()>inliers.size()) {
        inliers = inliers_curr;
        parcpy(param,param_curr);
      }
    }
  }

  //////////////////////////////////
  // final optimization (refinement)
  if (inliers.size()>=6) {
    int32_t iter=0;
    VisualOdometry::result result = UPDATED;
    while (result==UPDATED) {
      result = stereoUpdateParameters(p_matched,inliers,param,1,1e-8);
      if (iter++ > 100 || result==CONVERGED)
        break;
    }

    // not converged
    if (result!=CONVERGED)
      success = false;

  // not enough inliers
  } else {
    success = false;
  }

  // jump here if something went wrong
  failed:

  // release dynamic memory
  delete X;
  delete Y;
  delete Z;
  delete J;
  delete p_predict;
  delete p_observe;

  // parameter estimate succeeded?
  return success;
}

void VisualOdometry::parcpy(myFLOAT *param_dst,myFLOAT *param_src) {
  memcpy(param_dst,param_src,6*sizeof(myFLOAT));
}

void VisualOdometry::stereoPlotErrors(const vector<Matcher::p_match> p_matched,const vector<int32_t> &active,myFLOAT *param) {
  stereoComputeObservations(p_matched,active);
  stereoComputePredictionsAndJacobian(param,active);
  float err_u1 = 0;
  float err_v1 = 0;
  float err_u2 = 0;
  float err_v2 = 0;
  for (int32_t i=0; i<(int32_t)active.size(); i++) {
    err_u1 += fabs(p_predict[4*i+0]-p_observe[4*i+0]);
    err_v1 += fabs(p_predict[4*i+1]-p_observe[4*i+1]);
    err_u2 += fabs(p_predict[4*i+2]-p_observe[4*i+2]);
    err_v2 += fabs(p_predict[4*i+3]-p_observe[4*i+3]);
  }
  cout << "Errors: ";
  cout << err_u1/(float)active.size() << " ";
  cout << err_v1/(float)active.size() << " ";
  cout << err_u2/(float)active.size() << " ";
  cout << err_v2/(float)active.size() << endl;
}

vector<int32_t> VisualOdometry::stereoGetInlier(const vector<Matcher::p_match> p_matched,const myFLOAT *param,myFLOAT tau) {

  // mark all observations active
  vector<int32_t> active;
  for (int32_t i=0; i<(int32_t)p_matched.size(); i++)
    active.push_back(i);

  // extract observations and compute predictions
  stereoComputeObservations(p_matched,active);
  stereoComputePredictionsAndJacobian(param,active);

  // compute inliers
  vector<int32_t> inliers;
  for (int32_t i=0; i<(int32_t)p_matched.size(); i++)
    if (pow(p_predict[4*i+0]-p_observe[4*i+0],2)+pow(p_predict[4*i+1]-p_observe[4*i+1],2) +
        pow(p_predict[4*i+2]-p_observe[4*i+2],2)+pow(p_predict[4*i+3]-p_observe[4*i+3],2)<pow(tau,2))
      inliers.push_back(i);
  return inliers;
}

vector<int32_t> VisualOdometry::stereoGetRandomSample(const vector<Matcher::p_match> &p_matched,const float &min_dist) {

  // init sample and totalset
  vector<int32_t> sample;
  vector<int32_t> totalset;

  bool success = false;
  int32_t k=0;

  // try maximally 100 times to create a sample
  while (!success && ++k<100) {

    // create vector containing all indices
    totalset.clear();
    for (int32_t i=0; i<(int32_t)p_matched.size(); i++)
      totalset.push_back(i);

    // add 3 indices to current sample
    sample.clear();
    for (int32_t i=0; i<3; i++) {
      int32_t j = rand()%totalset.size();
      sample.push_back(totalset[j]);
      totalset.erase(totalset.begin()+j);
    }

    // check distances
    float du = p_matched[sample[0]].u1c-p_matched[sample[1]].u1c;
    float dv = p_matched[sample[0]].v1c-p_matched[sample[1]].v1c;
    if (sqrt(du*du+dv*dv)>min_dist) {
      float norm = sqrt(du*du+dv*dv);
      float nu   = +dv/norm;
      float nv   = -du/norm;
      float ru   = p_matched[sample[2]].u1c-p_matched[sample[0]].u1c;
      float rv   = p_matched[sample[2]].v1c-p_matched[sample[0]].v1c;
      if (fabs(nu*ru+nv*rv)>min_dist) {
        success = true;
        break;
      }
    }
  }

  // return empty sample on failure
  if (!success)
    sample.clear();

  // return sample
  return sample;
}

VisualOdometry::result VisualOdometry::stereoUpdateParameters(const vector<Matcher::p_match> &p_matched,const vector<int32_t> &active,myFLOAT *param,const myFLOAT &step_size,const myFLOAT &eps) {

  // we need at least 3 observations
  if (active.size()<3)
    return FAILED;

  // extract observations and compute predictions
  stereoComputeObservations(p_matched,active);
  stereoComputePredictionsAndJacobian(param,active);

  // init
  Matrix A(6,6);
  Matrix B(6,1);

  // fill matrices A and B
  for (int32_t m=0; m<6; m++) {
    for (int32_t n=0; n<6; n++) {
      myFLOAT a = 0;
      for (int32_t i=0; i<4*(int32_t)active.size(); i++)
        a += J[i*6+m]*J[i*6+n];
      A.val[m][n] = a;
    }
    myFLOAT b = 0;
    for (int32_t i=0; i<4*(int32_t)active.size(); i++)
      b += J[i*6+m]*(p_observe[i]-p_predict[i]);
    B.val[m][0] = b;
  }

  // perform elimination
  if (B.solve(A)) {
    bool converged = true;
    for (int32_t m=0; m<6; m++) {
      param[m] += step_size*B.val[m][0];
      if (fabs(B.val[m][0])>eps)
        converged = false;
    }
    if (converged)
      return CONVERGED;
    else
      return UPDATED;
  } else {
    return FAILED;
  }
}

void VisualOdometry::stereoComputeObservations(const vector<Matcher::p_match> &p_matched,const vector<int32_t> &active) {

  // set all observations
  for (int32_t i=0; i<(int32_t)active.size(); i++) {
    p_observe[4*i+0] = p_matched[active[i]].u1c; // u1
    p_observe[4*i+1] = p_matched[active[i]].v1c; // v1
    p_observe[4*i+2] = p_matched[active[i]].u2c; // u2
    p_observe[4*i+3] = p_matched[active[i]].v2c; // v2
  }
}

VisualOdometry::result VisualOdometry::monoUpdateParameters(const vector<Matcher::p_match> &p_matched,const vector<int32_t> &active,myFLOAT *param,const myFLOAT &step_size,const myFLOAT &eps) {

  // we need at least 3 observations
  if (active.size()<3)
    return FAILED;

  // extract observations and compute predictions
  monoComputeObservations(p_matched,active);
  monoComputePredictionsAndJacobian(param,active.size());

  // number of parameters
  int32_t N = 6+3*active.size();

  // init
  Matrix A(N,N);
  Matrix B(N,1);

  // fill matrices A and B
  for (int32_t m=0; m<N; m++) {
    for (int32_t n=0; n<N; n++) {
      myFLOAT a = 0;
      for (int32_t i=0; i<4*(int32_t)active.size(); i++)
        a += J[i*N+m]*J[i*N+n];
      A.val[m][n] = a;
    }
    myFLOAT b = 0;
    for (int32_t i=0; i<4*(int32_t)active.size(); i++)
      b += J[i*N+m]*(p_observe[i]-p_predict[i]);
    B.val[m][0] = b;
  }

  // perform elimination
  if (B.solve(A)) {
    bool converged = true;
    for (int32_t m=0; m<N; m++) {
      param[m] += step_size*B.val[m][0];
      if (fabs(B.val[m][0])>eps)
        converged = false;
    }
    if (converged)
      return CONVERGED;
    else
      return UPDATED;
  }
  return FAILED;
}

void VisualOdometry::monoComputeObservations(const vector<Matcher::p_match> &p_matched,const vector<int32_t> &active) {

  // set all observations
  for (int32_t i=0; i<(int32_t)active.size(); i++) {
    p_observe[4*i+0] = p_matched[active[i]].u1p; // u1
    p_observe[4*i+1] = p_matched[active[i]].v1p; // v1
    p_observe[4*i+2] = p_matched[active[i]].u1c; // u2
    p_observe[4*i+3] = p_matched[active[i]].v1c; // v2
  }
}

void VisualOdometry::monoComputePredictionsAndJacobian(const myFLOAT *param,int32_t num_pt) {

  // extract motion parameters
  myFLOAT rx = param[0]; myFLOAT ry = param[1]; myFLOAT rz = param[2];
  myFLOAT tx = param[3]; myFLOAT ty = param[4]; myFLOAT tz = param[5];

  // precompute sine/cosine
  myFLOAT sx = sin(rx); myFLOAT cx = cos(rx); myFLOAT sy = sin(ry);
  myFLOAT cy = cos(ry); myFLOAT sz = sin(rz); myFLOAT cz = cos(rz);

  // compute rotation matrix and derivatives
  myFLOAT r00    = +cy*cz;          myFLOAT r01    = -cy*sz;          myFLOAT r02    = +sy;
  myFLOAT r10    = +sx*sy*cz+cx*sz; myFLOAT r11    = -sx*sy*sz+cx*cz; myFLOAT r12    = -sx*cy;
  myFLOAT r20    = -cx*sy*cz+sx*sz; myFLOAT r21    = +cx*sy*sz+sx*cz; myFLOAT r22    = +cx*cy;
  //myFLOAT rdrx10 = +cx*sy*cz-sx*sz; myFLOAT rdrx11 = -cx*sy*sz-sx*sz; myFLOAT rdrx12 = -cx*cy;
  myFLOAT rdrx10 = +cx*sy*cz-sx*sz; myFLOAT rdrx11 = -cx*sy*sz-sx*cz; myFLOAT rdrx12 = -cx*cy;
  myFLOAT rdrx20 = +sx*sy*cz+cx*sz; myFLOAT rdrx21 = -sx*sy*sz+cx*cz; myFLOAT rdrx22 = -sx*cy;
  myFLOAT rdry00 = -sy*cz;          myFLOAT rdry01 = +sy*sz;          myFLOAT rdry02 = +cy;
  myFLOAT rdry10 = +sx*cy*cz;       myFLOAT rdry11 = -sx*cy*sz;       myFLOAT rdry12 = +sx*sy;
  myFLOAT rdry20 = -cx*cy*cz;       myFLOAT rdry21 = +cx*cy*sz;       myFLOAT rdry22 = -cx*sy;
  myFLOAT rdrz00 = -cy*sz;          myFLOAT rdrz01 = -cy*cz;
  myFLOAT rdrz10 = -sx*sy*sz+cx*cz; myFLOAT rdrz11 = -sx*sy*cz-cx*sz;
  myFLOAT rdrz20 = +cx*sy*sz+sx*cz; myFLOAT rdrz21 = +cx*sy*cz-sx*sz;

  // loop variables
  myFLOAT X1p,Y1p,Z1p;
  myFLOAT X1c,Y1c,Z1c;
  myFLOAT X1cd,Y1cd,Z1cd;

  // for all observations do
  for (int32_t i=0; i<num_pt; i++) {

    // get 3d point in previous coordinate system
    X1p = param[6+i*3+0];
    Y1p = param[6+i*3+1];
    Z1p = param[6+i*3+2];

    // compute 3d point in current left coordinate system
    X1c = r00*X1p+r01*Y1p+r02*Z1p+tx;
    Y1c = r10*X1p+r11*Y1p+r12*Z1p+ty;
    Z1c = r20*X1p+r21*Y1p+r22*Z1p+tz;

    // parameter derivatives
    for (int32_t j=0; j<6; j++) {
      switch (j) {
        case 0: X1cd = 0;
                Y1cd = rdrx10*X1p+rdrx11*Y1p+rdrx12*Z1p;
                Z1cd = rdrx20*X1p+rdrx21*Y1p+rdrx22*Z1p;
                break;
        case 1: X1cd = rdry00*X1p+rdry01*Y1p+rdry02*Z1p;
                Y1cd = rdry10*X1p+rdry11*Y1p+rdry12*Z1p;
                Z1cd = rdry20*X1p+rdry21*Y1p+rdry22*Z1p;
                break;
        case 2: X1cd = rdrz00*X1p+rdrz01*Y1p;
                Y1cd = rdrz10*X1p+rdrz11*Y1p;
                Z1cd = rdrz20*X1p+rdrz21*Y1p;
                break;
        case 3: X1cd = 1; Y1cd = 0; Z1cd = 0; break;
        case 4: X1cd = 0; Y1cd = 1; Z1cd = 0; break;
        case 5: X1cd = 0; Y1cd = 0; Z1cd = 1; break;
      }

      // set jacobian entries (project via K)
      J[(4*i+0)*(6+3*num_pt)+j] = 0;                                     // previous u'
      J[(4*i+1)*(6+3*num_pt)+j] = 0;                                     // previous v'
      J[(4*i+2)*(6+3*num_pt)+j] = calib_f*(X1cd*Z1c-X1c*Z1cd)/(Z1c*Z1c); // current u'
      J[(4*i+3)*(6+3*num_pt)+j] = calib_f*(Y1cd*Z1c-Y1c*Z1cd)/(Z1c*Z1c); // current v'
    }

    // 3d point derivatives
    for (int32_t j=0; j<num_pt; j++) {
      J[(4*i+0)*(6+3*num_pt)+6+3*j+0] = calib_f/Z1p;                         // previous u'/X
      J[(4*i+0)*(6+3*num_pt)+6+3*j+1] = 0;                                   // previous u'/Y
      J[(4*i+0)*(6+3*num_pt)+6+3*j+2] = -calib_f*X1p/(Z1p*Z1p);              // previous u'/Z
      J[(4*i+1)*(6+3*num_pt)+6+3*j+0] = 0;                                   // previous v'/X
      J[(4*i+1)*(6+3*num_pt)+6+3*j+1] = calib_f/Z1p;                         // previous v'/Y
      J[(4*i+1)*(6+3*num_pt)+6+3*j+2] = -calib_f*Y1p/(Z1p*Z1p);              // previous v'/Z
      J[(4*i+2)*(6+3*num_pt)+6+3*j+0] = calib_f*(r00*Z1c-X1c*r20)/(Z1c*Z1c); // current u'/X
      J[(4*i+2)*(6+3*num_pt)+6+3*j+1] = calib_f*(r01*Z1c-X1c*r21)/(Z1c*Z1c); // current u'/Y
      J[(4*i+2)*(6+3*num_pt)+6+3*j+2] = calib_f*(r02*Z1c-X1c*r22)/(Z1c*Z1c); // current u'/Z
      J[(4*i+3)*(6+3*num_pt)+6+3*j+0] = calib_f*(r10*Z1c-Y1c*r20)/(Z1c*Z1c); // current v'/X
      J[(4*i+3)*(6+3*num_pt)+6+3*j+1] = calib_f*(r11*Z1c-Y1c*r21)/(Z1c*Z1c); // current v'/Y
      J[(4*i+3)*(6+3*num_pt)+6+3*j+2] = calib_f*(r12*Z1c-Y1c*r22)/(Z1c*Z1c); // current v'/Z
    }

    // set prediction (project via K)
    p_predict[4*i+0] = calib_f*X1p/Z1p+calib_cu; // previous u
    p_predict[4*i+1] = calib_f*Y1p/Z1p+calib_cv; // previous v
    p_predict[4*i+2] = calib_f*X1c/Z1c+calib_cu; // current u
    p_predict[4*i+3] = calib_f*Y1c/Z1c+calib_cv; // current v
  }
}

void VisualOdometry::stereoComputePredictionsAndJacobian(const myFLOAT *param,const vector<int32_t> &active) {

  // extract motion parameters
  myFLOAT rx = param[0]; myFLOAT ry = param[1]; myFLOAT rz = param[2];
  myFLOAT tx = param[3]; myFLOAT ty = param[4]; myFLOAT tz = param[5];

  // precompute sine/cosine
  myFLOAT sx = sin(rx); myFLOAT cx = cos(rx); myFLOAT sy = sin(ry);
  myFLOAT cy = cos(ry); myFLOAT sz = sin(rz); myFLOAT cz = cos(rz);

  // compute rotation matrix and derivatives
  myFLOAT r00    = +cy*cz;          myFLOAT r01    = -cy*sz;          myFLOAT r02    = +sy;
  myFLOAT r10    = +sx*sy*cz+cx*sz; myFLOAT r11    = -sx*sy*sz+cx*cz; myFLOAT r12    = -sx*cy;
  myFLOAT r20    = -cx*sy*cz+sx*sz; myFLOAT r21    = +cx*sy*sz+sx*cz; myFLOAT r22    = +cx*cy;
  //myFLOAT rdrx10 = +cx*sy*cz-sx*sz; myFLOAT rdrx11 = -cx*sy*sz-sx*sz; myFLOAT rdrx12 = -cx*cy;
  myFLOAT rdrx10 = +cx*sy*cz-sx*sz; myFLOAT rdrx11 = -cx*sy*sz-sx*cz; myFLOAT rdrx12 = -cx*cy;
  myFLOAT rdrx20 = +sx*sy*cz+cx*sz; myFLOAT rdrx21 = -sx*sy*sz+cx*cz; myFLOAT rdrx22 = -sx*cy;
  myFLOAT rdry00 = -sy*cz;          myFLOAT rdry01 = +sy*sz;          myFLOAT rdry02 = +cy;
  myFLOAT rdry10 = +sx*cy*cz;       myFLOAT rdry11 = -sx*cy*sz;       myFLOAT rdry12 = +sx*sy;
  myFLOAT rdry20 = -cx*cy*cz;       myFLOAT rdry21 = +cx*cy*sz;       myFLOAT rdry22 = -cx*sy;
  myFLOAT rdrz00 = -cy*sz;          myFLOAT rdrz01 = -cy*cz;
  myFLOAT rdrz10 = -sx*sy*sz+cx*cz; myFLOAT rdrz11 = -sx*sy*cz-cx*sz;
  myFLOAT rdrz20 = +cx*sy*sz+sx*cz; myFLOAT rdrz21 = +cx*sy*cz-sx*sz;

  // loop variables
  myFLOAT X1p,Y1p,Z1p;
  myFLOAT X1c,Y1c,Z1c,X2c;
  myFLOAT X1cd,Y1cd,Z1cd;

  // for all observations do
  for (int32_t i=0; i<(int32_t)active.size(); i++) {

    // get 3d point in previous coordinate system
    X1p = X[active[i]];
    Y1p = Y[active[i]];
    Z1p = Z[active[i]];

    // compute 3d point in current left coordinate system
    X1c = r00*X1p+r01*Y1p+r02*Z1p+tx;
    Y1c = r10*X1p+r11*Y1p+r12*Z1p+ty;
    Z1c = r20*X1p+r21*Y1p+r22*Z1p+tz;

    // compute 3d point in current right coordinate system
    X2c = X1c-calib_b;

    // for all paramters do
    for (int32_t j=0; j<6; j++) {

      // compute derivatives of 3d point in current
      // left coordinate system wrt. parameter j
      switch (j) {
        case 0: X1cd = 0;
                Y1cd = rdrx10*X1p+rdrx11*Y1p+rdrx12*Z1p;
                Z1cd = rdrx20*X1p+rdrx21*Y1p+rdrx22*Z1p;
                break;
        case 1: X1cd = rdry00*X1p+rdry01*Y1p+rdry02*Z1p;
                Y1cd = rdry10*X1p+rdry11*Y1p+rdry12*Z1p;
                Z1cd = rdry20*X1p+rdry21*Y1p+rdry22*Z1p;
                break;
        case 2: X1cd = rdrz00*X1p+rdrz01*Y1p;
                Y1cd = rdrz10*X1p+rdrz11*Y1p;
                Z1cd = rdrz20*X1p+rdrz21*Y1p;
                break;
        case 3: X1cd = 1; Y1cd = 0; Z1cd = 0; break;
        case 4: X1cd = 0; Y1cd = 1; Z1cd = 0; break;
        case 5: X1cd = 0; Y1cd = 0; Z1cd = 1; break;
      }

      // set jacobian entries (project via K)
      J[(4*i+0)*6+j] = calib_f*(X1cd*Z1c-X1c*Z1cd)/(Z1c*Z1c); // left u'
      J[(4*i+1)*6+j] = calib_f*(Y1cd*Z1c-Y1c*Z1cd)/(Z1c*Z1c); // left v'
      J[(4*i+2)*6+j] = calib_f*(X1cd*Z1c-X2c*Z1cd)/(Z1c*Z1c); // right u'
      J[(4*i+3)*6+j] = calib_f*(Y1cd*Z1c-Y1c*Z1cd)/(Z1c*Z1c); // right v'
    }

    // set prediction (project via K)
    p_predict[4*i+0] = calib_f*X1c/Z1c+calib_cu; // left u
    p_predict[4*i+1] = calib_f*Y1c/Z1c+calib_cv; // left v
    p_predict[4*i+2] = calib_f*X2c/Z1c+calib_cu; // right u
    p_predict[4*i+3] = calib_f*Y1c/Z1c+calib_cv; // right v
  }
}

void VisualOdometry::stereoTestJacobian() {
  cout << "=================================" << endl;
  cout << "TESTING JACOBIAN" << endl;
  myFLOAT delta = 1e-8;
  myFLOAT param_ref[6];
  param_ref[0] = 0.05;
  param_ref[1] = -0.05;
  param_ref[2] = -0.022;
  param_ref[3] = 0.13;
  param_ref[4] = 0.101;
  param_ref[5] = -0.052;
  vector<int32_t> active;
  active.push_back(0);
  myFLOAT * p_predict1 = new myFLOAT[4*active.size()];
  myFLOAT * p_predict2 = new myFLOAT[4*active.size()];
  myFLOAT param1[6];
  myFLOAT param2[6];
  memcpy(param1,param_ref,6*sizeof(myFLOAT));
  for (int32_t i=0; i<6; i++) {
    memcpy(param2,param_ref,6*sizeof(myFLOAT));
    param2[i] += delta;
    cout << endl << "Checking parameter " << i << ":" << endl;
    cout << "param1: "; for (int32_t j=0; j<6; j++) cout << param1[j] << " "; cout << endl;
    cout << "param2: "; for (int32_t j=0; j<6; j++) cout << param2[j] << " "; cout << endl;
    stereoComputePredictionsAndJacobian(param2,active);
    memcpy(p_predict2,p_predict,4*active.size()*sizeof(myFLOAT));
    stereoComputePredictionsAndJacobian(param1,active);
    memcpy(p_predict1,p_predict,4*active.size()*sizeof(myFLOAT));
    for (int32_t j=0; j<4*(int32_t)active.size(); j++) {
      cout << "num: " << (p_predict2[j]-p_predict1[j])/delta;
      cout << ", ana: " << J[j*6+i] << endl;
    }
  }
  cout << "=================================" << endl;

  delete [] p_predict1;
  delete [] p_predict2;

}

Matrix VisualOdometry::getTransformation () {

  // extract parameters
  myFLOAT rx = param[0];
  myFLOAT ry = param[1];
  myFLOAT rz = param[2];
  myFLOAT tx = param[3];
  myFLOAT ty = param[4];
  myFLOAT tz = param[5];

  // precompute sine/cosine
  myFLOAT sx = sin(rx);
  myFLOAT cx = cos(rx);
  myFLOAT sy = sin(ry);
  myFLOAT cy = cos(ry);
  myFLOAT sz = sin(rz);
  myFLOAT cz = cos(rz);

  // compute transformation
  Matrix Tr(4,4);
  Tr.val[0][0] = +cy*cz;          Tr.val[0][1] = -cy*sz;          Tr.val[0][2] = +sy;    Tr.val[0][3] = tx;
  Tr.val[1][0] = +sx*sy*cz+cx*sz; Tr.val[1][1] = -sx*sy*sz+cx*cz; Tr.val[1][2] = -sx*cy; Tr.val[1][3] = ty;
  Tr.val[2][0] = -cx*sy*cz+sx*sz; Tr.val[2][1] = +cx*sy*sz+sx*cz; Tr.val[2][2] = +cx*cy; Tr.val[2][3] = tz;
  Tr.val[3][0] = 0;               Tr.val[3][1] = 0;               Tr.val[3][2] = 0;      Tr.val[3][3] = 1;
  return Tr;
}



