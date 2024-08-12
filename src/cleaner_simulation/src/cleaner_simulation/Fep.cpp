#include <cmath>
#include "cleaner_simulation/Fep.h"
#include <unsupported/Eigen/MatrixFunctions>
#include <iostream>

#include "cleaner_simulation/utils.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
using namespace std;
using namespace fep;
#define M_C 4.0
#define I_C 0.046
#define R 0.0375
#define d 0.114
#define M_W 0.046
#define I_W 3.23437e-05
#define p 1
#define dt 0.005
#define X_SIZE 2
#define V_SIZE 4

Eigen::MatrixXd A;
Eigen::MatrixXd B_x;
Eigen::MatrixXd B_v;
Eigen::SparseMatrix<double> D_x;
Eigen::SparseMatrix<double> D_v;
Eigen::SparseMatrix<double> Pi_w;
Eigen::SparseMatrix<double> Pi_z1;
Eigen::SparseMatrix<double> Pi_z2;
Eigen::MatrixXd dF_dxx;
Eigen::MatrixXd dF_dxv;
Eigen::MatrixXd dF_dvx;
Eigen::MatrixXd dF_dvv;

Eigen::MatrixXd H((p+1)*(X_SIZE+V_SIZE),(p+1)*(X_SIZE+V_SIZE)); //d(u_dot)/du
Eigen::MatrixXd U;
Eigen::VectorXd u_dot((p+1)*(X_SIZE+V_SIZE));
Eigen::VectorXd mu_x_tilde((p+1)*(X_SIZE));
Eigen::VectorXd mu_v_tilde((p+1)*(V_SIZE));
Eigen::VectorXd y_x((p+1)*(X_SIZE));
Eigen::VectorXd y_v((p+1)*(V_SIZE));

double s=0.002;
double l_w[X_SIZE]={10, 10}; //lambda_w 
double l_z[X_SIZE+V_SIZE]={50, 50, 50, 200, 150, 150}; //lambda_z
double FreeEnergy;
unsigned int mu_x_size,mu_v_size;

void sensory_embedding_x(deque<double> &left_wheelVel,deque<double> &right_wheelVel){
  y_x(0) = left_wheelVel[1];
  y_x(1) = right_wheelVel[1];
  y_x(2) = (left_wheelVel[1]-left_wheelVel[0])/dt;
  y_x(3) = (right_wheelVel[1]-right_wheelVel[0])/dt;
}
void sensory_embedding_v(deque<double>& chassis_angvel, deque<double>& chassis_accel, deque<double>& left_wheelTorque, deque<double> &right_wheelTorque){
  y_v(0) = (chassis_angvel[2]-chassis_angvel[1])/dt;
  y_v(1) = chassis_accel[1];
  y_v(2) = left_wheelTorque[1];
  y_v(3) = right_wheelTorque[1];
  y_v(4) = (chassis_angvel[2]-2*chassis_angvel[1]+chassis_angvel[0])/(dt*dt);
  y_v(5) = (chassis_accel[1]-chassis_accel[0])/dt;
  y_v(6) = (left_wheelTorque[1]-left_wheelTorque[0])/dt;
  y_v(7) = (right_wheelTorque[1]-right_wheelTorque[0])/dt;
}

void AI_Setup(){
    double k= 1/((M_W*R*R)+I_W);
    mu_x_size=mu_x_tilde.size();
    mu_v_size=mu_v_tilde.size();
    mu_x_tilde.setZero();
    mu_v_tilde.setZero();
    Eigen::MatrixXd tmp_A(X_SIZE,V_SIZE);
    tmp_A<<(I_C*R)/(2*d), -M_C*R/2.0, 1 , 0,
        (-I_C*R)/(2*d),  -M_C*R/2.0, 0 , 1;
    tmp_A = k*tmp_A;
    Eigen::MatrixXd I_p = Eigen::MatrixXd::Identity(p+1,p+1);
    A=kron(I_p,tmp_A);
    Eigen::MatrixXd tmp_B_v =  Eigen::MatrixXd::Identity(V_SIZE,V_SIZE);
    Eigen::MatrixXd tmp_B_x =  Eigen::MatrixXd::Identity(X_SIZE,X_SIZE);
    B_v = kron(I_p,tmp_B_v);
    B_x = kron(I_p,tmp_B_x);
    Eigen::MatrixXd tmp_D = ShiftMatrix(p+1);
    Eigen::MatrixXd tmp_x =  Eigen::MatrixXd::Identity(X_SIZE,X_SIZE);
    D_x = kron(tmp_D,tmp_x).sparseView();
    Eigen::MatrixXd tmp_v =  Eigen::MatrixXd::Identity(V_SIZE,V_SIZE);
    D_v = kron(tmp_D,tmp_v).sparseView();
 
    Eigen::MatrixXd S=temp_Cov_inv(1,s);
    Eigen::MatrixXd tmp_Pi_w(X_SIZE,X_SIZE);
    Eigen::MatrixXd tmp_Pi_z2(V_SIZE,V_SIZE);
    Eigen::MatrixXd tmp_Pi_z1(X_SIZE,X_SIZE);
    tmp_Pi_w << l_w[0], 0,
                0, l_w[1];
    tmp_Pi_z1 << l_z[0],0,
                0,l_z[1];
    tmp_Pi_z2 << l_z[2], 0, 0, 0,
                0, l_z[3], 0, 0,
                0, 0, l_z[4], 0,
                0, 0, 0, l_z[5];
    Pi_w = kron(S,tmp_Pi_w).sparseView();

    Pi_z1 = kron(S,tmp_Pi_z1).sparseView();

    Pi_z2 = kron(S,tmp_Pi_z2).sparseView();
    dF_dxx = D_x.transpose() * Pi_w * D_x + Pi_z1;

    dF_dvv = A.transpose() * Pi_w * A + Pi_z2;
    
    dF_dxv = D_x.transpose() * Pi_w * A;
    dF_dvx = A.transpose() * Pi_w * D_x;

    // cout<<H.cols()<<","<<H.rows()<<endl;

    H.block(0,0,dF_dxx.rows(),dF_dxx.cols())=D_x-dF_dxx;
    H.block(0,dF_dxx.cols(),dF_dxv.rows(),dF_dxv.cols())=-dF_dxv;
    H.block(dF_dxx.rows(),0,dF_dvx.rows(),dF_dvx.cols())=-dF_dvx;
    H.block(dF_dxx.rows(),dF_dxx.cols(),dF_dvv.rows(),dF_dvv.cols())=D_v-dF_dvv;
    Eigen::MatrixXd tmp_H=dt*H;
        
    U = (tmp_H.exp()-Eigen::MatrixXd::Identity(H.rows(),H.cols()))*H.inverse();
}
void AI_Update(){
    Eigen::VectorXd eps_x = D_x*mu_x_tilde-A*mu_v_tilde;
    Eigen::VectorXd eps_y1 = y_x-mu_x_tilde;
    Eigen::VectorXd eps_y2 = y_v-mu_v_tilde;
    cout<<"y_x:"<<y_x<<endl;
    cout<<"test1:"<<A*y_v<<endl;
    cout<<"y_v:"<<y_v<<endl;
    double noise_term = (p+1)*log(l_w[0]*l_w[1])+(p+1)*log(l_z[0]*l_z[1])+(p+1)*log(l_z[2]*l_z[3]*l_z[4]*l_z[5]);
    Eigen::MatrixXd E = eps_x.transpose()*Pi_w*eps_x + eps_y1.transpose()*Pi_z1*eps_y1 + eps_y2.transpose()*Pi_z2*eps_y2;
    FreeEnergy = E(0,0);
    Eigen::MatrixXd dF_dx= D_x.transpose()*(Pi_w*(eps_x))-Pi_z1*(eps_y1);
    Eigen::MatrixXd dF_dv= -A.transpose()*(Pi_w*(eps_x))-Pi_z2*(eps_y2);


    u_dot.segment(0,mu_x_size)= D_x*mu_x_tilde-dF_dx;
    u_dot.segment(mu_x_size,mu_v_size)= D_v*mu_v_tilde-dF_dv;
    Eigen::VectorXd delta_u = U*u_dot;
    mu_x_tilde = mu_x_tilde+delta_u.segment(0,mu_x_size);
    mu_v_tilde = mu_v_tilde+delta_u.segment(mu_x_size,mu_v_size);
}
