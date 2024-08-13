#include <Eigen/Dense>
#include <deque>

extern Eigen::VectorXd y_x;
extern Eigen::VectorXd y_v;
extern Eigen::VectorXd mu_v_tilde;
extern Eigen::VectorXd mu_x_tilde;
extern double FreeEnergy;
void sensory_embedding_x(std::deque<double> &left_wheelVel,std::deque<double> &right_wheelVel);
void sensory_embedding_v(std::deque<double>& chassis_accel, std::deque<double>& chassis_angvel, std::deque<double>& left_wheelTorque, std::deque<double> &right_wheelTorque);
void AI_Setup();
void AI_Update();