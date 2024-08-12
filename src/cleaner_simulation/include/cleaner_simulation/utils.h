#ifndef FEP_FUNCTIONS_H
#define FEP_FUNCTIONS_H

#include <Eigen/Dense>
#include <type_traits>
namespace fep {

    Eigen::VectorXd linspace(double start, double end, double step);
    Eigen::VectorXd arange(int end);
    Eigen::VectorXd arange(int start, int end, int step);

    Eigen::MatrixXd kron(double A, const Eigen::MatrixXd& B);
    Eigen::MatrixXd kron(const Eigen::MatrixXd& A, double B);
    Eigen::MatrixXd kron(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B);

    Eigen::VectorXd cumulativeProduct(Eigen::VectorXd v);

    Eigen::MatrixXd CovarianceMatrix(const Eigen::MatrixXd& data);


    Eigen::MatrixXd ShiftMatrix(unsigned int size);

    Eigen::MatrixXd temp_Cov_inv(unsigned int p, double s2);

} // namespace fep

#endif // FEP_FUNCTIONS_H