#include <cmath>
#include <iostream>
#include <random>
#include <omp.h>
#include <type_traits>
#include "cleaner_simulation/utils.h"
using namespace std;
namespace fep{


Eigen::VectorXd linspace(double start, double end, double step) {
    int size = static_cast<int>((end - start) / step + 1e-9); 
    Eigen::VectorXd array(size);
    for (int i = 0; i < size; ++i) {
        array[i] = start + i * step;
    }
    return array;
}
Eigen::VectorXd arange(int end) {

    Eigen::VectorXd array(end);
    for (int i = 0; i < end; ++i) {
        array[i] = i;
    }
    return array;
}
Eigen::VectorXd arange(int start, int end, int step) {
    int size = static_cast<int>((end - start) / step + 1e-9); 
    Eigen::VectorXd array(size);
    for (int i = 0; i < size; ++i) {
        array[i] = start + i * step;
    }
    return array;
}

Eigen::MatrixXd kron(double A, const Eigen::MatrixXd& B){

    Eigen::MatrixXd kronProd(B.rows(), B.cols());
    kronProd=A*B;
    return kronProd;
}

Eigen::MatrixXd kron(const Eigen::MatrixXd& A, double B){

    Eigen::MatrixXd kronProd(A.rows(), A.cols());
    kronProd=A*B;
    return kronProd;
}

Eigen::MatrixXd kron(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B){
    int A_rows=A.rows();
    int A_cols=A.cols();
    int B_rows=B.rows();
    int B_cols=B.cols();
    Eigen::MatrixXd kronProd(A.rows()*B.rows(), A.cols()*B.cols());
    for (int i = 0; i < A_rows; ++i) {
        for (int j = 0; j < A_cols; ++j) {
            kronProd.block(i*B_rows,j*B_cols,B.rows(),B.cols())=A(i,j)*B;
        }
    }
    return kronProd;
}

Eigen::VectorXd cumulativeProduct(Eigen::VectorXd v) {
    Eigen::VectorXd cumProd(v.size());
    cumProd(0) = v(0);  // 첫 번째 요소는 변경 없음

    for (int i = 1; i < v.size(); i++) {
        cumProd(i) = cumProd(i - 1) * v(i);
    }

    return cumProd;
}


Eigen::MatrixXd CovarianceMatrix(const Eigen::MatrixXd& data) {
    // 데이터 행렬의 열별 평균 계산
    Eigen::VectorXd mean = data.rowwise().mean();

    // 평균을 빼서 중심화
    Eigen::MatrixXd centered = data.colwise() - mean;
    cout<<centered.cols()<<endl;
    cout<<centered.rows()<<endl;
    // 공분산 행렬 계산 (비편향 추정량)
    int n = data.cols() ;  // 샘플 크기 보정
    Eigen::MatrixXd cov = ( centered * centered.adjoint()) / n;

    return cov;
}



Eigen::MatrixXd ShiftMatrix(unsigned int size){
    Eigen::MatrixXd shiftMatrix = Eigen::MatrixXd::Zero(size, size);
    for (int i = 0; i < size-1; ++i) {

        shiftMatrix(i, i+1) = 1;
        
    }
    return shiftMatrix;
}

Eigen::MatrixXd temp_Cov_inv(unsigned int p,double s2){
    Eigen::MatrixXd Mat;
    switch (p)
    {
    case 1:
        Mat.resize(p+1,p+1);
        Mat<<1,0,
            0,2*s2;
        break;
    case 2:
        Mat.resize(p+1,p+1);
        Mat<<3/2,0,s2,
        0,2*s2,0,
        s2,0, 2*pow(s2,2);
        break;
    case 3:
        Mat.resize(p+1,p+1);
        Mat<<3/2,0,s2, 0,
        0,5*s2,0, 2*pow(s2,2),
        0, 2*pow(s2,2),0,4*pow(s2,3)/3;
        break;
    case 4:
        Mat.resize(p+1,p+1);
        Mat<<15/8,0,5*s2/2, 0, pow(s2,2)/2,
        0,5*s2,0, 2*pow(s2,2), 0,
        5*pow(s2,2)/2,0,8*pow(s2,2), 0, 2*pow(s2,3),
        0, 2*pow(s2,2), 0, 4*pow(s2,3)/3, 0,
        pow(s2,2)/2, 0, 2*pow(s2,3), 0, 2*pow(s2,4)/3;
        break;
    case 5:
        Mat.resize(p+1,p+1);
        Mat<<15/8,0,5*s2/2, 0, pow(s2,2)/2, 0,
        0,35*s2/4,0, 7*pow(s2,2), 0, pow(s2,3),
        5*pow(s2,2)/2,0,8*pow(s2,2), 0, 2*pow(s2,3), 0,
        0, 7*pow(s2,2), 0, 8*pow(s2,3), 0, 4*pow(s2,4)/3,
        pow(s2,2)/2, 0, 2*pow(s2,3), 0, 2*pow(s2,4)/3, 0,
        0, pow(s2,3), 0, 4*pow(s2,4)/3, 0, 4*pow(s2,5)/15;
        break;

    default:
        Mat.resize(6,6);
        Mat<<15/8,0,5*s2/2, 0, pow(s2,2)/2, 0,
        0,35*s2/4,0, 7*pow(s2,2), 0, pow(s2,3),
        5*pow(s2,2)/2,0,8*pow(s2,2), 0, 2*pow(s2,3), 0,
        0, 7*pow(s2,2), 0, 8*pow(s2,3), 0, 4*pow(s2,4)/3,
        pow(s2,2)/2, 0, 2*pow(s2,3), 0, 2*pow(s2,4)/3, 0,
        0, pow(s2,3), 0, 4*pow(s2,4)/3, 0, 4*pow(s2,5)/15;
        break;
    }
    return Mat;
}

}
