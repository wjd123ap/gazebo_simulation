#include <cmath>
#include "cleaner_simulation/cleaner.hpp"
#include <Eigen/Dense>
inline unsigned int Indexing(int index, int length) {
    return index < 0 ? index + length : index;
}

Eigen::MatrixXd createToeplitz(const Eigen::VectorXd& firstColumn) {
    int cols = firstColumn.size();
    
    Eigen::MatrixXd toeplitz(cols, cols);

    for (int i = 0; i < cols; ++i) {
        for (int j = 0; j < cols; ++j) {
            
            toeplitz(i, j) = firstColumn(Indexing(i - j,cols));
        }
    }

    return toeplitz;
}

Eigen::MatrixXd createToeplitz(const Eigen::VectorXd& firstColumn, const Eigen::VectorXd& firstRow) {
    int rows = firstColumn.size();
    int cols = firstRow.size(); 
    Eigen::MatrixXd toeplitz(rows, cols);

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            if (j >= i) {
                toeplitz(i, j) = firstRow(j - i);
            } else {
                toeplitz(i, j) = firstColumn(i - j);
            }
        }
    }

    return toeplitz;
}
