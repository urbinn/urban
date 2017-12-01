#ifndef ORBG2O_H
#define ORBG2O_H

#include <Eigen/StdVector>

int poseOptimization(Eigen::Ref<Eigen::MatrixXd> coords, Eigen::Ref<Eigen::MatrixXd> pose);

#endif // OPTIMIZER_H
