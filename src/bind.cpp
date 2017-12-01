#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include "ndarray_converter.h"

#include <Eigen/LU>
#include <Eigen/StdVector>

#include "urbg2o11.h"

namespace py = pybind11;

PYBIND11_PLUGIN(urbg2o)
{
    NDArrayConverter::init_numpy();

    py::module m("urbg2o", "pybind11 opencv example plugin");

    m.def("poseOptimization", &poseOptimization, "pose-only bundle adjustment",
	py::arg("coords").noconvert(), py::arg("pose"));

    return m.ptr();
}

