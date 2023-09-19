#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>
#include <moveit_compute_default_collisions.h>

namespace py = pybind11;


PYBIND11_MODULE(pymcdc, m) {
    py::class_<MoveitComputeDefaultCollisions>(m, "MoveitComputeDefaultCollisions")
        .def(py::init<>())
        .def("setVerbose", &MoveitComputeDefaultCollisions::setVerbose)
        .def("initFromPath", &MoveitComputeDefaultCollisions::initFromPath)
        .def("initFromString", &MoveitComputeDefaultCollisions::initFromString)
        .def("printDisabledCollisions", &MoveitComputeDefaultCollisions::print)
        .def("save", (bool (MoveitComputeDefaultCollisions::*)()) &MoveitComputeDefaultCollisions::save)
        .def("dump", (bool (MoveitComputeDefaultCollisions::*)(const std::string&)) &MoveitComputeDefaultCollisions::save)
        .def("computeDefaultCollisions", &MoveitComputeDefaultCollisions::computeDefaultCollisions)
        .def("getXmlString", &MoveitComputeDefaultCollisions::getXmlString);
}
