#include <pybind11/pybind11.h>
#include <jaka_kinematics/jaka_kinematics.h>
#include <pybind11/stl.h>
#include <assert.h>

PYBIND11_MODULE(jaka_kinematics_pybind11, m)
{
    pybind11::class_<jaka_kinematics>(m, "jaka_kinematics")
        .def(pybind11::init())
        .def(pybind11::init<const std::vector<double>>())
        .def("forward", &jaka_kinematics::forward_vector, pybind11::arg("joints"), "forward kinematics function", pybind11::return_value_policy::reference)
        .def("inverse", &jaka_kinematics::inverse_vector, pybind11::arg("matrix"), pybind11::arg("q6_des") = 0, "inverse kinematics function", pybind11::return_value_policy::reference);
}
