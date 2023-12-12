#include <string>
#include <iostream>
#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include "roboflex_core/core.h"
#include "roboflex_dvs/dvs.h"

namespace py = pybind11;

using namespace roboflex;
using namespace roboflex::dvs;


PYBIND11_MODULE(roboflex_dvs_ext, m) {
    m.doc() = "roboflex_dvs_ext";

    py::class_<DVSRawData, core::Message, std::shared_ptr<DVSRawData>>(m, "DVSRawData")
        .def(py::init([](const std::shared_ptr<core::Message> o) {
            return std::make_shared<DVSRawData>(*o); }),
            "Construct a DVSRawData from a core message",
            py::arg("other"))
        .def_property_readonly("t0", &DVSRawData::get_t0)
        .def_property_readonly("t1", &DVSRawData::get_t1)
        .def("__repr__",  &DVSRawData::to_string)
    ;

    py::class_<DVSEigenData, core::Message, std::shared_ptr<DVSEigenData>>(m, "DVSEigenData")
        .def(py::init([](const std::shared_ptr<core::Message> o) {
            return std::make_shared<DVSEigenData>(*o); }),
            "Construct a DVSEigenData from a core message",
            py::arg("other"))
        .def("on", &DVSEigenData::get_on_events)
        .def("off", &DVSEigenData::get_off_events)
        .def_property_readonly("t", &DVSEigenData::get_t)
        .def_property_readonly("t0", &DVSEigenData::get_t0)
        .def_property_readonly("t1", &DVSEigenData::get_t1)
        .def("__repr__", &DVSEigenData::to_string)

        // .def(py::pickle(
        //     [](const DVSEigenData &d) { // __getstate__
        //         /* Return a tuple that fully encodes the state of the object */
        //         return py::make_tuple(d.get_on_events(), d.get_off_events(), d.get_t(), d.get_t0(), d.get_t1());
        //     },
        //     [](py::tuple t) { // __setstate__
        //         if (t.size() != 5)
        //             throw std::runtime_error("Invalid state!");

        //         /* Create a new C++ instance */
        //         DVSEigenData d(
        //             t[0].cast<DVSEigenData::DVSFrame>(),
        //             t[1].cast<DVSEigenData::DVSFrame>(),
        //             t[2].cast<double>(),
        //             t[3].cast<double>(),
        //             t[4].cast<double>());

        //         return d;
        //     }
        // ))
    ;

    py::class_<DVSSensor, core::RunnableNode, std::shared_ptr<DVSSensor>>(m, "DVSSensor")
        .def(py::init<const std::string &>(),
            "Create a DVS sensor that outputs raw, unparsed data",
            py::arg("name") = "dvs_sensor")
    ;

    py::class_<DVSEncoder, core::Node, std::shared_ptr<DVSEncoder>>(m, "DVSEncoder")
        .def(py::init<const std::string &>(),
            "Create a transformer that consumes DVSRawData and emits DVSEigenData.",
            py::arg("name") = "dvs_encoder")
    ;

    py::class_<DVSEigenToGrayScale, nodes::FrequencyGenerator, std::shared_ptr<DVSEigenToGrayScale>>(m, "DVSEigenToGrayScale")
        .def(py::init<float, const std::string &>(),
            "Consumes DVSEigenData and periodically emits a grayscale image as a TensorMessage under the key \"image\"",
            py::arg("emit_frequency_hz") = 24.0,
            py::arg("name") = "DVSEigenToGrayScale")
    ;
}
