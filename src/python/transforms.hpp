#pragma once

#include <pybind11/pybind11.h>
#include <pybind11/chrono.h>
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/operators.h>
#include <pybind11/stl.h>

#include <motion3d/transforms.hpp>

namespace m3d = motion3d;
namespace py = pybind11;

void init_transforms(py::module &m)
{
  /************************************************
   *  Exceptions
   ***********************************************/
  py::register_exception<m3d::InvalidEulerAxesException>(m, "InvalidEulerAxesException");
  py::register_exception<m3d::InvalidTransformException>(m, "InvalidTransformException");
  py::register_exception<m3d::InvalidTransformTypeException>(m, "InvalidTransformTypeException");
  py::register_exception<m3d::InvalidTypeSizeException>(m, "InvalidTypeSizeException");


  /************************************************
   *  EulerAxes
   ***********************************************/
  py::enum_<m3d::EulerAxes>(m, "EulerAxes",
      ":cpp:enum:`motion3d::EulerAxes`")

    .value("kSXYZ", m3d::EulerAxes::kSXYZ)
    .value("kSXYX", m3d::EulerAxes::kSXYX)
    .value("kSXZY", m3d::EulerAxes::kSXZY)
    .value("kSXZX", m3d::EulerAxes::kSXZX)
    .value("kSYZX", m3d::EulerAxes::kSYZX)
    .value("kSYZY", m3d::EulerAxes::kSYZY)
    .value("kSYXZ", m3d::EulerAxes::kSYXZ)
    .value("kSYXY", m3d::EulerAxes::kSYXY)
    .value("kSZXY", m3d::EulerAxes::kSZXY)
    .value("kSZXZ", m3d::EulerAxes::kSZXZ)
    .value("kSZYX", m3d::EulerAxes::kSZYX)
    .value("kSZYZ", m3d::EulerAxes::kSZYZ)
    .value("kRZYX", m3d::EulerAxes::kRZYX)
    .value("kRXYX", m3d::EulerAxes::kRXYX)
    .value("kRYZX", m3d::EulerAxes::kRYZX)
    .value("kRXZX", m3d::EulerAxes::kRXZX)
    .value("kRXZY", m3d::EulerAxes::kRXZY)
    .value("kRYZY", m3d::EulerAxes::kRYZY)
    .value("kRZXY", m3d::EulerAxes::kRZXY)
    .value("kRYXY", m3d::EulerAxes::kRYXY)
    .value("kRYXZ", m3d::EulerAxes::kRYXZ)
    .value("kRZXZ", m3d::EulerAxes::kRZXZ)
    .value("kRXYZ", m3d::EulerAxes::kRXYZ)
    .value("kRZYZ", m3d::EulerAxes::kRZYZ)

    .def_static("FromStr", &m3d::eulerAxesFromStr,
        ":cpp:func:`motion3d::eulerAxesFromStr()`")

    .def("toStr", [](const m3d::EulerAxes& self) { return m3d::eulerAxesToStr(self); },
        ":cpp:func:`motion3d::eulerAxesToStr()`");


  /************************************************
   *  TransformType
   ***********************************************/
  py::enum_<m3d::TransformType>(m, "TransformType",
      ":cpp:enum:`motion3d::TransformType`")

    .value("kAxisAngle", m3d::TransformType::kAxisAngle)
    .value("kDualQuaternion", m3d::TransformType::kDualQuaternion)
    .value("kEuler", m3d::TransformType::kEuler)
    .value("kMatrix", m3d::TransformType::kMatrix)
    .value("kQuaternion", m3d::TransformType::kQuaternion)

    .def_static("FromChar", &m3d::transformTypeFromChar,
        ":cpp:func:`motion3d::transformTypeFromChar()`")

    .def("toChar", [](const m3d::TransformType& self) { return m3d::transformTypeToChar(self); },
        ":cpp:func:`motion3d::transformTypeToChar()`")

    .def("getVectorSize", [](const m3d::TransformType& self) { return m3d::getVectorSize(self); },
        ":cpp:func:`motion3d::getVectorSize()`")

    .def("getBinarySize", [](const m3d::TransformType& self) { return m3d::getBinarySize(self); },
        ":cpp:func:`motion3d::getBinarySize()`");


  /************************************************
   *  TransformInterface
   ***********************************************/
  py::class_<m3d::TransformInterface, m3d::TransformInterface::Ptr>(m, "TransformInterface",
      ":cpp:class:`motion3d::TransformInterface`")

    .def("copy", &m3d::TransformInterface::copy,
        ":cpp:func:`motion3d::TransformInterface::copy()`")

    .def("identity", &m3d::TransformInterface::identity,
        ":cpp:func:`motion3d::TransformInterface::identity()`")

    .def("isType", &m3d::TransformInterface::isType,
        py::arg("other"),
        ":cpp:func:`motion3d::TransformInterface::isType()`")

    .def("asType", static_cast<m3d::TransformInterface::Ptr (m3d::TransformInterface::*)(const m3d::TransformType&) const>(&m3d::TransformInterface::asType),
        py::arg("type"),
        ":cpp:func:`motion3d::TransformInterface::asType()`")

    .def("isEqual", py::overload_cast<const m3d::TransformInterface::ConstPtr&, double>(&m3d::TransformInterface::isEqual, py::const_),
        py::arg("other"), py::arg("eps") = m3d::kDefaultEps,
        ":cpp:func:`motion3d::TransformInterface::isEqual()`")

    .def_static("Factory", &m3d::TransformInterface::Factory<>,
        py::arg("type"),
        ":cpp:func:`motion3d::TransformInterface::Factory()`")

    .def_static("Factory", &m3d::TransformInterface::Factory<const std::vector<double>&, bool>,
        py::arg("type"), py::arg("vector"), py::arg("unsafe") = false,
        ":cpp:func:`motion3d::TransformInterface::Factory()`")

    .def_static("Factory",
        [](const m3d::TransformType &type, const Eigen::Ref<const Eigen::VectorXd> &matrix, bool unsafe) {
          return m3d::TransformInterface::Factory(type, matrix, unsafe);
        },
        py::arg("type"), py::arg("matrix"), py::arg("unsafe") = false,
        ":cpp:func:`motion3d::TransformInterface::Factory()`");


  /************************************************
   *  AxisAngleTransform
   ***********************************************/
  py::class_<m3d::AxisAngleTransform, m3d::TransformInterface, m3d::AxisAngleTransform::Ptr>(m, "AxisAngleTransform",
      ":cpp:class:`motion3d::AxisAngleTransform`")

    .def(py::init<>(),
        ":cpp:func:`motion3d::AxisAngleTransform::AxisAngleTransform()`")

    .def(py::init<const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>&, double, const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>&, bool>(),
        py::arg("translation"), py::arg("angle"), py::arg("axis"), py::arg("unsafe") = false,
        ":cpp:func:`motion3d::AxisAngleTransform::AxisAngleTransform()`")

    .def(py::init<const Eigen::Ref<const Eigen::Matrix<double, 7, 1>>&, bool>(),
        py::arg("data"), py::arg("unsafe") = false,
        ":cpp:func:`motion3d::AxisAngleTransform::AxisAngleTransform()`")

    .def("copy", [](const m3d::AxisAngleTransform &self) { return m3d::AxisAngleTransform(self); },
        "Call copy constructor :cpp:func:`motion3d::AxisAngleTransform::AxisAngleTransform()`.")

    .def("setIdentity", &m3d::AxisAngleTransform::setIdentity,
        ":cpp:func:`motion3d::AxisAngleTransform::setIdentity()`")

    .def("toList", &m3d::AxisAngleTransform::toVector,
        ":cpp:func:`motion3d::AxisAngleTransform::toVector()`")

    .def("toBinary", &m3d::AxisAngleTransform::toBinary,
        ":cpp:func:`motion3d::TransformInterface::toBinary()`")

    .def("toArray", &m3d::AxisAngleTransform::toEigenVector,
        ":cpp:func:`motion3d::AxisAngleTransform::toEigenVector()`")

    .def("isUnsafe", &m3d::AxisAngleTransform::isUnsafe,
        ":cpp:func:`motion3d::TransformInterface::isUnsafe()`")

    .def("isValid", py::overload_cast<double>(&m3d::AxisAngleTransform::isValid, py::const_),
        py::arg("eps") = m3d::kDefaultEps,
        ":cpp:func:`motion3d::AxisAngleTransform::isValid()`")

    .def("inverse_", &m3d::AxisAngleTransform::inverse_,
        ":cpp:func:`motion3d::AxisAngleTransform::inverse_()`")

    .def("inverse", &m3d::AxisAngleTransform::inverse,
        ":cpp:func:`motion3d::AxisAngleTransform::inverse()`")

    .def("normalized_", &m3d::AxisAngleTransform::normalized_,
        ":cpp:func:`motion3d::AxisAngleTransform::normalized_()`")

    .def("normalized", &m3d::AxisAngleTransform::normalized,
        ":cpp:func:`motion3d::AxisAngleTransform::normalized()`")

    .def("scaleTranslation_", &m3d::AxisAngleTransform::scaleTranslation_,
        py::arg("factor"),
        ":cpp:func:`motion3d::AxisAngleTransform::scaleTranslation_()`")

    .def("scaleTranslation", &m3d::AxisAngleTransform::scaleTranslation,
        py::arg("factor"),
        ":cpp:func:`motion3d::AxisAngleTransform::scaleTranslation()`")

    .def("applyPre_", &m3d::AxisAngleTransform::applyPre_<m3d::DualQuaternionTransform>,
        py::arg("other"),
        ":cpp:func:`motion3d::AxisAngleTransform::applyPre_()`")

    .def("applyPre", &m3d::AxisAngleTransform::applyPre<m3d::DualQuaternionTransform>,
        py::arg("other"),
        ":cpp:func:`motion3d::AxisAngleTransform::applyPre()`")

    .def("applyPost_", &m3d::AxisAngleTransform::applyPost_<m3d::DualQuaternionTransform>,
        py::arg("other"),
        ":cpp:func:`motion3d::AxisAngleTransform::applyPost_()`")

    .def("applyPost", &m3d::AxisAngleTransform::applyPost<m3d::DualQuaternionTransform>,
        py::arg("other"),
        ":cpp:func:`motion3d::AxisAngleTransform::applyPost()`")

    .def("applyPre_", &m3d::AxisAngleTransform::applyPre_<m3d::TransformInterface::Ptr>,
        py::arg("other"),
        ":cpp:func:`motion3d::AxisAngleTransform::applyPre_()`")

    .def("applyPre", &m3d::AxisAngleTransform::applyPre<m3d::TransformInterface::Ptr>,
        py::arg("other"),
        ":cpp:func:`motion3d::AxisAngleTransform::applyPre()`")

    .def("applyPost_", &m3d::AxisAngleTransform::applyPost_<m3d::TransformInterface::Ptr>,
        py::arg("other"),
        ":cpp:func:`motion3d::AxisAngleTransform::applyPost_()`")

    .def("applyPost", &m3d::AxisAngleTransform::applyPost<m3d::TransformInterface::Ptr>,
        py::arg("other"),
        ":cpp:func:`motion3d::AxisAngleTransform::applyPost()`")

    .def("transformPoint", 
        [](const m3d::AxisAngleTransform &self, const Eigen::Ref<const Eigen::Vector3d> &point) {
          return self.transformPoint(point);
        },
        py::arg("point"),
        ":cpp:func:`motion3d::AxisAngleTransform::transformPoint()`")

    .def("transformCloud", 
        [](const m3d::AxisAngleTransform &self, const Eigen::Ref<const Eigen::Matrix<double, 3, Eigen::Dynamic>> &cloud) {
          return self.transformCloud(cloud);
        },
        py::arg("cloud"),
        ":cpp:func:`motion3d::AxisAngleTransform::transformCloud()`")

    .def("rotationNorm", &m3d::AxisAngleTransform::rotationNorm,
        ":cpp:func:`motion3d::AxisAngleTransform::rotationNorm()`")

    .def("translationNorm", &m3d::AxisAngleTransform::translationNorm,
        ":cpp:func:`motion3d::AxisAngleTransform::translationNorm()`")

    .def("getTranslation", &m3d::AxisAngleTransform::getTranslation,
        ":cpp:func:`motion3d::AxisAngleTransform::getTranslation()`")

    .def("getAngle", &m3d::AxisAngleTransform::getAngle,
        ":cpp:func:`motion3d::AxisAngleTransform::getAngle()`")

    .def("getAxis", &m3d::AxisAngleTransform::getAxis,
        ":cpp:func:`motion3d::AxisAngleTransform::getAxis()`")

    .def("setTranslation",
        [](m3d::AxisAngleTransform &self, const Eigen::Ref<const Eigen::Vector3d> &translation, const bool unsafe) -> m3d::AxisAngleTransform& {
          return self.setTranslation(translation, unsafe);
        },
        py::arg("translation"), py::arg("unsafe") = false,
        ":cpp:func:`motion3d::AxisAngleTransform::setTranslation()`")

    .def("setAngle", &m3d::AxisAngleTransform::setAngle,
        py::arg("angle"), py::arg("unsafe") = false,
        ":cpp:func:`motion3d::AxisAngleTransform::setAngle()`")

    .def("setAxis",
        [](m3d::AxisAngleTransform &self, const Eigen::Ref<const Eigen::Vector3d> &axis, const bool unsafe) -> m3d::AxisAngleTransform& {
          return self.setAxis(axis, unsafe);
        },
        py::arg("axis"), py::arg("unsafe") = false,
        ":cpp:func:`motion3d::AxisAngleTransform::setAxis()`")

    .def("__repr__", &m3d::AxisAngleTransform::desc,
        ":cpp:func:`motion3d::AxisAngleTransform::desc()`")

    .def(py::self * m3d::AxisAngleTransform())
    .def(py::self * m3d::DualQuaternionTransform())
    .def(py::self * m3d::EulerTransform())
    .def(py::self * m3d::MatrixTransform())
    .def(py::self * m3d::QuaternionTransform())
    .def(py::self *= m3d::AxisAngleTransform())
    .def(py::self *= m3d::DualQuaternionTransform())
    .def(py::self *= m3d::EulerTransform())
    .def(py::self *= m3d::MatrixTransform())
    .def(py::self *= m3d::QuaternionTransform())
    .def(py::self / m3d::AxisAngleTransform())
    .def(py::self / m3d::DualQuaternionTransform())
    .def(py::self / m3d::EulerTransform())
    .def(py::self / m3d::MatrixTransform())
    .def(py::self / m3d::QuaternionTransform())
    .def(py::self /= m3d::AxisAngleTransform())
    .def(py::self /= m3d::DualQuaternionTransform())
    .def(py::self /= m3d::EulerTransform())
    .def(py::self /= m3d::MatrixTransform())
    .def(py::self /= m3d::QuaternionTransform());


  /************************************************
   *  DualQuaternionTransform
   ***********************************************/
  py::class_<m3d::DualQuaternionTransform, m3d::TransformInterface, m3d::DualQuaternionTransform::Ptr>(m, "DualQuaternionTransform",
      ":cpp:class:`motion3d::DualQuaternionTransform`")

    .def(py::init<>(),
        ":cpp:func:`motion3d::DualQuaternionTransform::DualQuaternionTransform()`")

    .def(py::init<const m3d::Quaterniond&, const m3d::Quaterniond&, bool>(),
        py::arg("real"), py::arg("dual"), py::arg("unsafe") = false,
        ":cpp:func:`motion3d::DualQuaternionTransform::DualQuaternionTransform()`")

    .def(py::init<const Eigen::Ref<const Eigen::Matrix<double, 4, 1>>&, const Eigen::Ref<const Eigen::Matrix<double, 4, 1>>&, bool>(),
        py::arg("real"), py::arg("dual"), py::arg("unsafe") = false,
        ":cpp:func:`motion3d::DualQuaternionTransform::DualQuaternionTransform()`")

    .def(py::init<const Eigen::Ref<const Eigen::Matrix<double, 8, 1>>&, bool>(),
        py::arg("data"), py::arg("unsafe") = false,
        ":cpp:func:`motion3d::DualQuaternionTransform::DualQuaternionTransform()`")

    .def("copy", [](const m3d::DualQuaternionTransform &self) { return m3d::DualQuaternionTransform(self); },
        "Call copy constructor :cpp:func:`motion3d::DualQuaternionTransform::DualQuaternionTransform()`.")

    .def("setIdentity", &m3d::DualQuaternionTransform::setIdentity,
        ":cpp:func:`motion3d::DualQuaternionTransform::setIdentity()`")

    .def("toList", &m3d::DualQuaternionTransform::toVector,
        ":cpp:func:`motion3d::DualQuaternionTransform::toVector()`")

    .def("toBinary", &m3d::DualQuaternionTransform::toBinary,
        ":cpp:func:`motion3d::TransformInterface::toBinary()`")

    .def("toArray", &m3d::DualQuaternionTransform::toEigenVector,
        ":cpp:func:`motion3d::DualQuaternionTransform::toEigenVector()`")

    .def("isUnsafe", &m3d::DualQuaternionTransform::isUnsafe,
        ":cpp:func:`motion3d::TransformInterface::isUnsafe()`")

    .def("isValid", py::overload_cast<double>(&m3d::DualQuaternionTransform::isValid, py::const_),
        py::arg("eps") = m3d::kDefaultEps,
        ":cpp:func:`motion3d::DualQuaternionTransform::isValid()`")

    .def("inverse_", &m3d::DualQuaternionTransform::inverse_,
        ":cpp:func:`motion3d::DualQuaternionTransform::inverse_()`")

    .def("inverse", &m3d::DualQuaternionTransform::inverse,
        ":cpp:func:`motion3d::DualQuaternionTransform::inverse()`")

    .def("normalized_", &m3d::DualQuaternionTransform::normalized_,
        ":cpp:func:`motion3d::DualQuaternionTransform::normalized_()`")

    .def("normalized", &m3d::DualQuaternionTransform::normalized,
        ":cpp:func:`motion3d::DualQuaternionTransform::normalized()`")

    .def("scaleTranslation_", &m3d::DualQuaternionTransform::scaleTranslation_,
        py::arg("factor"),
        ":cpp:func:`motion3d::DualQuaternionTransform::scaleTranslation_()`")

    .def("scaleTranslation", &m3d::DualQuaternionTransform::scaleTranslation,
        py::arg("factor"),
        ":cpp:func:`motion3d::DualQuaternionTransform::scaleTranslation()`")

    .def("applyPre_", &m3d::DualQuaternionTransform::applyPre_<m3d::DualQuaternionTransform>,
        py::arg("other"),
        ":cpp:func:`motion3d::DualQuaternionTransform::applyPre_()`")

    .def("applyPre", &m3d::DualQuaternionTransform::applyPre<m3d::DualQuaternionTransform>,
        py::arg("other"),
        ":cpp:func:`motion3d::DualQuaternionTransform::applyPre()`")

    .def("applyPost_", &m3d::DualQuaternionTransform::applyPost_<m3d::DualQuaternionTransform>,
        py::arg("other"),
        ":cpp:func:`motion3d::DualQuaternionTransform::applyPost_()`")

    .def("applyPost", &m3d::DualQuaternionTransform::applyPost<m3d::DualQuaternionTransform>,
        py::arg("other"),
        ":cpp:func:`motion3d::DualQuaternionTransform::applyPost()`")

    .def("applyPre_", &m3d::DualQuaternionTransform::applyPre_<m3d::TransformInterface::Ptr>,
        py::arg("other"),
        ":cpp:func:`motion3d::DualQuaternionTransform::applyPre_()`")

    .def("applyPre", &m3d::DualQuaternionTransform::applyPre<m3d::TransformInterface::Ptr>,
        py::arg("other"),
        ":cpp:func:`motion3d::DualQuaternionTransform::applyPre()`")

    .def("applyPost_", &m3d::DualQuaternionTransform::applyPost_<m3d::TransformInterface::Ptr>,
        py::arg("other"),
        ":cpp:func:`motion3d::DualQuaternionTransform::applyPost_()`")

    .def("applyPost", &m3d::DualQuaternionTransform::applyPost<m3d::TransformInterface::Ptr>,
        py::arg("other"),
        ":cpp:func:`motion3d::DualQuaternionTransform::applyPost()`")
        
    .def("transformPoint", 
        [](const m3d::DualQuaternionTransform &self, const Eigen::Ref<const Eigen::Vector3d> &point) {
          return self.transformPoint(point);
        },
        py::arg("point"),
        ":cpp:func:`motion3d::DualQuaternionTransform::transformPoint()`")

    .def("transformCloud", 
        [](const m3d::DualQuaternionTransform &self, const Eigen::Ref<const Eigen::Matrix<double, 3, Eigen::Dynamic>> &cloud) {
          return self.transformCloud(cloud);
        },
        py::arg("cloud"),
        ":cpp:func:`motion3d::DualQuaternionTransform::transformCloud()`")

    .def("rotationNorm", &m3d::DualQuaternionTransform::rotationNorm,
        ":cpp:func:`motion3d::DualQuaternionTransform::rotationNorm()`")

    .def("translationNorm", &m3d::DualQuaternionTransform::translationNorm,
        ":cpp:func:`motion3d::DualQuaternionTransform::translationNorm()`")

    .def("getDualQuaternion", &m3d::DualQuaternionTransform::getDualQuaternion,
        ":cpp:func:`motion3d::DualQuaternionTransform::getDualQuaternion()`")

    .def("getReal", &m3d::DualQuaternionTransform::getReal,
        ":cpp:func:`motion3d::DualQuaternionTransform::getReal()`")

    .def("getDual", &m3d::DualQuaternionTransform::getDual,
        ":cpp:func:`motion3d::DualQuaternionTransform::getDual()`")

    .def("setDualQuaternion", &m3d::DualQuaternionTransform::setDualQuaternion,
        py::arg("dq"), py::arg("unsafe") = false,
        ":cpp:func:`motion3d::DualQuaternionTransform::setDualQuaternion()`")

    .def("setReal", &m3d::DualQuaternionTransform::setReal,
        py::arg("real"), py::arg("unsafe") = false,
        ":cpp:func:`motion3d::DualQuaternionTransform::setReal()`")

    .def("setDual", &m3d::DualQuaternionTransform::setDual,
        py::arg("dual"), py::arg("unsafe") = false,
        ":cpp:func:`motion3d::DualQuaternionTransform::setDual()`")

    .def("__repr__", &m3d::DualQuaternionTransform::desc,
        ":cpp:func:`motion3d::DualQuaternionTransform::desc()`")

    .def(py::self * m3d::AxisAngleTransform())
    .def(py::self * m3d::DualQuaternionTransform())
    .def(py::self * m3d::EulerTransform())
    .def(py::self * m3d::MatrixTransform())
    .def(py::self * m3d::QuaternionTransform())
    .def(py::self *= m3d::AxisAngleTransform())
    .def(py::self *= m3d::DualQuaternionTransform())
    .def(py::self *= m3d::EulerTransform())
    .def(py::self *= m3d::MatrixTransform())
    .def(py::self *= m3d::QuaternionTransform())
    .def(py::self / m3d::AxisAngleTransform())
    .def(py::self / m3d::DualQuaternionTransform())
    .def(py::self / m3d::EulerTransform())
    .def(py::self / m3d::MatrixTransform())
    .def(py::self / m3d::QuaternionTransform())
    .def(py::self /= m3d::AxisAngleTransform())
    .def(py::self /= m3d::DualQuaternionTransform())
    .def(py::self /= m3d::EulerTransform())
    .def(py::self /= m3d::MatrixTransform())
    .def(py::self /= m3d::QuaternionTransform());


  /************************************************
   *  EulerTransform
   ***********************************************/
  py::class_<m3d::EulerTransform, m3d::TransformInterface, m3d::EulerTransform::Ptr>(m, "EulerTransform",
      ":cpp:class:`motion3d::EulerTransform`")

    .def(py::init<>(),
        ":cpp:func:`motion3d::EulerTransform::EulerTransform()`")

    .def(py::init<const m3d::AxisAngleTransform&, m3d::EulerAxes>(),
        py::arg("obj"), py::arg("axes"),
        ":cpp:func:`motion3d::EulerTransform::EulerTransform()`")

    .def(py::init<const m3d::DualQuaternionTransform&, m3d::EulerAxes>(),
        py::arg("obj"), py::arg("axes"),
        ":cpp:func:`motion3d::EulerTransform::EulerTransform()`")

    .def(py::init<const m3d::EulerTransform&, m3d::EulerAxes>(),
        py::arg("obj"), py::arg("axes"),
        ":cpp:func:`motion3d::EulerTransform::EulerTransform()`")

    .def(py::init<const m3d::MatrixTransform&, m3d::EulerAxes>(),
        py::arg("obj"), py::arg("axes"),
        ":cpp:func:`motion3d::EulerTransform::EulerTransform()`")

    .def(py::init<const m3d::QuaternionTransform&, m3d::EulerAxes>(),
        py::arg("obj"), py::arg("axes"),
        ":cpp:func:`motion3d::EulerTransform::EulerTransform()`")

    .def(py::init<const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>&, double, double, double, m3d::EulerAxes, bool>(),
        py::arg("translation"), py::arg("ai"), py::arg("aj"), py::arg("ak"), py::arg("axes") = m3d::kEulerAxesDefault, py::arg("unsafe") = false,
        ":cpp:func:`motion3d::EulerTransform::EulerTransform()`")

    .def(py::init<const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>&, const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>&, m3d::EulerAxes, bool>(),
        py::arg("translation"), py::arg("angles"), py::arg("axes") = m3d::kEulerAxesDefault, py::arg("unsafe") = false,
        ":cpp:func:`motion3d::EulerTransform::EulerTransform()`")

    .def(py::init<const Eigen::Ref<const Eigen::Matrix<double, 7, 1>>&, bool>(),
        py::arg("data"), py::arg("unsafe") = false,
        ":cpp:func:`motion3d::EulerTransform::EulerTransform()`")

    .def("copy", [](const m3d::EulerTransform &self) { return m3d::EulerTransform(self); },
        "Call copy constructor :cpp:func:`motion3d::EulerTransform::EulerTransform()`.")

    .def("setIdentity", &m3d::EulerTransform::setIdentity,
        ":cpp:func:`motion3d::EulerTransform::setIdentity()`")

    .def("toList", &m3d::EulerTransform::toVector,
        ":cpp:func:`motion3d::EulerTransform::toVector()`")

    .def("toBinary", &m3d::EulerTransform::toBinary,
        ":cpp:func:`motion3d::TransformInterface::toBinary()`")

    .def("toArray", &m3d::EulerTransform::toEigenVector,
        ":cpp:func:`motion3d::EulerTransform::toEigenVector()`")

    .def("isUnsafe", &m3d::EulerTransform::isUnsafe,
        ":cpp:func:`motion3d::TransformInterface::isUnsafe()`")

    .def("isValid", py::overload_cast<double>(&m3d::EulerTransform::isValid, py::const_),
        py::arg("eps") = m3d::kDefaultEps,
        ":cpp:func:`motion3d::EulerTransform::isValid()`")

    .def("changeAxes_", &m3d::EulerTransform::changeAxes_,
        py::arg("axes"),
        ":cpp:func:`motion3d::EulerTransform::changeAxes_()`")

    .def("changeAxes", &m3d::EulerTransform::changeAxes,
        py::arg("axes"),
        ":cpp:func:`motion3d::EulerTransform::changeAxes()`")

    .def("inverse_", &m3d::EulerTransform::inverse_,
        ":cpp:func:`motion3d::EulerTransform::inverse_()`")

    .def("inverse", &m3d::EulerTransform::inverse,
        ":cpp:func:`motion3d::EulerTransform::inverse()`")

    .def("normalized_", &m3d::EulerTransform::normalized_,
        ":cpp:func:`motion3d::EulerTransform::normalized_()`")

    .def("normalized", &m3d::EulerTransform::normalized,
        ":cpp:func:`motion3d::EulerTransform::normalized()`")

    .def("scaleTranslation_", &m3d::EulerTransform::scaleTranslation_,
        py::arg("factor"),
        ":cpp:func:`motion3d::EulerTransform::scaleTranslation_()`")

    .def("scaleTranslation", &m3d::EulerTransform::scaleTranslation,
        py::arg("factor"),
        ":cpp:func:`motion3d::EulerTransform::scaleTranslation()`")

    .def("applyPre_", &m3d::EulerTransform::applyPre_<m3d::DualQuaternionTransform>,
        py::arg("other"),
        ":cpp:func:`motion3d::EulerTransform::applyPre_()`")

    .def("applyPre", &m3d::EulerTransform::applyPre<m3d::DualQuaternionTransform>,
        py::arg("other"),
        ":cpp:func:`motion3d::EulerTransform::applyPre()`")

    .def("applyPost_", &m3d::EulerTransform::applyPost_<m3d::DualQuaternionTransform>,
        py::arg("other"),
        ":cpp:func:`motion3d::EulerTransform::applyPost_()`")

    .def("applyPost", &m3d::EulerTransform::applyPost<m3d::DualQuaternionTransform>,
        py::arg("other"),
        ":cpp:func:`motion3d::EulerTransform::applyPost()`")

    .def("applyPre_", &m3d::EulerTransform::applyPre_<m3d::TransformInterface::Ptr>,
        py::arg("other"),
        ":cpp:func:`motion3d::EulerTransform::applyPre_()`")

    .def("applyPre", &m3d::EulerTransform::applyPre<m3d::TransformInterface::Ptr>,
        py::arg("other"),
        ":cpp:func:`motion3d::EulerTransform::applyPre()`")

    .def("applyPost_", &m3d::EulerTransform::applyPost_<m3d::TransformInterface::Ptr>,
        py::arg("other"),
        ":cpp:func:`motion3d::EulerTransform::applyPost_()`")

    .def("applyPost", &m3d::EulerTransform::applyPost<m3d::TransformInterface::Ptr>,
        py::arg("other"),
        ":cpp:func:`motion3d::EulerTransform::applyPost()`")

    .def("transformPoint", 
        [](const m3d::EulerTransform &self, const Eigen::Ref<const Eigen::Vector3d> &point) {
          return self.transformPoint(point);
        },
        py::arg("point"),
        ":cpp:func:`motion3d::EulerTransform::transformPoint()`")

    .def("transformCloud", 
        [](const m3d::EulerTransform &self, const Eigen::Ref<const Eigen::Matrix<double, 3, Eigen::Dynamic>> &cloud) {
          return self.transformCloud(cloud);
        },
        py::arg("cloud"),
        ":cpp:func:`motion3d::EulerTransform::transformCloud()`")

    .def("rotationNorm", &m3d::EulerTransform::rotationNorm,
        ":cpp:func:`motion3d::EulerTransform::rotationNorm()`")

    .def("translationNorm", &m3d::EulerTransform::translationNorm,
        ":cpp:func:`motion3d::EulerTransform::translationNorm()`")

    .def("getTranslation", &m3d::EulerTransform::getTranslation,
        ":cpp:func:`motion3d::EulerTransform::getTranslation()`")

    .def("getAi", &m3d::EulerTransform::getAi,
        ":cpp:func:`motion3d::EulerTransform::getAi()`")

    .def("getAj", &m3d::EulerTransform::getAj,
        ":cpp:func:`motion3d::EulerTransform::getAj()`")

    .def("getAk", &m3d::EulerTransform::getAk,
        ":cpp:func:`motion3d::EulerTransform::getAk()`")

    .def("getAngles", &m3d::EulerTransform::getAngles,
        ":cpp:func:`motion3d::EulerTransform::getAngles()`")

    .def("getAxes", &m3d::EulerTransform::getAxes,
        ":cpp:func:`motion3d::EulerTransform::getAxes()`")

    .def("setTranslation",
        [](m3d::EulerTransform &self, const Eigen::Ref<const Eigen::Vector3d> &translation, const bool unsafe) -> m3d::EulerTransform& {
          return self.setTranslation(translation, unsafe);
        },
        py::arg("translation"), py::arg("unsafe") = false,
        ":cpp:func:`motion3d::EulerTransform::setTranslation()`")

    .def("setAi", &m3d::EulerTransform::setAi,
        py::arg("ai"), py::arg("unsafe") = false,
        ":cpp:func:`motion3d::EulerTransform::setAi()`")

    .def("setAj", &m3d::EulerTransform::setAj,
        py::arg("aj"), py::arg("unsafe") = false,
        ":cpp:func:`motion3d::EulerTransform::setAj()`")

    .def("setAk", &m3d::EulerTransform::setAk,
        py::arg("ak"), py::arg("unsafe") = false,
        ":cpp:func:`motion3d::EulerTransform::setAk()`")

    .def("setAngles",
        [](m3d::EulerTransform &self, const Eigen::Ref<const Eigen::Vector3d> &angles, const bool unsafe) -> m3d::EulerTransform& {
          return self.setAngles(angles, unsafe);
        },
        py::arg("angles"), py::arg("unsafe") = false,
        ":cpp:func:`motion3d::EulerTransform::setAngles()`")

    .def("setAxes", &m3d::EulerTransform::setAxes,
        py::arg("axes"), py::arg("unsafe") = false,
        ":cpp:func:`motion3d::EulerTransform::setAxes()`")

    .def("__repr__", &m3d::EulerTransform::desc,
        ":cpp:func:`motion3d::EulerTransform::desc()`")

    .def(py::self * m3d::AxisAngleTransform())
    .def(py::self * m3d::DualQuaternionTransform())
    .def(py::self * m3d::EulerTransform())
    .def(py::self * m3d::MatrixTransform())
    .def(py::self * m3d::QuaternionTransform())
    .def(py::self *= m3d::AxisAngleTransform())
    .def(py::self *= m3d::DualQuaternionTransform())
    .def(py::self *= m3d::EulerTransform())
    .def(py::self *= m3d::MatrixTransform())
    .def(py::self *= m3d::QuaternionTransform())
    .def(py::self / m3d::AxisAngleTransform())
    .def(py::self / m3d::DualQuaternionTransform())
    .def(py::self / m3d::EulerTransform())
    .def(py::self / m3d::MatrixTransform())
    .def(py::self / m3d::QuaternionTransform())
    .def(py::self /= m3d::AxisAngleTransform())
    .def(py::self /= m3d::DualQuaternionTransform())
    .def(py::self /= m3d::EulerTransform())
    .def(py::self /= m3d::MatrixTransform())
    .def(py::self /= m3d::QuaternionTransform());


  /************************************************
   *  MatrixTransform
   ***********************************************/
  py::class_<m3d::MatrixTransform, m3d::TransformInterface, m3d::MatrixTransform::Ptr>(m, "MatrixTransform",
      ":cpp:class:`motion3d::MatrixTransform`")

    .def(py::init<>(),
        ":cpp:func:`motion3d::MatrixTransform::MatrixTransform()`")

    .def(py::init<const Eigen::Ref<const Eigen::Matrix<double, 4, 4>>&, bool>(),
        py::arg("matrix"), py::arg("unsafe") = false,
        ":cpp:func:`motion3d::MatrixTransform::MatrixTransform()`")

    .def(py::init<const Eigen::Ref<const Eigen::Matrix<double, 3, 4>>&, bool>(),
        py::arg("matrix"), py::arg("unsafe") = false,
        ":cpp:func:`motion3d::MatrixTransform::MatrixTransform()`")

    .def(py::init<const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>&, const Eigen::Ref<const Eigen::Matrix<double, 3, 3>>&, bool>(),
        py::arg("translation"), py::arg("rotation_matrix"), py::arg("unsafe") = false,
        ":cpp:func:`motion3d::MatrixTransform::MatrixTransform()`")

    .def(py::init<const Eigen::Ref<const Eigen::Matrix<double, 12, 1>>&, bool>(),
        py::arg("data"), py::arg("unsafe") = false,
        ":cpp:func:`motion3d::MatrixTransform::MatrixTransform()`")

    .def("copy", [](const m3d::MatrixTransform &self) { return m3d::MatrixTransform(self); },
        "Call copy constructor :cpp:func:`motion3d::MatrixTransform::MatrixTransform()`.")

    .def("setIdentity", &m3d::MatrixTransform::setIdentity,
        ":cpp:func:`motion3d::MatrixTransform::setIdentity()`")

    .def("toList", &m3d::MatrixTransform::toVector,
        ":cpp:func:`motion3d::MatrixTransform::toVector()`")

    .def("toBinary", &m3d::MatrixTransform::toBinary,
        ":cpp:func:`motion3d::TransformInterface::toBinary()`")
        
    .def("toArray", &m3d::MatrixTransform::toEigenVector,
        ":cpp:func:`motion3d::MatrixTransform::toEigenVector()`")

    .def("isUnsafe", &m3d::MatrixTransform::isUnsafe,
        ":cpp:func:`motion3d::TransformInterface::isUnsafe()`")

    .def("isValid", py::overload_cast<double>(&m3d::MatrixTransform::isValid, py::const_),
        py::arg("eps") = m3d::kDefaultEps,
        ":cpp:func:`motion3d::MatrixTransform::isValid()`")

    .def("inverse_", &m3d::MatrixTransform::inverse_,
        ":cpp:func:`motion3d::MatrixTransform::inverse_()`")
        
    .def("inverse", &m3d::MatrixTransform::inverse,
        ":cpp:func:`motion3d::MatrixTransform::inverse()`")

    .def("normalized_", &m3d::MatrixTransform::normalized_,
        ":cpp:func:`motion3d::MatrixTransform::normalized_()`")

    .def("normalized", &m3d::MatrixTransform::normalized,
        ":cpp:func:`motion3d::MatrixTransform::normalized()`")

    .def("scaleTranslation_", &m3d::MatrixTransform::scaleTranslation_,
        py::arg("factor"),
        ":cpp:func:`motion3d::MatrixTransform::scaleTranslation_()`")

    .def("scaleTranslation", &m3d::MatrixTransform::scaleTranslation,
        py::arg("factor"),
        ":cpp:func:`motion3d::MatrixTransform::scaleTranslation()`")

    .def("applyPre_", &m3d::MatrixTransform::applyPre_<m3d::MatrixTransform>,
        py::arg("other"),
        ":cpp:func:`motion3d::MatrixTransform::applyPre_()`")

    .def("applyPre", &m3d::MatrixTransform::applyPre<m3d::MatrixTransform>,
        py::arg("other"),
        ":cpp:func:`motion3d::MatrixTransform::applyPre()`")

    .def("applyPost_", &m3d::MatrixTransform::applyPost_<m3d::MatrixTransform>,
        py::arg("other"),
        ":cpp:func:`motion3d::MatrixTransform::applyPost_()`")

    .def("applyPost", &m3d::MatrixTransform::applyPost<m3d::MatrixTransform>,
        py::arg("other"),
        ":cpp:func:`motion3d::MatrixTransform::applyPost()`")

    .def("applyPre_", &m3d::MatrixTransform::applyPre_<m3d::TransformInterface::Ptr>,
        py::arg("other"),
        ":cpp:func:`motion3d::MatrixTransform::applyPre_()`")

    .def("applyPre", &m3d::MatrixTransform::applyPre<m3d::TransformInterface::Ptr>,
        py::arg("other"),
        ":cpp:func:`motion3d::MatrixTransform::applyPre()`")

    .def("applyPost_", &m3d::MatrixTransform::applyPost_<m3d::TransformInterface::Ptr>,
        py::arg("other"),
        ":cpp:func:`motion3d::MatrixTransform::applyPost_()`")

    .def("applyPost", &m3d::MatrixTransform::applyPost<m3d::TransformInterface::Ptr>,
        py::arg("other"),
        ":cpp:func:`motion3d::MatrixTransform::applyPost()`")

    .def("transformPoint", 
        [](const m3d::MatrixTransform &self, const Eigen::Ref<const Eigen::Vector3d> &point) {
          return self.transformPoint(point);
        },
        py::arg("point"),
        ":cpp:func:`motion3d::MatrixTransform::transformPoint()`")

    .def("transformCloud", 
        [](const m3d::MatrixTransform &self, const Eigen::Ref<const Eigen::Matrix<double, 3, Eigen::Dynamic>> &cloud) {
          return self.transformCloud(cloud);
        },
        py::arg("cloud"),
        ":cpp:func:`motion3d::MatrixTransform::transformCloud()`")

    .def("rotationNorm", &m3d::MatrixTransform::rotationNorm,
        ":cpp:func:`motion3d::MatrixTransform::rotationNorm()`")

    .def("translationNorm", &m3d::MatrixTransform::translationNorm,
        ":cpp:func:`motion3d::MatrixTransform::translationNorm()`")

    .def("getMatrix", &m3d::MatrixTransform::getMatrix,
        ":cpp:func:`motion3d::MatrixTransform::getMatrix()`")

    .def("getTranslation", &m3d::MatrixTransform::getTranslation,
        ":cpp:func:`motion3d::MatrixTransform::getTranslation()`")

    .def("getRotationMatrix", &m3d::MatrixTransform::getRotationMatrix,
        ":cpp:func:`motion3d::MatrixTransform::getRotationMatrix()`")

    .def("setMatrix",
        [](m3d::MatrixTransform &self, const Eigen::Ref<const Eigen::Matrix<double, 3, 4>> &matrix, const bool unsafe) -> m3d::MatrixTransform& {
          return self.setMatrix(matrix, unsafe);
        },
        py::arg("matrix"), py::arg("unsafe") = false,
        ":cpp:func:`motion3d::MatrixTransform::setMatrix()`")

    .def("setMatrix",
        [](m3d::MatrixTransform &self, const Eigen::Ref<const Eigen::Matrix4d> &matrix, const bool unsafe) -> m3d::MatrixTransform& {
          return self.setMatrix(matrix, unsafe);
        },
        py::arg("matrix"), py::arg("unsafe") = false,
        ":cpp:func:`motion3d::MatrixTransform::setMatrix()`")

    .def("setTranslation",
        [](m3d::MatrixTransform &self, const Eigen::Ref<const Eigen::Vector3d> &translation, const bool unsafe) -> m3d::MatrixTransform& {
          return self.setTranslation(translation, unsafe);
        },
        py::arg("translation"), py::arg("unsafe") = false,
        ":cpp:func:`motion3d::MatrixTransform::setTranslation()`")

    .def("setRotationMatrix",
        [](m3d::MatrixTransform &self, const Eigen::Ref<const Eigen::Matrix3d> &rotation_matrix, const bool unsafe) -> m3d::MatrixTransform& {
          return self.setRotationMatrix(rotation_matrix, unsafe);
        },
        py::arg("rotation_matrix"), py::arg("unsafe") = false,
        ":cpp:func:`motion3d::MatrixTransform::setRotationMatrix()`")

    .def("__repr__", &m3d::MatrixTransform::desc,
        ":cpp:func:`motion3d::MatrixTransform::desc()`")

    .def(py::self * m3d::AxisAngleTransform())
    .def(py::self * m3d::DualQuaternionTransform())
    .def(py::self * m3d::EulerTransform())
    .def(py::self * m3d::MatrixTransform())
    .def(py::self * m3d::QuaternionTransform())
    .def(py::self *= m3d::AxisAngleTransform())
    .def(py::self *= m3d::DualQuaternionTransform())
    .def(py::self *= m3d::EulerTransform())
    .def(py::self *= m3d::MatrixTransform())
    .def(py::self *= m3d::QuaternionTransform())
    .def(py::self / m3d::AxisAngleTransform())
    .def(py::self / m3d::DualQuaternionTransform())
    .def(py::self / m3d::EulerTransform())
    .def(py::self / m3d::MatrixTransform())
    .def(py::self / m3d::QuaternionTransform())
    .def(py::self /= m3d::AxisAngleTransform())
    .def(py::self /= m3d::DualQuaternionTransform())
    .def(py::self /= m3d::EulerTransform())
    .def(py::self /= m3d::MatrixTransform())
    .def(py::self /= m3d::QuaternionTransform());


  /************************************************
   *  QuaternionTransform
   ***********************************************/
  py::class_<m3d::QuaternionTransform, m3d::TransformInterface, m3d::QuaternionTransform::Ptr>(m, "QuaternionTransform",
      ":cpp:class:`motion3d::QuaternionTransform`")

    .def(py::init<>(),
        ":cpp:func:`motion3d::QuaternionTransform::QuaternionTransform()`")

    .def(py::init<const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>&, const m3d::Quaterniond&, bool>(),
        py::arg("translation"), py::arg("quaternion"), py::arg("unsafe") = false,
        ":cpp:func:`motion3d::QuaternionTransform::QuaternionTransform()`")

    .def(py::init<const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>&, const Eigen::Ref<const Eigen::Matrix<double, 4, 1>>&, bool>(),
        py::arg("translation"), py::arg("quaternion"), py::arg("unsafe") = false,
        ":cpp:func:`motion3d::QuaternionTransform::QuaternionTransform()`")

    .def(py::init<const Eigen::Ref<const Eigen::Matrix<double, 7, 1>>&, bool>(),
        py::arg("data"), py::arg("unsafe") = false,
        ":cpp:func:`motion3d::QuaternionTransform::QuaternionTransform()`")

    .def("copy", [](const m3d::QuaternionTransform &self) { return m3d::QuaternionTransform(self); },
        "Call copy constructor :cpp:func:`motion3d::QuaternionTransform::QuaternionTransform()`.")

    .def("setIdentity", &m3d::QuaternionTransform::setIdentity,
        ":cpp:func:`motion3d::QuaternionTransform::setIdentity()`")

    .def("toList", &m3d::QuaternionTransform::toVector,
        ":cpp:func:`motion3d::QuaternionTransform::toVector()`")

    .def("toBinary", &m3d::QuaternionTransform::toBinary,
        ":cpp:func:`motion3d::TransformInterface::toBinary()`")

    .def("toArray", &m3d::QuaternionTransform::toEigenVector,
        ":cpp:func:`motion3d::QuaternionTransform::toEigenVector()`")

    .def("isUnsafe", &m3d::QuaternionTransform::isUnsafe,
        ":cpp:func:`motion3d::TransformInterface::isUnsafe()`")

    .def("isValid", py::overload_cast<double>(&m3d::QuaternionTransform::isValid, py::const_),
        py::arg("eps") = m3d::kDefaultEps,
        ":cpp:func:`motion3d::QuaternionTransform::isValid()`")

    .def("inverse_", &m3d::QuaternionTransform::inverse_,
        ":cpp:func:`motion3d::QuaternionTransform::inverse_()`")

    .def("inverse", &m3d::QuaternionTransform::inverse,
        ":cpp:func:`motion3d::QuaternionTransform::inverse()`")

    .def("normalized_", &m3d::QuaternionTransform::normalized_,
        ":cpp:func:`motion3d::QuaternionTransform::normalized_()`")

    .def("normalized", &m3d::QuaternionTransform::normalized,
        ":cpp:func:`motion3d::QuaternionTransform::normalized()`")

    .def("scaleTranslation_", &m3d::QuaternionTransform::scaleTranslation_,
        py::arg("factor"),
        ":cpp:func:`motion3d::QuaternionTransform::scaleTranslation_()`")

    .def("scaleTranslation", &m3d::QuaternionTransform::scaleTranslation,
        py::arg("factor"),
        ":cpp:func:`motion3d::QuaternionTransform::scaleTranslation()`")

    .def("applyPre_", &m3d::QuaternionTransform::applyPre_<m3d::DualQuaternionTransform>,
        py::arg("other"),
        ":cpp:func:`motion3d::QuaternionTransform::applyPre_()`")

    .def("applyPre", &m3d::QuaternionTransform::applyPre<m3d::DualQuaternionTransform>,
        py::arg("other"),
        ":cpp:func:`motion3d::QuaternionTransform::applyPre()`")

    .def("applyPost_", &m3d::QuaternionTransform::applyPost_<m3d::DualQuaternionTransform>,
        py::arg("other"),
        ":cpp:func:`motion3d::QuaternionTransform::applyPost_()`")

    .def("applyPost", &m3d::QuaternionTransform::applyPost<m3d::DualQuaternionTransform>,
        py::arg("other"),
        ":cpp:func:`motion3d::QuaternionTransform::applyPost()`")

    .def("applyPre_", &m3d::QuaternionTransform::applyPre_<m3d::TransformInterface::Ptr>,
        py::arg("other"),
        ":cpp:func:`motion3d::QuaternionTransform::applyPre_()`")

    .def("applyPre", &m3d::QuaternionTransform::applyPre<m3d::TransformInterface::Ptr>,
        py::arg("other"),
        ":cpp:func:`motion3d::QuaternionTransform::applyPre()`")

    .def("applyPost_", &m3d::QuaternionTransform::applyPost_<m3d::TransformInterface::Ptr>,
        py::arg("other"),
        ":cpp:func:`motion3d::QuaternionTransform::applyPost_()`")

    .def("applyPost", &m3d::QuaternionTransform::applyPost<m3d::TransformInterface::Ptr>,
        py::arg("other"),
        ":cpp:func:`motion3d::QuaternionTransform::applyPost()`")

    .def("transformPoint", 
        [](const m3d::QuaternionTransform &self, const Eigen::Ref<const Eigen::Vector3d> &point) {
          return self.transformPoint(point);
        },
        py::arg("point"),
        ":cpp:func:`motion3d::QuaternionTransform::transformPoint()`")

    .def("transformCloud", 
        [](const m3d::QuaternionTransform &self, const Eigen::Ref<const Eigen::Matrix<double, 3, Eigen::Dynamic>> &cloud) {
          return self.transformCloud(cloud);
        },
        py::arg("cloud"),
        ":cpp:func:`motion3d::QuaternionTransform::transformCloud()`")

    .def("rotationNorm", &m3d::QuaternionTransform::rotationNorm,
        ":cpp:func:`motion3d::QuaternionTransform::rotationNorm()`")

    .def("translationNorm", &m3d::QuaternionTransform::translationNorm,
        ":cpp:func:`motion3d::QuaternionTransform::translationNorm()`")

    .def("getTranslation", &m3d::QuaternionTransform::getTranslation,
        ":cpp:func:`motion3d::QuaternionTransform::getTranslation()`")

    .def("getQuaternion", &m3d::QuaternionTransform::getQuaternion,
        ":cpp:func:`motion3d::QuaternionTransform::getQuaternion()`")

    .def("setTranslation",
        [](m3d::QuaternionTransform &self, const Eigen::Ref<const Eigen::Vector3d> &translation, const bool unsafe) -> m3d::QuaternionTransform& {
          return self.setTranslation(translation, unsafe);
        },
        py::arg("translation"), py::arg("unsafe") = false,
        ":cpp:func:`motion3d::QuaternionTransform::setTranslation()`")

    .def("setQuaternion", &m3d::QuaternionTransform::setQuaternion,
        py::arg("quaternion"), py::arg("unsafe") = false,
        ":cpp:func:`motion3d::QuaternionTransform::setQuaternion()`")

    .def("__repr__", &m3d::QuaternionTransform::desc,
        ":cpp:func:`motion3d::QuaternionTransform::desc()`")
        
    .def(py::self * m3d::AxisAngleTransform())
    .def(py::self * m3d::DualQuaternionTransform())
    .def(py::self * m3d::EulerTransform())
    .def(py::self * m3d::MatrixTransform())
    .def(py::self * m3d::QuaternionTransform())
    .def(py::self *= m3d::AxisAngleTransform())
    .def(py::self *= m3d::DualQuaternionTransform())
    .def(py::self *= m3d::EulerTransform())
    .def(py::self *= m3d::MatrixTransform())
    .def(py::self *= m3d::QuaternionTransform())
    .def(py::self / m3d::AxisAngleTransform())
    .def(py::self / m3d::DualQuaternionTransform())
    .def(py::self / m3d::EulerTransform())
    .def(py::self / m3d::MatrixTransform())
    .def(py::self / m3d::QuaternionTransform())
    .def(py::self /= m3d::AxisAngleTransform())
    .def(py::self /= m3d::DualQuaternionTransform())
    .def(py::self /= m3d::EulerTransform())
    .def(py::self /= m3d::MatrixTransform())
    .def(py::self /= m3d::QuaternionTransform());
}
