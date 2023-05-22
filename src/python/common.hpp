#pragma once

#include <pybind11/pybind11.h>
#include <pybind11/chrono.h>
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/operators.h>
#include <pybind11/stl.h>

#include <motion3d/common.hpp>

namespace m3d = motion3d;
namespace py = pybind11;

void init_common(py::module &m)
{
  /************************************************
   *  Exceptions
   ***********************************************/
  py::register_exception<m3d::MathException>(m, "MathException");


  /************************************************
   *  General Functions
   ***********************************************/
  m.def("normalizeAngle_", &m3d::normalizeAngle_<double>,
      py::arg("x"),
      ":cpp:func:`motion3d::normalizeAngle_`");

  m.def("normalizeAngle", &m3d::normalizeAngle<double>,
      py::arg("x"),
      ":cpp:func:`motion3d::normalizeAngle_`");

  m.def("getAxisNormalizationFactorInverse",
      [](const Eigen::Ref<const Eigen::VectorXd> &v, bool unit_norm) {
        return m3d::getAxisNormalizationFactorInverse(v, unit_norm);
      },
      py::arg("v"), py::arg("unit_norm"),
      ":cpp:func:`motion3d::getAxisNormalizationFactorInverse`");

  m.def("normalizeRotationMatrix_",
      [](Eigen::Ref<Eigen::Matrix3d> m) {
        m3d::normalizeRotationMatrix_(m);
      },
      py::arg("m"),
      ":cpp:func:`motion3d::normalizeRotationMatrix_`");

  m.def("normalizeRotationMatrix",
      [](const Eigen::Ref<const Eigen::Matrix3d> &m) {
        return m3d::normalizeRotationMatrix(m);
      },
      py::arg("m"),
      ":cpp:func:`motion3d::normalizeRotationMatrix`");

  m.def("decomposeRZS",
      [](const Eigen::Ref<const Eigen::Matrix3d> &m) {
        return m3d::decomposeRZS(m);
      },
      py::arg("m"),
      ":cpp:func:`motion3d::decomposeRZS`");


  /************************************************
   *  Time
   ***********************************************/
  py::class_<m3d::Time>(m, "Time",
      ":cpp:class:`motion3d::Time`")
  
    .def(py::init<>(),
        ":cpp:func:`motion3d::Time::Time()`")

    .def(py::init<double>(),
        py::arg("t_sec"),
        ":cpp:func:`motion3d::Time::Time()`")

    .def(py::init<std::uint64_t>(),
        py::arg("t_nsec"),
        ":cpp:func:`motion3d::Time::Time()`")

    .def(py::init<std::chrono::system_clock::time_point>(),
        py::arg("t"),
        ":cpp:func:`motion3d::Time::Time()`")

    .def(py::init<std::uint32_t, std::uint32_t>(),
        py::arg("t_sec"), py::arg("t_nsec"),
        ":cpp:func:`motion3d::Time::Time()`")

    .def_static("FromSec", &m3d::Time::FromSec,
        py::arg("t_sec"),
        ":cpp:func:`motion3d::Time::FromSec()`")

    .def_static("FromNSec", &m3d::Time::FromNSec,
        py::arg("t_nsec"),
        ":cpp:func:`motion3d::Time::FromNSec()`")

    .def_static("FromSecNSec", &m3d::Time::FromSecNSec,
        py::arg("t_sec"), py::arg("t_nsec"),
        ":cpp:func:`motion3d::Time::FromSecNSec()`")

    .def("toSec", &m3d::Time::toSec,
        ":cpp:func:`motion3d::Time::toSec()`")

    .def("toNSec", &m3d::Time::toNSec,
        ":cpp:func:`motion3d::Time::toNSec()`")

    .def("toSecNSec", &m3d::Time::toSecNSec,
        ":cpp:func:`motion3d::Time::toSecNSec()`")

    .def("__repr__", &m3d::Time::desc,
        ":cpp:func:`motion3d::Time::desc()`")

    .def("__hash__", &m3d::Time::toNSec)

    .def(py::self == py::self)
    .def(py::self != py::self)
    .def(py::self < py::self)
    .def(py::self <= py::self)
    .def(py::self > py::self)
    .def(py::self >= py::self);


  m.def("timeDiffAbs", &m3d::timeDiffAbs,
      py::arg("ta"), py::arg("tb"),
      ":cpp:func:`motion3d::timeDiffAbs()`");


  /************************************************
   *  Quaternion
   ***********************************************/
  py::class_<m3d::Quaterniond>(m, "Quaternion",
      ":cpp:class:`motion3d::Quaternion` with **_Scalar** = ``double``")

    .def(py::init<>(),
        ":cpp:func:`motion3d::Quaternion::Quaternion()`")

    .def(py::init<const m3d::Quaterniond&>(),
        py::arg("other"),
        ":cpp:func:`motion3d::Quaternion::Quaternion()`")

    .def(py::init<const double&, const double&, const double&, const double&>(),
        py::arg("w"), py::arg("x"), py::arg("y"), py::arg("z"),
        ":cpp:func:`motion3d::Quaternion::Quaternion()`")

    .def(py::init<const double&, const m3d::Quaterniond::Vector3&>(),
        py::arg("angle"), py::arg("axis"),
        ":cpp:func:`motion3d::Quaternion::Quaternion()`")

    .def_static("Zero", &m3d::Quaterniond::Zero,
        ":cpp:func:`motion3d::Quaternion::Zero()`")

    .def_static("Identity", &m3d::Quaterniond::Identity,
        ":cpp:func:`motion3d::Quaternion::Identity()`")

    .def_static("UnitRandom", &m3d::Quaterniond::UnitRandom,
        ":cpp:func:`motion3d::Quaternion::UnitRandom()`")

    .def_static("FromList", 
        [](const std::vector<double> &data) {
          return m3d::Quaterniond::FromVector(data);
        },
        py::arg("data"),
        ":cpp:func:`motion3d::Quaternion::FromVector()`")

    .def_static("FromArray", 
        [](const Eigen::Ref<const Eigen::Matrix<double,4,1>> &data) {
          return m3d::Quaterniond::FromVector(data);
        },
        py::arg("data"),
        ":cpp:func:`motion3d::Quaternion::FromVector()`")

    .def_static("FromRotationMatrix",
        [](const Eigen::Ref<const m3d::Quaterniond::Matrix3> &matrix) {
          return m3d::Quaterniond::FromRotationMatrix(matrix);
        },
        py::arg("matrix"),
        ":cpp:func:`motion3d::Quaternion::FromRotationMatrix()`")

    .def("copy", [](const m3d::Quaterniond &self) { return m3d::Quaterniond(self); },
        "Call copy constructor :cpp:func:`motion3d::Quaternion::Quaternion()`.")

    .def("toList", &m3d::Quaterniond::toVector,
        ":cpp:func:`motion3d::Quaternion::toVector()`")

    .def("toArray", &m3d::Quaterniond::toEigenVector,
        ":cpp:func:`motion3d::Quaternion::toEigenVector()`")

    .def_property("w", py::overload_cast<>(&m3d::Quaterniond::w, py::const_),
        [](m3d::Quaterniond &self, const double &value) { self.w() = value; },
        ":cpp:func:`motion3d::Quaternion::w`")

    .def_property("x", py::overload_cast<>(&m3d::Quaterniond::x, py::const_),
        [](m3d::Quaterniond &self, const double &value) { self.x() = value; },
        ":cpp:func:`motion3d::Quaternion::x`")

    .def_property("y", py::overload_cast<>(&m3d::Quaterniond::y, py::const_),
        [](m3d::Quaterniond &self, const double &value) { self.y() = value; },
        ":cpp:func:`motion3d::Quaternion::y`")

    .def_property("z", py::overload_cast<>(&m3d::Quaterniond::z, py::const_),
        [](m3d::Quaterniond &self, const double &value) { self.z() = value; },
        ":cpp:func:`motion3d::Quaternion::z`")

    .def("vec", py::overload_cast<>(&m3d::Quaterniond::vec, py::const_),
        ":cpp:func:`motion3d::Quaternion::vec()`")

    .def("toPositiveMatrix", &m3d::Quaterniond::toPositiveMatrix,
        ":cpp:func:`motion3d::Quaternion::toPositiveMatrix()`")

    .def("toNegativeMatrix", &m3d::Quaterniond::toNegativeMatrix,
        ":cpp:func:`motion3d::Quaternion::toNegativeMatrix()`")

    .def("toAxisAngle", [](const m3d::Quaterniond &self) {
           m3d::Quaterniond::AngleAxisType axis_angle = self.toAxisAngle();
           return std::make_pair(axis_angle.angle(), axis_angle.axis());
        },
        ":cpp:func:`motion3d::Quaternion::toAxisAngle()`")

    .def("toRotationMatrix", &m3d::Quaterniond::toRotationMatrix,
        ":cpp:func:`motion3d::Quaternion::toRotationMatrix()`")

    .def("squaredNorm", &m3d::Quaterniond::squaredNorm,
        ":cpp:func:`motion3d::Quaternion::squaredNorm()`")

    .def("norm", &m3d::Quaterniond::norm,
        ":cpp:func:`motion3d::Quaternion::norm()`")

    .def("rotationNorm", &m3d::Quaterniond::rotationNorm,
        ":cpp:func:`motion3d::Quaternion::rotationNorm()`")

    .def("conjugate_", &m3d::Quaterniond::conjugate_,
        ":cpp:func:`motion3d::Quaternion::conjugate_()`")

    .def("conjugate", &m3d::Quaterniond::conjugate,
        ":cpp:func:`motion3d::Quaternion::conjugate()`")

    .def("inverse_", &m3d::Quaterniond::inverse_,
        ":cpp:func:`motion3d::Quaternion::inverse_()`")

    .def("inverse", &m3d::Quaterniond::inverse,
        ":cpp:func:`motion3d::Quaternion::inverse()`")

    .def("normalized_", &m3d::Quaterniond::normalized_,
        ":cpp:func:`motion3d::Quaternion::normalized_()`")

    .def("normalized", &m3d::Quaterniond::normalized,
        ":cpp:func:`motion3d::Quaternion::normalized()`")

    .def("isEqual", &m3d::Quaterniond::isEqual,
        py::arg("other"), py::arg("eps") = m3d::kDefaultEps,
        ":cpp:func:`motion3d::Quaternion::isEqual()`")

    .def("slerp", &m3d::Quaterniond::slerp,
        py::arg("t"), py::arg("other"),
        ":cpp:func:`motion3d::Quaternion::slerp()`")

    .def("angularDistance", &m3d::Quaterniond::angularDistance,
        py::arg("other"),
        ":cpp:func:`motion3d::Quaternion::angularDistance()`")

    .def("setZero", &m3d::Quaterniond::setZero,
        ":cpp:func:`motion3d::Quaternion::setZero()`")

    .def("setIdentity", &m3d::Quaterniond::setIdentity,
        ":cpp:func:`motion3d::Quaternion::setIdentity()`")

    .def("transformPoint", 
        [](const m3d::Quaterniond &self, const Eigen::Ref<const Eigen::Vector3d> &point) {
          return self.transformPoint(point);
        },
        py::arg("point"),
        ":cpp:func:`motion3d::Quaternion::transformPoint()`")

    .def("__repr__", &m3d::Quaterniond::desc,
        ":cpp:func:`motion3d::Quaternion::desc()`")

    .def(py::self * m3d::Quaterniond())
    .def(py::self * double())
    .def(double() * py::self)
    .def(py::self *= double())
    .def(py::self / double())
    .def(py::self /= double())
    .def(py::self + m3d::Quaterniond())
    .def(py::self += m3d::Quaterniond())
    .def(-py::self)
    .def(py::self - m3d::Quaterniond())
    .def(py::self -= m3d::Quaterniond());


  /************************************************
   *  DualQuaternion
   ***********************************************/
  py::class_<m3d::DualQuaterniond>(m, "DualQuaternion",
      ":cpp:class:`motion3d::DualQuaternion` with **_Scalar** = ``double``")

    .def(py::init<>(),
        ":cpp:func:`motion3d::DualQuaternion::DualQuaternion()`")

    .def(py::init<const m3d::DualQuaterniond&>(),
        py::arg("other"),
        ":cpp:func:`motion3d::DualQuaternion::DualQuaternion()`")

    .def(py::init<const m3d::Quaterniond&, const m3d::Quaterniond&>(),
        py::arg("real"), py::arg("dual"),
        ":cpp:func:`motion3d::DualQuaternion::DualQuaternion()`")

    .def(py::init<const m3d::DualQuaterniond::Vector3&, const m3d::Quaterniond&>(),
        py::arg("translation"), py::arg("rotation"),
        ":cpp:func:`motion3d::DualQuaternion::DualQuaternion()`")

    .def(py::init<const double&, const double&, const double&, const double&,
        const double&, const double&, const double&, const double&>(),
        py::arg("real_w"), py::arg("real_x"), py::arg("real_y"), py::arg("real_z"),
        py::arg("dual_w"), py::arg("dual_x"), py::arg("dual_y"), py::arg("dual_z"),
        ":cpp:func:`motion3d::DualQuaternion::DualQuaternion()`")

    .def_static("Zero", &m3d::DualQuaterniond::Zero,
        ":cpp:func:`motion3d::DualQuaternion::Zero()`")

    .def_static("Identity", &m3d::DualQuaterniond::Identity,
        ":cpp:func:`motion3d::DualQuaternion::Identity()`")

    .def_static("FromList", 
        [](const std::vector<double> &data) {
          return m3d::DualQuaterniond::FromVector(data);
        },
        py::arg("data"),
        ":cpp:func:`motion3d::DualQuaternion::FromVector()`")

    .def_static("FromList", 
        [](const std::vector<double> &real, const std::vector<double> &dual) {
          return m3d::DualQuaterniond::FromVector(real, dual);
        },
        py::arg("real"), py::arg("dual"),
        ":cpp:func:`motion3d::DualQuaternion::FromVector()`")

    .def_static("FromArray", 
        [](const Eigen::Ref<const Eigen::Matrix<double,8,1>> &data) {
          return m3d::DualQuaterniond::FromVector(data);
        },
        py::arg("data"),
        ":cpp:func:`motion3d::DualQuaternion::FromVector()`")

    .def_static("FromArray", 
        [](const Eigen::Ref<const Eigen::Matrix<double,4,1>> &real, const Eigen::Ref<const Eigen::Matrix<double,4,1>> &dual) {
          return m3d::DualQuaterniond::FromVector(real, dual);
        },
        py::arg("real"), py::arg("dual"),
        ":cpp:func:`motion3d::DualQuaternion::FromVector()`")

    .def_static("FromPoint", 
        [](const Eigen::Ref<const m3d::DualQuaterniond::Vector3> &point) {
          return m3d::DualQuaterniond::FromPoint(point);
        },
        py::arg("point"),
        ":cpp:func:`motion3d::DualQuaternion::FromPoint()`")

    .def("copy", [](const m3d::DualQuaterniond &self) { return m3d::DualQuaterniond(self); },
        "Call copy constructor :cpp:func:`motion3d::DualQuaternion::DualQuaternion()`.")

    .def("toList", &m3d::DualQuaterniond::toVector,
        ":cpp:func:`motion3d::DualQuaternion::toVector()`")

    .def("toArray", &m3d::DualQuaterniond::toEigenVector,
        ":cpp:func:`motion3d::DualQuaternion::toEigenVector()`")

    .def_property("real", py::overload_cast<>(&m3d::DualQuaterniond::real),
        [](m3d::DualQuaterniond &self, const m3d::Quaterniond &quat) { self.real() = quat; },
        ":cpp:func:`motion3d::DualQuaternion::real()`")

    .def_property("dual", py::overload_cast<>(&m3d::DualQuaterniond::dual),
        [](m3d::DualQuaterniond &self, const m3d::Quaterniond &quat) { self.dual() = quat; },
        ":cpp:func:`motion3d::DualQuaternion::dual()`")

    .def("squaredNorm", &m3d::DualQuaterniond::squaredNorm,
        ":cpp:func:`motion3d::DualQuaternion::squaredNorm()`")

    .def("norm", &m3d::DualQuaterniond::norm,
        ":cpp:func:`motion3d::DualQuaternion::norm()`")

    .def("translationNorm", &m3d::DualQuaterniond::translationNorm,
        ":cpp:func:`motion3d::DualQuaternion::translationNorm()`")

    .def("rotationNorm", &m3d::DualQuaterniond::rotationNorm,
        ":cpp:func:`motion3d::DualQuaternion::rotationNorm()`")

    .def("quatConjugate_", &m3d::DualQuaterniond::quatConjugate_,
        ":cpp:func:`motion3d::DualQuaternion::quatConjugate_()`")

    .def("quatConjugate", &m3d::DualQuaterniond::quatConjugate,
        ":cpp:func:`motion3d::DualQuaternion::quatConjugate()`")

    .def("dualConjugate_", &m3d::DualQuaterniond::dualConjugate_,
        ":cpp:func:`motion3d::DualQuaternion::dualConjugate_()`")

    .def("dualConjugate", &m3d::DualQuaterniond::dualConjugate,
        ":cpp:func:`motion3d::DualQuaternion::dualConjugate()`")

    .def("combConjugate_", &m3d::DualQuaterniond::combConjugate_,
        ":cpp:func:`motion3d::DualQuaternion::combConjugate_()`")

    .def("combConjugate", &m3d::DualQuaterniond::combConjugate,
        ":cpp:func:`motion3d::DualQuaternion::combConjugate()`")

    .def("inverse_", &m3d::DualQuaterniond::inverse_,
        ":cpp:func:`motion3d::DualQuaternion::inverse_()`")

    .def("inverse", &m3d::DualQuaterniond::inverse,
        ":cpp:func:`motion3d::DualQuaternion::inverse()`")

    .def("normalized_", &m3d::DualQuaterniond::normalized_,
        ":cpp:func:`motion3d::DualQuaternion::normalized_()`")

    .def("normalized", &m3d::DualQuaterniond::normalized,
        ":cpp:func:`motion3d::DualQuaternion::normalized()`")

    .def("isEqual", &m3d::DualQuaterniond::isEqual,
        py::arg("other"), py::arg("eps") = m3d::kDefaultEps,
        ":cpp:func:`motion3d::DualQuaternion::isEqual()`")

    .def("setZero", &m3d::DualQuaterniond::setZero,
        ":cpp:func:`motion3d::DualQuaternion::setZero()`")

    .def("setIdentity", &m3d::DualQuaterniond::setIdentity,
        ":cpp:func:`motion3d::DualQuaternion::setIdentity()`")

    .def("toPositiveMatrix", &m3d::DualQuaterniond::toPositiveMatrix,
        ":cpp:func:`motion3d::DualQuaternion::toPositiveMatrix()`")

    .def("toNegativeMatrix", &m3d::DualQuaterniond::toNegativeMatrix,
        ":cpp:func:`motion3d::DualQuaternion::toNegativeMatrix()`")

    .def("getTranslationQuaternion", &m3d::DualQuaterniond::getTranslationQuaternion,
        ":cpp:func:`motion3d::DualQuaternion::getTranslationQuaternion()`")

    .def("getTranslationVector", &m3d::DualQuaterniond::getTranslationVector,
        ":cpp:func:`motion3d::DualQuaternion::getTranslationVector()`")

    .def("transformPoint", 
        [](const m3d::DualQuaterniond &self, const Eigen::Ref<const Eigen::Vector3d> &point) {
          return self.transformPoint(point);
        },
        py::arg("point"),
        ":cpp:func:`motion3d::DualQuaternion::transformPoint()`")

    .def("__repr__", &m3d::DualQuaterniond::desc,
        ":cpp:func:`motion3d::DualQuaternion::desc()`")
        
    .def(py::self * m3d::DualQuaterniond())
    .def(py::self * double())
    .def(double() * py::self)
    .def(py::self *= double())
    .def(py::self / double())
    .def(py::self /= double())
    .def(py::self + m3d::DualQuaterniond())
    .def(py::self += m3d::DualQuaterniond())
    .def(-py::self)
    .def(py::self - m3d::DualQuaterniond())
    .def(py::self -= m3d::DualQuaterniond());
}
