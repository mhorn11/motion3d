#pragma once

#include <pybind11/pybind11.h>
#include <pybind11/chrono.h>
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/operators.h>
#include <pybind11/stl.h>

#include <motion3d/io.hpp>

namespace m3d = motion3d;
namespace py = pybind11;

void init_io(py::module &m)
{
  /************************************************
   *  MotionData
   ***********************************************/
  py::class_<m3d::MotionData>(m, "MotionData",
      ":cpp:class:`motion3d::MotionData`")

    .def(py::init<const std::string>(),
        py::arg("frame_id") = "",
        ":cpp:func:`motion3d::MotionData::MotionData()`")

    .def(py::init<const m3d::TransformType&, const m3d::TransformContainer::ConstPtr&, std::string>(),
        py::arg("type"), py::arg("transforms"), py::arg("frame_id") = "",
        ":cpp:func:`motion3d::MotionData::MotionData()`")

    .def(py::init<const m3d::TransformType&, const m3d::TransformContainer::ConstPtr&, const m3d::TransformInterface::ConstPtr&, std::string>(),
        py::arg("type"), py::arg("transforms"), py::arg("origin"), py::arg("frame_id") = "",
        ":cpp:func:`motion3d::MotionData::MotionData()`")

    .def(py::init<const m3d::MotionData&>(),
        py::arg("data"),
        ":cpp:func:`motion3d::MotionData::MotionData()`")

    .def("getFrameId", &m3d::MotionData::getFrameId,
        ":cpp:func:`motion3d::MotionData::getFrameId()`")

    .def("getTransformType",
        [](const m3d::MotionData &self) -> const m3d::TransformType* {
          // appearently, pybind11 cannot handle a shared_ptr to an enum class
          return self.getTransformType().get();
        },
        py::return_value_policy::reference_internal,
        ":cpp:func:`motion3d::MotionData::getTransformType()`")

    .def("getOrigin", &m3d::MotionData::getOrigin,
        ":cpp:func:`motion3d::MotionData::getOrigin()`")

    .def("getTransforms", &m3d::MotionData::getTransforms,
        ":cpp:func:`motion3d::MotionData::getTransforms()`")

    .def("setFrameId", &m3d::MotionData::setFrameId,
        py::arg("frame_id"),
        ":cpp:func:`motion3d::MotionData::setFrameId()`")

    .def("setTransformType", &m3d::MotionData::setTransformType,
        py::arg("type"),
        ":cpp:func:`motion3d::MotionData::setTransformType()`")

    .def("setOrigin", &m3d::MotionData::setOrigin,
        py::arg("origin"),
        ":cpp:func:`motion3d::MotionData::setOrigin()`")

    .def("setTransforms", &m3d::MotionData::setTransforms,
        py::arg("transforms"),
        ":cpp:func:`motion3d::MotionData::setTransforms()`")

    .def("__repr__", &m3d::MotionData::desc,
        ":cpp:func:`motion3d::MotionData::desc()`");


  /************************************************
   *  M3DIOStatus
   ***********************************************/
  py::enum_<m3d::M3DIOStatus>(m, "M3DIOStatus",
      ":cpp:enum:`motion3d::M3DIOStatus`")
    .value("kSuccess", m3d::M3DIOStatus::kSuccess)
    .value("kFileNotFound", m3d::M3DIOStatus::kFileNotFound)
    .value("kFileOpenError", m3d::M3DIOStatus::kFileOpenError)
    .value("kFileLockError", m3d::M3DIOStatus::kFileLockError)
    .value("kFilePermissionsError", m3d::M3DIOStatus::kFilePermissionsError)
    .value("kUnsupportedFileType", m3d::M3DIOStatus::kUnsupportedFileType)
    .value("kNoTransformType", m3d::M3DIOStatus::kNoTransformType)
    .value("kInvalidTransformType", m3d::M3DIOStatus::kInvalidTransformType)
    .value("kNoTransforms", m3d::M3DIOStatus::kNoTransforms)
    .value("kInvalidBinarySize", m3d::M3DIOStatus::kInvalidBinarySize)
    .value("kInvalidTransform", m3d::M3DIOStatus::kInvalidTransform)
    .value("kMoreTransforms", m3d::M3DIOStatus::kMoreTransforms)
    .value("kLessTransforms", m3d::M3DIOStatus::kLessTransforms)
    .value("kInvalidFrame", m3d::M3DIOStatus::kInvalidFrame)
    .value("kInvalidType", m3d::M3DIOStatus::kInvalidType)
    .value("kInvalidStamps", m3d::M3DIOStatus::kInvalidStamps)
    .value("kInvalidPoses", m3d::M3DIOStatus::kInvalidPoses)
    .value("kInvalidSize", m3d::M3DIOStatus::kInvalidSize);


  /************************************************
   *  M3DFileType
   ***********************************************/
  py::enum_<m3d::M3DFileType>(m, "M3DFileType",
      ":cpp:enum:`motion3d::M3DFileType`")
    .value("kASCII", m3d::M3DFileType::kASCII)
    .value("kBinary", m3d::M3DFileType::kBinary);


  /************************************************
   *  M3DReader
   ***********************************************/
  py::class_<m3d::M3DReader>(m, "M3DReader",
      ":cpp:class:`motion3d::M3DReader`")

    .def_static("read",
        [](const std::string &file_name, const bool unsafe, const int offset) { 
          m3d::M3DIOStatus result;
          std::optional<m3d::MotionData> motion = m3d::M3DReader::read(file_name, result, unsafe, offset);
          return std::make_tuple(motion, result);
        },
        py::arg("file_name"), py::arg("unsafe") = false, py::arg("offset") = 0,
        ":cpp:func:`motion3d::M3DReader::read()`")

    .def_static("read", py::overload_cast<const std::string&, m3d::MotionData&, const bool, const int>(&m3d::M3DReader::read),
        py::arg("file_name"), py::arg("motion"), py::arg("unsafe") = false, py::arg("offset") = 0,
        ":cpp:func:`motion3d::M3DReader::read()`");


  /************************************************
   *  M3DWriter
   ***********************************************/
  py::class_<m3d::M3DWriter>(m, "M3DWriter",
      ":cpp:class:`motion3d::M3DWriter`")

    .def_static("write", &m3d::M3DWriter::write,
        py::arg("file_name"), py::arg("motion"),
        py::arg("file_type"), py::arg("precision") = m3d::M3DWriter::kDefaultASCIIPrecision,
        ":cpp:func:`motion3d::M3DWriter::write()`")

    .def_static("writeASCII", &m3d::M3DWriter::writeASCII,
        py::arg("file_name"), py::arg("motion"), py::arg("precision") = m3d::M3DWriter::kDefaultASCIIPrecision,
        ":cpp:func:`motion3d::M3DWriter::writeASCII()`")
        
    .def_static("writeBinary", &m3d::M3DWriter::writeBinary,
        py::arg("file_name"), py::arg("motion"),
        ":cpp:func:`motion3d::M3DWriter::writeBinary()`");
}
