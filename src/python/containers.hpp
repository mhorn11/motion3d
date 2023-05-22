#pragma once

#include <pybind11/pybind11.h>
#include <pybind11/chrono.h>
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/operators.h>
#include <pybind11/stl.h>

#include <motion3d/containers.hpp>

namespace m3d = motion3d;
namespace py = pybind11;

void init_containers(py::module &m)
{
  /************************************************
   *  Exceptions
   ***********************************************/
  py::register_exception<m3d::TransformContainerException>(m, "TransformContainerException");


  /************************************************
   *  TransformContainer
   ***********************************************/
  py::class_<m3d::TransformContainer, m3d::TransformContainer::Ptr>(m, "TransformContainer",
      ":cpp:class:`motion3d::TransformContainer`")

    // Constructors
    .def(py::init<bool, bool>(),
        py::arg("has_stamps"), py::arg("has_poses"),
        ":cpp:func:`motion3d::TransformContainer::TransformContainer()`")

    .def(py::init<const std::vector<m3d::TransformInterface::Ptr>&, bool>(),
        py::arg("transforms"), py::arg("has_poses"),
        ":cpp:func:`motion3d::TransformContainer::TransformContainer()`")

    .def(py::init<const std::vector<m3d::TransformInterface::ConstPtr>&, bool>(),
        py::arg("transforms"), py::arg("has_poses"),
        ":cpp:func:`motion3d::TransformContainer::TransformContainer()`")
        
    .def(py::init<const std::map<m3d::Time, m3d::TransformInterface::Ptr>&, bool>(),
        py::arg("transforms"), py::arg("has_poses"),
        ":cpp:func:`motion3d::TransformContainer::TransformContainer()`")

    .def(py::init<const std::map<m3d::Time, m3d::TransformInterface::ConstPtr>&, bool>(),
        py::arg("transforms"), py::arg("has_poses"),
        ":cpp:func:`motion3d::TransformContainer::TransformContainer()`")
        
    .def(py::init<const std::vector<m3d::Time>&, const std::vector<m3d::TransformInterface::Ptr>&, bool, bool>(),
        py::arg("stamps"), py::arg("transforms"), py::arg("has_poses"), py::arg("sorted_data") = false,
        ":cpp:func:`motion3d::TransformContainer::TransformContainer()`")

    .def(py::init<const std::vector<m3d::Time>&, const std::vector<m3d::TransformInterface::ConstPtr>&, bool, bool>(),
        py::arg("stamps"), py::arg("transforms"), py::arg("has_poses"), py::arg("sorted_data") = false,
        ":cpp:func:`motion3d::TransformContainer::TransformContainer()`")

    .def(py::init<const m3d::TransformContainer&>(),
        py::arg("other"),
        ":cpp:func:`motion3d::TransformContainer::TransformContainer()`")
    .def("copy", [](const m3d::TransformContainer &self) { return m3d::TransformContainer(self); })

    // Access
    .def("size", &m3d::TransformContainer::size,
        ":cpp:func:`motion3d::TransformContainer::size()`")

    .def("empty", &m3d::TransformContainer::empty,
        ":cpp:func:`motion3d::TransformContainer::empty()`")

    .def("hasStamp", &m3d::TransformContainer::hasStamp,
        ":cpp:func:`motion3d::TransformContainer::hasStamp()`")

    .def("at",
        py::overload_cast<const std::size_t&>(&m3d::TransformContainer::at),
        py::arg("index"),
        ":cpp:func:`motion3d::TransformContainer::at()`")

    .def("at_stamp",
        py::overload_cast<const m3d::Time&>(&m3d::TransformContainer::at_stamp),
        py::arg("stamp"),
        ":cpp:func:`motion3d::TransformContainer::at_stamp()`")

    .def("stamp_at", &m3d::TransformContainer::stamp_at,
        py::arg("index"),
        ":cpp:func:`motion3d::TransformContainer::stamp_at()`")

    .def("item_at",
        py::overload_cast<const std::size_t&>(&m3d::TransformContainer::item_at),
        py::arg("index"),
        ":cpp:func:`motion3d::TransformContainer::item_at()`")

    .def("clear", &m3d::TransformContainer::clear,
        ":cpp:func:`motion3d::TransformContainer::clear()`")

    .def("erase", py::overload_cast<const std::size_t&>(&m3d::TransformContainer::erase),
        py::arg("index"),
        ":cpp:func:`motion3d::TransformContainer::erase()`")

    .def("erase", py::overload_cast<const m3d::TransformContainer::StampType&>(&m3d::TransformContainer::erase),
        py::arg("stamp"),
        ":cpp:func:`motion3d::TransformContainer::erase()`")

    .def("append",
        py::overload_cast<const m3d::TransformInterface::ConstPtr&>(&m3d::TransformContainer::append),
        py::arg("transform"),
        ":cpp:func:`motion3d::TransformContainer::append()`")

    .def("append",
        py::overload_cast<const m3d::TransformContainer::StampType&, const m3d::TransformInterface::ConstPtr&>(
            &m3d::TransformContainer::append),
        py::arg("stamp"), py::arg("transform"),
        ":cpp:func:`motion3d::TransformContainer::append()`")

    .def("insert",
        py::overload_cast<const std::size_t&, const m3d::TransformInterface::ConstPtr&>(&m3d::TransformContainer::insert),
        py::arg("index"), py::arg("transform"),
        ":cpp:func:`motion3d::TransformContainer::insert()`")

    .def("insert",
        py::overload_cast<const m3d::TransformContainer::StampType&, const m3d::TransformInterface::ConstPtr&, bool>(
            &m3d::TransformContainer::insert),
        py::arg("stamp"), py::arg("transform"), py::arg("overwrite") = false,
        ":cpp:func:`motion3d::TransformContainer::insert()`")

    .def("extend",
        [](m3d::TransformContainer &self, const m3d::TransformContainer &other, const bool overwrite) {
          self.extend(other, overwrite);
        },
        py::arg("other"), py::arg("overwrite") = false,
        ":cpp:func:`motion3d::TransformContainer::extend()`")

    .def("extend",
         [](m3d::TransformContainer &self, const m3d::TransformContainer::ConstPtr &other, const bool overwrite) {
           self.extend(other, overwrite);
         },
         py::arg("other"), py::arg("overwrite") = false,
         ":cpp:func:`motion3d::TransformContainer::extend()`")

    // Stamps, Poses and Motions
    .def("hasStamps", &m3d::TransformContainer::hasStamps,
        ":cpp:func:`motion3d::TransformContainer::hasStamps()`")

    .def("hasPoses", &m3d::TransformContainer::hasPoses,
        ":cpp:func:`motion3d::TransformContainer::hasPoses()`")

    .def("hasMotions", &m3d::TransformContainer::hasMotions,
        ":cpp:func:`motion3d::TransformContainer::hasMotions()`")

    .def("removeStamps_", &m3d::TransformContainer::removeStamps_,
        ":cpp:func:`motion3d::TransformContainer::removeStamps_()`")

    .def("removeStamps", &m3d::TransformContainer::removeStamps,
        ":cpp:func:`motion3d::TransformContainer::removeStamps()`")

    .def("addStamps_", &m3d::TransformContainer::addStamps_,
        py::arg("stamps"),
        ":cpp:func:`motion3d::TransformContainer::addStamps_()`")

    .def("addStamps", &m3d::TransformContainer::addStamps,
        py::arg("stamps"),
        ":cpp:func:`motion3d::TransformContainer::addStamps()`")

    // Find
    .def("find_eq", 
        [](m3d::TransformContainer &self, const m3d::TransformContainer::StampType &stamp) -> py::object {
          auto it = self.find_eq(stamp);
          if (it == self.end_items()) {
              return py::cast(std::make_pair(py::none(), py::none()));
          }
          return py::cast(std::make_pair(it->first, it->second));
        },
        py::arg("stamp"),
        ":cpp:func:`motion3d::TransformContainer::find_eq()`, :return: ``(stamp, transform)`` or  ``(None, None)`` if no element was found.")

    .def("find_ge", 
        [](m3d::TransformContainer &self, const m3d::TransformContainer::StampType &stamp) -> py::object {
          auto it = self.find_ge(stamp);
          if (it == self.end_items()) {
              return py::cast(std::make_pair(py::none(), py::none()));
          }
          return py::cast(std::make_pair(it->first, it->second));
        },
        py::arg("stamp"),
        ":cpp:func:`motion3d::TransformContainer::find_ge()`, :return: ``(stamp, transform)`` or ``(None, None)`` if no element was found.")

    .def("find_gt", 
        [](m3d::TransformContainer &self, const m3d::TransformContainer::StampType &stamp) -> py::object {
          auto it = self.find_gt(stamp);
          if (it == self.end_items()) {
              return py::cast(std::make_pair(py::none(), py::none()));
          }
          return py::cast(std::make_pair(it->first, it->second));
        },
        py::arg("stamp"),
        ":cpp:func:`motion3d::TransformContainer::find_gt()`, :return: ``(stamp, transform)`` or ``(None, None)`` if no element was found.")

    .def("find_le", 
        [](m3d::TransformContainer &self, const m3d::TransformContainer::StampType &stamp) -> py::object {
          auto it = self.find_le(stamp);
          if (it == self.end_items()) {
              return py::cast(std::make_pair(py::none(), py::none()));
          }
          return py::cast(std::make_pair(it->first, it->second));
        },
        py::arg("stamp"),
        ":cpp:func:`motion3d::TransformContainer::find_le()`, :return: ``(stamp, transform)`` or  ``(None, None)`` if no element was found.")

    .def("find_lt", 
        [](m3d::TransformContainer &self, const m3d::TransformContainer::StampType &stamp) -> py::object {
          auto it = self.find_lt(stamp);
          if (it == self.end_items()) {
              return py::cast(std::make_pair(py::none(), py::none()));
          }
          return py::cast(std::make_pair(it->first, it->second));
        },
        py::arg("stamp"),
        ":cpp:func:`motion3d::TransformContainer::find_lt()`, :return: ``(stamp, transform)`` or  ``(None, None)`` if no element was found.")

    .def("find_closest", 
        [](m3d::TransformContainer &self, const m3d::TransformContainer::StampType &stamp) -> py::object {
          auto it = self.find_closest(stamp);
          if (it == self.end_items()) {
              return py::cast(std::make_pair(py::none(), py::none()));
          }
          return py::cast(std::make_pair(it->first, it->second));
        },
        py::arg("stamp"),
        ":cpp:func:`motion3d::TransformContainer::find_closest()`, :return: ``(stamp, transform)`` or  ``(None, None)`` if no element was found.")

    // Conversion
    .def("asType_", py::overload_cast<const m3d::TransformType&>(&m3d::TransformContainer::asType_),
        py::arg("type"),
        ":cpp:func:`motion3d::TransformContainer::asType_()`")

    .def("asType", py::overload_cast<const m3d::TransformType&>(&m3d::TransformContainer::asType, py::const_),
        py::arg("type"),
        ":cpp:func:`motion3d::TransformContainer::asType()`")
    
    .def("asPoses_", py::overload_cast<>(&m3d::TransformContainer::asPoses_),
        ":cpp:func:`motion3d::TransformContainer::asPoses_()`")

    .def("asPoses", py::overload_cast<>(&m3d::TransformContainer::asPoses, py::const_),
        ":cpp:func:`motion3d::TransformContainer::asPoses()`")

    .def("asPoses_", py::overload_cast<const m3d::TransformInterface::ConstPtr&>(&m3d::TransformContainer::asPoses_),
        ":cpp:func:`motion3d::TransformContainer::asPoses_()`")

    .def("asPoses", py::overload_cast<const m3d::TransformInterface::ConstPtr&>(&m3d::TransformContainer::asPoses, py::const_),
        ":cpp:func:`motion3d::TransformContainer::asPoses()`")

    .def("asMotions_", &m3d::TransformContainer::asMotions_,
        ":cpp:func:`motion3d::TransformContainer::asMotions_()`")

    .def("asMotions", &m3d::TransformContainer::asMotions,
        ":cpp:func:`motion3d::TransformContainer::asMotions()`")

    // Transformations
    .def("inverse_", &m3d::TransformContainer::inverse_,
        ":cpp:func:`motion3d::TransformContainer::inverse_()`")

    .def("inverse", &m3d::TransformContainer::inverse,
        ":cpp:func:`motion3d::TransformContainer::inverse()`")

    .def("normalized_", &m3d::TransformContainer::normalized_,
        ":cpp:func:`motion3d::TransformContainer::normalized_()`")

    .def("normalized", &m3d::TransformContainer::normalized,
        ":cpp:func:`motion3d::TransformContainer::normalized()`")

    .def("scaleTranslation_", &m3d::TransformContainer::scaleTranslation_,
        py::arg("factor"),
        ":cpp:func:`motion3d::TransformContainer::scaleTranslation_()`")

    .def("scaleTranslation", &m3d::TransformContainer::scaleTranslation,
        py::arg("factor"),
        ":cpp:func:`motion3d::TransformContainer::scaleTranslation()`")

    .def("applyPre_", &m3d::TransformContainer::applyPre_<const m3d::TransformInterface::ConstPtr&>,
        py::arg("transform"),
        ":cpp:func:`motion3d::TransformContainer::applyPre_()`")

    .def("applyPre", &m3d::TransformContainer::applyPre<const m3d::TransformInterface::ConstPtr&>,
        py::arg("transform"),
        ":cpp:func:`motion3d::TransformContainer::applyPre()`")

    .def("applyPost_", &m3d::TransformContainer::applyPost_<const m3d::TransformInterface::ConstPtr&>,
        py::arg("transform"),
        ":cpp:func:`motion3d::TransformContainer::applyPost_()`")

    .def("applyPost", &m3d::TransformContainer::applyPost<const m3d::TransformInterface::ConstPtr&>,
        py::arg("transform"),
        ":cpp:func:`motion3d::TransformContainer::applyPost()`")

    .def("apply_", &m3d::TransformContainer::apply_<const m3d::TransformInterface::ConstPtr&, const m3d::TransformInterface::ConstPtr&>, 
        py::arg("transform_pre"), py::arg("transform_post"),
        ":cpp:func:`motion3d::TransformContainer::apply_()`")

    .def("apply", &m3d::TransformContainer::apply<const m3d::TransformInterface::ConstPtr&, const m3d::TransformInterface::ConstPtr&>, 
        py::arg("transform_pre"), py::arg("transform_post"),
        ":cpp:func:`motion3d::TransformContainer::apply()`")

    .def("applyFunc_", &m3d::TransformContainer::applyFunc_,
        py::arg("func"),
        ":cpp:func:`motion3d::TransformContainer::applyFunc_()`")

    .def("applyFunc", &m3d::TransformContainer::applyFunc,
        py::arg("func"),
        ":cpp:func:`motion3d::TransformContainer::applyFunc()`")

    .def("applyIndexFunc_", &m3d::TransformContainer::applyIndexFunc_,
        py::arg("func"),
        ":cpp:func:`motion3d::TransformContainer::applyIndexFunc_()`")

    .def("applyIndexFunc", &m3d::TransformContainer::applyIndexFunc,
        py::arg("func"),
        ":cpp:func:`motion3d::TransformContainer::applyIndexFunc()`")

    .def("applyStampFunc_", &m3d::TransformContainer::applyStampFunc_,
        py::arg("func"),
        ":cpp:func:`motion3d::TransformContainer::applyStampFunc_()`")

    .def("applyStampFunc", &m3d::TransformContainer::applyStampFunc,
        py::arg("func"),
        ":cpp:func:`motion3d::TransformContainer::applyStampFunc()`")

    .def("changeFrame_", &m3d::TransformContainer::changeFrame_<m3d::TransformInterface::ConstPtr>,
        py::arg("transform"),
        ":cpp:func:`motion3d::TransformContainer::changeFrame_()`")

    .def("changeFrame", &m3d::TransformContainer::changeFrame<m3d::TransformInterface::ConstPtr>,
        py::arg("transform"),
        ":cpp:func:`motion3d::TransformContainer::changeFrame()`")

    // Raw Data
    .def("toList", static_cast<std::vector<m3d::TransformInterface::Ptr> (m3d::TransformContainer::*)() const>(&m3d::TransformContainer::toVector),
        ":cpp:func:`motion3d::TransformContainer::toVector()`")

    .def("toList", static_cast<std::vector<m3d::TransformInterface::Ptr> (m3d::TransformContainer::*)(const m3d::TransformType&) const>(&m3d::TransformContainer::toVector), 
        py::arg("type"),
        ":cpp:func:`motion3d::TransformContainer::toVector()`")

    .def("toArray", py::overload_cast<const m3d::TransformType&>(&m3d::TransformContainer::toEigenVector, py::const_), 
        py::arg("type"),
        ":cpp:func:`motion3d::TransformContainer::toEigenVector()`")

    // Special Members
    .def("__len__", &m3d::TransformContainer::size,
        ":cpp:func:`motion3d::TransformContainer::size()`")

    .def("__contains__", &m3d::TransformContainer::hasStamp,
         ":cpp:func:`motion3d::TransformContainer::hasStamp()`")

    .def("__getitem__",
        [](const m3d::TransformContainer &self, std::size_t i) {
          if (i >= self.size())
            throw py::index_error();
          return self.at(i);
        })

    .def("__getitem__",
        [](const m3d::TransformContainer &self, const m3d::TransformContainer::StampType &stamp) {
          auto it = self.find_eq(stamp);
          if (it == self.end_items())
            throw py::index_error();
          return it->second;
        })

    .def("__getitem__",
        [](const m3d::TransformContainer &self, const py::slice &slice) -> m3d::TransformContainer {
          size_t start = 0, stop = 0, step = 0, slicelength = 0;
          if (!slice.compute(self.size(), &start, &stop, &step, &slicelength))
            throw py::error_already_set();
          m3d::TransformContainer new_list(self.hasStamps(), self.hasPoses());
          if (self.hasStamps()) {
            for (size_t i = 0; i < slicelength; ++i) {
              new_list.insert(self.stamp_at(start), self.at(start));
              start += step;
            }
          } else {
            for (size_t i = 0; i < slicelength; ++i) {
              new_list.append(self.at(start));
              start += step;
            }
          }
          return new_list;
        })

    .def("__setitem__",
         [](m3d::TransformContainer &self, std::size_t i, const m3d::TransformInterface::ConstPtr &transform) {
           if (i >= self.size())
             throw py::index_error();
           self.at(i) = transform->copy();
         })

    .def("__setitem__",
         [](m3d::TransformContainer &self,
            const m3d::TransformContainer::StampType &stamp,
            const m3d::TransformInterface::ConstPtr &transform) {
           self.insert(stamp, transform, true);
         })

    .def("__repr__", &m3d::TransformContainer::desc,
        ":cpp:func:`motion3d::TransformContainer::desc()`")

    // Iterators
    .def("__iter__", [](m3d::TransformContainer &self) { return py::make_iterator(self.begin(), self.end()); }, py::keep_alive<0, 1>(),
        ":return: an iterator from :cpp:func:`motion3d::TransformContainer::begin()` to :cpp:func:`motion3d::TransformContainer::end()`")

    .def("stamps", [](const m3d::TransformContainer &self) { return py::make_iterator(self.cbegin_stamps(), self.cend_stamps()); }, py::keep_alive<0, 1>(),
        ":return: an iterator from :cpp:func:`motion3d::TransformContainer::cbegin_stamps()` to :cpp:func:`motion3d::TransformContainer::cend_stamps()`")

    .def("transforms", [](m3d::TransformContainer &self) { return py::make_iterator(self.begin(), self.end()); }, py::keep_alive<0, 1>(),
        ":return: an iterator from :cpp:func:`motion3d::TransformContainer::begin()` to :cpp:func:`motion3d::TransformContainer::end()`")

    .def("items", 
        [](m3d::TransformContainer &self) {
          return py::make_iterator(self.begin_items(), self.end_items());
        }, py::keep_alive<0, 1>(),
        ":return: an iterator from :cpp:func:`motion3d::TransformContainer::begin_items()` to :cpp:func:`motion3d::TransformContainer::end_items()`")

    .def("items_ge", 
        [](m3d::TransformContainer &self, const m3d::TransformContainer::StampType &stamp) {
          return py::make_iterator(self.find_ge(stamp), self.end_items());
        }, py::keep_alive<0, 1>(),
        py::arg("stamp"),
        ":return: an iterator from :cpp:func:`motion3d::TransformContainer::find_ge()` to :cpp:func:`motion3d::TransformContainer::end_items()`")

    .def("items_gt", 
        [](m3d::TransformContainer &self, const m3d::TransformContainer::StampType &stamp) {
          return py::make_iterator(self.find_gt(stamp), self.end_items());
        }, py::keep_alive<0, 1>(),
        py::arg("stamp"),
        ":return: an iterator from :cpp:func:`motion3d::TransformContainer::find_gt()` to :cpp:func:`motion3d::TransformContainer::end_items()`")

    .def("items_le", 
        [](m3d::TransformContainer &self, const m3d::TransformContainer::StampType &stamp) {
          return py::make_iterator(self.find_le(stamp), self.end_items());
        }, py::keep_alive<0, 1>(),
        py::arg("stamp"),
        ":return: an iterator from :cpp:func:`motion3d::TransformContainer::find_le()` to :cpp:func:`motion3d::TransformContainer::end_items()`")

    .def("items_lt", 
        [](m3d::TransformContainer &self, const m3d::TransformContainer::StampType &stamp) {
          return py::make_iterator(self.find_lt(stamp), self.end_items());
        }, py::keep_alive<0, 1>(),
        py::arg("stamp"),
        ":return: an iterator from :cpp:func:`motion3d::TransformContainer::find_lt()` to :cpp:func:`motion3d::TransformContainer::end_items()`")
        
    .def("items_closest", 
        [](m3d::TransformContainer &self, const m3d::TransformContainer::StampType &stamp) {
          return py::make_iterator(self.find_closest(stamp), self.end_items());
        }, py::keep_alive<0, 1>(),
        py::arg("stamp"),
        ":return: an iterator from :cpp:func:`motion3d::TransformContainer::find_closest()` to :cpp:func:`motion3d::TransformContainer::end_items()`");
}
