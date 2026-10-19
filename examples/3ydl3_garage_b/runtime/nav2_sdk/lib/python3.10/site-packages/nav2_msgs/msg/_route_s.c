// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from nav2_msgs:msg/Route.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "nav2_msgs/msg/detail/route__struct.h"
#include "nav2_msgs/msg/detail/route__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

// Nested array functions includes
#include "nav2_msgs/msg/detail/route_edge__functions.h"
#include "nav2_msgs/msg/detail/route_node__functions.h"
// end nested array functions include
ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);
bool nav2_msgs__msg__route_node__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * nav2_msgs__msg__route_node__convert_to_py(void * raw_ros_message);
bool nav2_msgs__msg__route_edge__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * nav2_msgs__msg__route_edge__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool nav2_msgs__msg__route__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[27];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("nav2_msgs.msg._route.Route", full_classname_dest, 26) == 0);
  }
  nav2_msgs__msg__Route * ros_message = _ros_message;
  {  // header
    PyObject * field = PyObject_GetAttrString(_pymsg, "header");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__header__convert_from_py(field, &ros_message->header)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // route_cost
    PyObject * field = PyObject_GetAttrString(_pymsg, "route_cost");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->route_cost = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // nodes
    PyObject * field = PyObject_GetAttrString(_pymsg, "nodes");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'nodes'");
    if (!seq_field) {
      Py_DECREF(field);
      return false;
    }
    Py_ssize_t size = PySequence_Size(field);
    if (-1 == size) {
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    if (!nav2_msgs__msg__RouteNode__Sequence__init(&(ros_message->nodes), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create nav2_msgs__msg__RouteNode__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    nav2_msgs__msg__RouteNode * dest = ros_message->nodes.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!nav2_msgs__msg__route_node__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }
  {  // edges
    PyObject * field = PyObject_GetAttrString(_pymsg, "edges");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'edges'");
    if (!seq_field) {
      Py_DECREF(field);
      return false;
    }
    Py_ssize_t size = PySequence_Size(field);
    if (-1 == size) {
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    if (!nav2_msgs__msg__RouteEdge__Sequence__init(&(ros_message->edges), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create nav2_msgs__msg__RouteEdge__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    nav2_msgs__msg__RouteEdge * dest = ros_message->edges.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!nav2_msgs__msg__route_edge__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * nav2_msgs__msg__route__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Route */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("nav2_msgs.msg._route");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Route");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  nav2_msgs__msg__Route * ros_message = (nav2_msgs__msg__Route *)raw_ros_message;
  {  // header
    PyObject * field = NULL;
    field = std_msgs__msg__header__convert_to_py(&ros_message->header);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "header", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // route_cost
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->route_cost);
    {
      int rc = PyObject_SetAttrString(_pymessage, "route_cost", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // nodes
    PyObject * field = NULL;
    size_t size = ros_message->nodes.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    nav2_msgs__msg__RouteNode * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->nodes.data[i]);
      PyObject * pyitem = nav2_msgs__msg__route_node__convert_to_py(item);
      if (!pyitem) {
        Py_DECREF(field);
        return NULL;
      }
      int rc = PyList_SetItem(field, i, pyitem);
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "nodes", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // edges
    PyObject * field = NULL;
    size_t size = ros_message->edges.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    nav2_msgs__msg__RouteEdge * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->edges.data[i]);
      PyObject * pyitem = nav2_msgs__msg__route_edge__convert_to_py(item);
      if (!pyitem) {
        Py_DECREF(field);
        return NULL;
      }
      int rc = PyList_SetItem(field, i, pyitem);
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "edges", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
