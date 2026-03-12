// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from uav_visual_landing:msg/TargetObservation.idl
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
#include "uav_visual_landing/msg/detail/target_observation__struct.h"
#include "uav_visual_landing/msg/detail/target_observation__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool uav_visual_landing__msg__target_observation__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[61];
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
    assert(strncmp("uav_visual_landing.msg._target_observation.TargetObservation", full_classname_dest, 60) == 0);
  }
  uav_visual_landing__msg__TargetObservation * ros_message = _ros_message;
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
  {  // detected
    PyObject * field = PyObject_GetAttrString(_pymsg, "detected");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->detected = (Py_True == field);
    Py_DECREF(field);
  }
  {  // pose_valid
    PyObject * field = PyObject_GetAttrString(_pymsg, "pose_valid");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->pose_valid = (Py_True == field);
    Py_DECREF(field);
  }
  {  // confidence
    PyObject * field = PyObject_GetAttrString(_pymsg, "confidence");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->confidence = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // pixel_err_u
    PyObject * field = PyObject_GetAttrString(_pymsg, "pixel_err_u");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->pixel_err_u = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // pixel_err_v
    PyObject * field = PyObject_GetAttrString(_pymsg, "pixel_err_v");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->pixel_err_v = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // err_u_norm
    PyObject * field = PyObject_GetAttrString(_pymsg, "err_u_norm");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->err_u_norm = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // err_v_norm
    PyObject * field = PyObject_GetAttrString(_pymsg, "err_v_norm");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->err_v_norm = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // yaw_err_rad
    PyObject * field = PyObject_GetAttrString(_pymsg, "yaw_err_rad");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->yaw_err_rad = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // marker_span_px
    PyObject * field = PyObject_GetAttrString(_pymsg, "marker_span_px");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->marker_span_px = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // reproj_err_px
    PyObject * field = PyObject_GetAttrString(_pymsg, "reproj_err_px");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->reproj_err_px = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * uav_visual_landing__msg__target_observation__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of TargetObservation */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("uav_visual_landing.msg._target_observation");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "TargetObservation");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  uav_visual_landing__msg__TargetObservation * ros_message = (uav_visual_landing__msg__TargetObservation *)raw_ros_message;
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
  {  // detected
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->detected ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "detected", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pose_valid
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->pose_valid ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pose_valid", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // confidence
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->confidence);
    {
      int rc = PyObject_SetAttrString(_pymessage, "confidence", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pixel_err_u
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->pixel_err_u);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pixel_err_u", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pixel_err_v
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->pixel_err_v);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pixel_err_v", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // err_u_norm
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->err_u_norm);
    {
      int rc = PyObject_SetAttrString(_pymessage, "err_u_norm", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // err_v_norm
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->err_v_norm);
    {
      int rc = PyObject_SetAttrString(_pymessage, "err_v_norm", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // yaw_err_rad
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->yaw_err_rad);
    {
      int rc = PyObject_SetAttrString(_pymessage, "yaw_err_rad", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // marker_span_px
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->marker_span_px);
    {
      int rc = PyObject_SetAttrString(_pymessage, "marker_span_px", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // reproj_err_px
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->reproj_err_px);
    {
      int rc = PyObject_SetAttrString(_pymessage, "reproj_err_px", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
