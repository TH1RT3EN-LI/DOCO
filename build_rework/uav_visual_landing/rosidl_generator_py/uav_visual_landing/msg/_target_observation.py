# generated from rosidl_generator_py/resource/_idl.py.em
# with input from uav_visual_landing:msg/TargetObservation.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_TargetObservation(type):
    """Metaclass of message 'TargetObservation'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('uav_visual_landing')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'uav_visual_landing.msg.TargetObservation')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__target_observation
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__target_observation
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__target_observation
            cls._TYPE_SUPPORT = module.type_support_msg__msg__target_observation
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__target_observation

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class TargetObservation(metaclass=Metaclass_TargetObservation):
    """Message class 'TargetObservation'."""

    __slots__ = [
        '_header',
        '_detected',
        '_pose_valid',
        '_confidence',
        '_pixel_err_u',
        '_pixel_err_v',
        '_err_u_norm',
        '_err_v_norm',
        '_yaw_err_rad',
        '_marker_span_px',
        '_reproj_err_px',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'detected': 'boolean',
        'pose_valid': 'boolean',
        'confidence': 'float',
        'pixel_err_u': 'float',
        'pixel_err_v': 'float',
        'err_u_norm': 'float',
        'err_v_norm': 'float',
        'yaw_err_rad': 'float',
        'marker_span_px': 'float',
        'reproj_err_px': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.detected = kwargs.get('detected', bool())
        self.pose_valid = kwargs.get('pose_valid', bool())
        self.confidence = kwargs.get('confidence', float())
        self.pixel_err_u = kwargs.get('pixel_err_u', float())
        self.pixel_err_v = kwargs.get('pixel_err_v', float())
        self.err_u_norm = kwargs.get('err_u_norm', float())
        self.err_v_norm = kwargs.get('err_v_norm', float())
        self.yaw_err_rad = kwargs.get('yaw_err_rad', float())
        self.marker_span_px = kwargs.get('marker_span_px', float())
        self.reproj_err_px = kwargs.get('reproj_err_px', float())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.header != other.header:
            return False
        if self.detected != other.detected:
            return False
        if self.pose_valid != other.pose_valid:
            return False
        if self.confidence != other.confidence:
            return False
        if self.pixel_err_u != other.pixel_err_u:
            return False
        if self.pixel_err_v != other.pixel_err_v:
            return False
        if self.err_u_norm != other.err_u_norm:
            return False
        if self.err_v_norm != other.err_v_norm:
            return False
        if self.yaw_err_rad != other.yaw_err_rad:
            return False
        if self.marker_span_px != other.marker_span_px:
            return False
        if self.reproj_err_px != other.reproj_err_px:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def detected(self):
        """Message field 'detected'."""
        return self._detected

    @detected.setter
    def detected(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'detected' field must be of type 'bool'"
        self._detected = value

    @builtins.property
    def pose_valid(self):
        """Message field 'pose_valid'."""
        return self._pose_valid

    @pose_valid.setter
    def pose_valid(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'pose_valid' field must be of type 'bool'"
        self._pose_valid = value

    @builtins.property
    def confidence(self):
        """Message field 'confidence'."""
        return self._confidence

    @confidence.setter
    def confidence(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'confidence' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'confidence' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._confidence = value

    @builtins.property
    def pixel_err_u(self):
        """Message field 'pixel_err_u'."""
        return self._pixel_err_u

    @pixel_err_u.setter
    def pixel_err_u(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pixel_err_u' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'pixel_err_u' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._pixel_err_u = value

    @builtins.property
    def pixel_err_v(self):
        """Message field 'pixel_err_v'."""
        return self._pixel_err_v

    @pixel_err_v.setter
    def pixel_err_v(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pixel_err_v' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'pixel_err_v' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._pixel_err_v = value

    @builtins.property
    def err_u_norm(self):
        """Message field 'err_u_norm'."""
        return self._err_u_norm

    @err_u_norm.setter
    def err_u_norm(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'err_u_norm' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'err_u_norm' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._err_u_norm = value

    @builtins.property
    def err_v_norm(self):
        """Message field 'err_v_norm'."""
        return self._err_v_norm

    @err_v_norm.setter
    def err_v_norm(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'err_v_norm' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'err_v_norm' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._err_v_norm = value

    @builtins.property
    def yaw_err_rad(self):
        """Message field 'yaw_err_rad'."""
        return self._yaw_err_rad

    @yaw_err_rad.setter
    def yaw_err_rad(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'yaw_err_rad' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'yaw_err_rad' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._yaw_err_rad = value

    @builtins.property
    def marker_span_px(self):
        """Message field 'marker_span_px'."""
        return self._marker_span_px

    @marker_span_px.setter
    def marker_span_px(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'marker_span_px' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'marker_span_px' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._marker_span_px = value

    @builtins.property
    def reproj_err_px(self):
        """Message field 'reproj_err_px'."""
        return self._reproj_err_px

    @reproj_err_px.setter
    def reproj_err_px(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'reproj_err_px' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'reproj_err_px' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._reproj_err_px = value
