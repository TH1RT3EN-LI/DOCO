# generated from rosidl_generator_py/resource/_idl.py.em
# with input from uav_visual_landing:msg/LandingControllerState.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_LandingControllerState(type):
    """Metaclass of message 'LandingControllerState'."""

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
                'uav_visual_landing.msg.LandingControllerState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__landing_controller_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__landing_controller_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__landing_controller_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__landing_controller_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__landing_controller_state

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


class LandingControllerState(metaclass=Metaclass_LandingControllerState):
    """Message class 'LandingControllerState'."""

    __slots__ = [
        '_header',
        '_active',
        '_phase',
        '_target_detected',
        '_observation_age_s',
        '_target_confidence',
        '_height_source',
        '_current_height_m',
        '_range_valid',
        '_range_height_m',
        '_cmd_vx',
        '_cmd_vy',
        '_cmd_vz',
        '_cmd_yaw_rate',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'active': 'boolean',
        'phase': 'string',
        'target_detected': 'boolean',
        'observation_age_s': 'float',
        'target_confidence': 'float',
        'height_source': 'string',
        'current_height_m': 'float',
        'range_valid': 'boolean',
        'range_height_m': 'float',
        'cmd_vx': 'float',
        'cmd_vy': 'float',
        'cmd_vz': 'float',
        'cmd_yaw_rate': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
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
        self.active = kwargs.get('active', bool())
        self.phase = kwargs.get('phase', str())
        self.target_detected = kwargs.get('target_detected', bool())
        self.observation_age_s = kwargs.get('observation_age_s', float())
        self.target_confidence = kwargs.get('target_confidence', float())
        self.height_source = kwargs.get('height_source', str())
        self.current_height_m = kwargs.get('current_height_m', float())
        self.range_valid = kwargs.get('range_valid', bool())
        self.range_height_m = kwargs.get('range_height_m', float())
        self.cmd_vx = kwargs.get('cmd_vx', float())
        self.cmd_vy = kwargs.get('cmd_vy', float())
        self.cmd_vz = kwargs.get('cmd_vz', float())
        self.cmd_yaw_rate = kwargs.get('cmd_yaw_rate', float())

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
        if self.active != other.active:
            return False
        if self.phase != other.phase:
            return False
        if self.target_detected != other.target_detected:
            return False
        if self.observation_age_s != other.observation_age_s:
            return False
        if self.target_confidence != other.target_confidence:
            return False
        if self.height_source != other.height_source:
            return False
        if self.current_height_m != other.current_height_m:
            return False
        if self.range_valid != other.range_valid:
            return False
        if self.range_height_m != other.range_height_m:
            return False
        if self.cmd_vx != other.cmd_vx:
            return False
        if self.cmd_vy != other.cmd_vy:
            return False
        if self.cmd_vz != other.cmd_vz:
            return False
        if self.cmd_yaw_rate != other.cmd_yaw_rate:
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
    def active(self):
        """Message field 'active'."""
        return self._active

    @active.setter
    def active(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'active' field must be of type 'bool'"
        self._active = value

    @builtins.property
    def phase(self):
        """Message field 'phase'."""
        return self._phase

    @phase.setter
    def phase(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'phase' field must be of type 'str'"
        self._phase = value

    @builtins.property
    def target_detected(self):
        """Message field 'target_detected'."""
        return self._target_detected

    @target_detected.setter
    def target_detected(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'target_detected' field must be of type 'bool'"
        self._target_detected = value

    @builtins.property
    def observation_age_s(self):
        """Message field 'observation_age_s'."""
        return self._observation_age_s

    @observation_age_s.setter
    def observation_age_s(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'observation_age_s' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'observation_age_s' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._observation_age_s = value

    @builtins.property
    def target_confidence(self):
        """Message field 'target_confidence'."""
        return self._target_confidence

    @target_confidence.setter
    def target_confidence(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'target_confidence' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'target_confidence' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._target_confidence = value

    @builtins.property
    def height_source(self):
        """Message field 'height_source'."""
        return self._height_source

    @height_source.setter
    def height_source(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'height_source' field must be of type 'str'"
        self._height_source = value

    @builtins.property
    def current_height_m(self):
        """Message field 'current_height_m'."""
        return self._current_height_m

    @current_height_m.setter
    def current_height_m(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'current_height_m' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'current_height_m' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._current_height_m = value

    @builtins.property
    def range_valid(self):
        """Message field 'range_valid'."""
        return self._range_valid

    @range_valid.setter
    def range_valid(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'range_valid' field must be of type 'bool'"
        self._range_valid = value

    @builtins.property
    def range_height_m(self):
        """Message field 'range_height_m'."""
        return self._range_height_m

    @range_height_m.setter
    def range_height_m(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'range_height_m' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'range_height_m' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._range_height_m = value

    @builtins.property
    def cmd_vx(self):
        """Message field 'cmd_vx'."""
        return self._cmd_vx

    @cmd_vx.setter
    def cmd_vx(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'cmd_vx' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'cmd_vx' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._cmd_vx = value

    @builtins.property
    def cmd_vy(self):
        """Message field 'cmd_vy'."""
        return self._cmd_vy

    @cmd_vy.setter
    def cmd_vy(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'cmd_vy' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'cmd_vy' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._cmd_vy = value

    @builtins.property
    def cmd_vz(self):
        """Message field 'cmd_vz'."""
        return self._cmd_vz

    @cmd_vz.setter
    def cmd_vz(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'cmd_vz' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'cmd_vz' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._cmd_vz = value

    @builtins.property
    def cmd_yaw_rate(self):
        """Message field 'cmd_yaw_rate'."""
        return self._cmd_yaw_rate

    @cmd_yaw_rate.setter
    def cmd_yaw_rate(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'cmd_yaw_rate' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'cmd_yaw_rate' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._cmd_yaw_rate = value
