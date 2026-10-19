# generated from rosidl_generator_py/resource/_idl.py.em
# with input from nav2_msgs:srv/DynamicEdges.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'closed_edges'
# Member 'opened_edges'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_DynamicEdges_Request(type):
    """Metaclass of message 'DynamicEdges_Request'."""

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
            module = import_type_support('nav2_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'nav2_msgs.srv.DynamicEdges_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__dynamic_edges__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__dynamic_edges__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__dynamic_edges__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__dynamic_edges__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__dynamic_edges__request

            from nav2_msgs.msg import EdgeCost
            if EdgeCost.__class__._TYPE_SUPPORT is None:
                EdgeCost.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class DynamicEdges_Request(metaclass=Metaclass_DynamicEdges_Request):
    """Message class 'DynamicEdges_Request'."""

    __slots__ = [
        '_closed_edges',
        '_opened_edges',
        '_adjust_edges',
    ]

    _fields_and_field_types = {
        'closed_edges': 'sequence<uint16>',
        'opened_edges': 'sequence<uint16>',
        'adjust_edges': 'sequence<nav2_msgs/EdgeCost>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('uint16')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('uint16')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['nav2_msgs', 'msg'], 'EdgeCost')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.closed_edges = array.array('H', kwargs.get('closed_edges', []))
        self.opened_edges = array.array('H', kwargs.get('opened_edges', []))
        self.adjust_edges = kwargs.get('adjust_edges', [])

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
        if self.closed_edges != other.closed_edges:
            return False
        if self.opened_edges != other.opened_edges:
            return False
        if self.adjust_edges != other.adjust_edges:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def closed_edges(self):
        """Message field 'closed_edges'."""
        return self._closed_edges

    @closed_edges.setter
    def closed_edges(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'H', \
                "The 'closed_edges' array.array() must have the type code of 'H'"
            self._closed_edges = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 65536 for val in value)), \
                "The 'closed_edges' field must be a set or sequence and each value of type 'int' and each unsigned integer in [0, 65535]"
        self._closed_edges = array.array('H', value)

    @builtins.property
    def opened_edges(self):
        """Message field 'opened_edges'."""
        return self._opened_edges

    @opened_edges.setter
    def opened_edges(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'H', \
                "The 'opened_edges' array.array() must have the type code of 'H'"
            self._opened_edges = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 65536 for val in value)), \
                "The 'opened_edges' field must be a set or sequence and each value of type 'int' and each unsigned integer in [0, 65535]"
        self._opened_edges = array.array('H', value)

    @builtins.property
    def adjust_edges(self):
        """Message field 'adjust_edges'."""
        return self._adjust_edges

    @adjust_edges.setter
    def adjust_edges(self, value):
        if __debug__:
            from nav2_msgs.msg import EdgeCost
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, EdgeCost) for v in value) and
                 True), \
                "The 'adjust_edges' field must be a set or sequence and each value of type 'EdgeCost'"
        self._adjust_edges = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_DynamicEdges_Response(type):
    """Metaclass of message 'DynamicEdges_Response'."""

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
            module = import_type_support('nav2_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'nav2_msgs.srv.DynamicEdges_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__dynamic_edges__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__dynamic_edges__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__dynamic_edges__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__dynamic_edges__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__dynamic_edges__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class DynamicEdges_Response(metaclass=Metaclass_DynamicEdges_Response):
    """Message class 'DynamicEdges_Response'."""

    __slots__ = [
        '_success',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())

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
        if self.success != other.success:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def success(self):
        """Message field 'success'."""
        return self._success

    @success.setter
    def success(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'success' field must be of type 'bool'"
        self._success = value


class Metaclass_DynamicEdges(type):
    """Metaclass of service 'DynamicEdges'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('nav2_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'nav2_msgs.srv.DynamicEdges')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__dynamic_edges

            from nav2_msgs.srv import _dynamic_edges
            if _dynamic_edges.Metaclass_DynamicEdges_Request._TYPE_SUPPORT is None:
                _dynamic_edges.Metaclass_DynamicEdges_Request.__import_type_support__()
            if _dynamic_edges.Metaclass_DynamicEdges_Response._TYPE_SUPPORT is None:
                _dynamic_edges.Metaclass_DynamicEdges_Response.__import_type_support__()


class DynamicEdges(metaclass=Metaclass_DynamicEdges):
    from nav2_msgs.srv._dynamic_edges import DynamicEdges_Request as Request
    from nav2_msgs.srv._dynamic_edges import DynamicEdges_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
