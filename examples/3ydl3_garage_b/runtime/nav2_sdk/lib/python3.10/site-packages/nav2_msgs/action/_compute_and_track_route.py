# generated from rosidl_generator_py/resource/_idl.py.em
# with input from nav2_msgs:action/ComputeAndTrackRoute.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ComputeAndTrackRoute_Goal(type):
    """Metaclass of message 'ComputeAndTrackRoute_Goal'."""

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
                'nav2_msgs.action.ComputeAndTrackRoute_Goal')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__compute_and_track_route__goal
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__compute_and_track_route__goal
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__compute_and_track_route__goal
            cls._TYPE_SUPPORT = module.type_support_msg__action__compute_and_track_route__goal
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__compute_and_track_route__goal

            from geometry_msgs.msg import PoseStamped
            if PoseStamped.__class__._TYPE_SUPPORT is None:
                PoseStamped.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ComputeAndTrackRoute_Goal(metaclass=Metaclass_ComputeAndTrackRoute_Goal):
    """Message class 'ComputeAndTrackRoute_Goal'."""

    __slots__ = [
        '_start_id',
        '_start',
        '_goal_id',
        '_goal',
        '_use_start',
        '_use_poses',
    ]

    _fields_and_field_types = {
        'start_id': 'uint16',
        'start': 'geometry_msgs/PoseStamped',
        'goal_id': 'uint16',
        'goal': 'geometry_msgs/PoseStamped',
        'use_start': 'boolean',
        'use_poses': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'PoseStamped'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'PoseStamped'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.start_id = kwargs.get('start_id', int())
        from geometry_msgs.msg import PoseStamped
        self.start = kwargs.get('start', PoseStamped())
        self.goal_id = kwargs.get('goal_id', int())
        from geometry_msgs.msg import PoseStamped
        self.goal = kwargs.get('goal', PoseStamped())
        self.use_start = kwargs.get('use_start', bool())
        self.use_poses = kwargs.get('use_poses', bool())

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
        if self.start_id != other.start_id:
            return False
        if self.start != other.start:
            return False
        if self.goal_id != other.goal_id:
            return False
        if self.goal != other.goal:
            return False
        if self.use_start != other.use_start:
            return False
        if self.use_poses != other.use_poses:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def start_id(self):
        """Message field 'start_id'."""
        return self._start_id

    @start_id.setter
    def start_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'start_id' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'start_id' field must be an unsigned integer in [0, 65535]"
        self._start_id = value

    @builtins.property
    def start(self):
        """Message field 'start'."""
        return self._start

    @start.setter
    def start(self, value):
        if __debug__:
            from geometry_msgs.msg import PoseStamped
            assert \
                isinstance(value, PoseStamped), \
                "The 'start' field must be a sub message of type 'PoseStamped'"
        self._start = value

    @builtins.property
    def goal_id(self):
        """Message field 'goal_id'."""
        return self._goal_id

    @goal_id.setter
    def goal_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'goal_id' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'goal_id' field must be an unsigned integer in [0, 65535]"
        self._goal_id = value

    @builtins.property
    def goal(self):
        """Message field 'goal'."""
        return self._goal

    @goal.setter
    def goal(self, value):
        if __debug__:
            from geometry_msgs.msg import PoseStamped
            assert \
                isinstance(value, PoseStamped), \
                "The 'goal' field must be a sub message of type 'PoseStamped'"
        self._goal = value

    @builtins.property
    def use_start(self):
        """Message field 'use_start'."""
        return self._use_start

    @use_start.setter
    def use_start(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'use_start' field must be of type 'bool'"
        self._use_start = value

    @builtins.property
    def use_poses(self):
        """Message field 'use_poses'."""
        return self._use_poses

    @use_poses.setter
    def use_poses(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'use_poses' field must be of type 'bool'"
        self._use_poses = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_ComputeAndTrackRoute_Result(type):
    """Metaclass of message 'ComputeAndTrackRoute_Result'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'NONE': 0,
        'UNKNOWN': 400,
        'TF_ERROR': 401,
        'NO_VALID_GRAPH': 402,
        'INDETERMINANT_NODES_ON_GRAPH': 403,
        'TIMEOUT': 404,
        'NO_VALID_ROUTE': 405,
        'OPERATION_FAILED': 406,
        'INVALID_EDGE_SCORER_USE': 407,
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
                'nav2_msgs.action.ComputeAndTrackRoute_Result')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__compute_and_track_route__result
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__compute_and_track_route__result
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__compute_and_track_route__result
            cls._TYPE_SUPPORT = module.type_support_msg__action__compute_and_track_route__result
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__compute_and_track_route__result

            from builtin_interfaces.msg import Duration
            if Duration.__class__._TYPE_SUPPORT is None:
                Duration.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'NONE': cls.__constants['NONE'],
            'UNKNOWN': cls.__constants['UNKNOWN'],
            'TF_ERROR': cls.__constants['TF_ERROR'],
            'NO_VALID_GRAPH': cls.__constants['NO_VALID_GRAPH'],
            'INDETERMINANT_NODES_ON_GRAPH': cls.__constants['INDETERMINANT_NODES_ON_GRAPH'],
            'TIMEOUT': cls.__constants['TIMEOUT'],
            'NO_VALID_ROUTE': cls.__constants['NO_VALID_ROUTE'],
            'OPERATION_FAILED': cls.__constants['OPERATION_FAILED'],
            'INVALID_EDGE_SCORER_USE': cls.__constants['INVALID_EDGE_SCORER_USE'],
        }

    @property
    def NONE(self):
        """Message constant 'NONE'."""
        return Metaclass_ComputeAndTrackRoute_Result.__constants['NONE']

    @property
    def UNKNOWN(self):
        """Message constant 'UNKNOWN'."""
        return Metaclass_ComputeAndTrackRoute_Result.__constants['UNKNOWN']

    @property
    def TF_ERROR(self):
        """Message constant 'TF_ERROR'."""
        return Metaclass_ComputeAndTrackRoute_Result.__constants['TF_ERROR']

    @property
    def NO_VALID_GRAPH(self):
        """Message constant 'NO_VALID_GRAPH'."""
        return Metaclass_ComputeAndTrackRoute_Result.__constants['NO_VALID_GRAPH']

    @property
    def INDETERMINANT_NODES_ON_GRAPH(self):
        """Message constant 'INDETERMINANT_NODES_ON_GRAPH'."""
        return Metaclass_ComputeAndTrackRoute_Result.__constants['INDETERMINANT_NODES_ON_GRAPH']

    @property
    def TIMEOUT(self):
        """Message constant 'TIMEOUT'."""
        return Metaclass_ComputeAndTrackRoute_Result.__constants['TIMEOUT']

    @property
    def NO_VALID_ROUTE(self):
        """Message constant 'NO_VALID_ROUTE'."""
        return Metaclass_ComputeAndTrackRoute_Result.__constants['NO_VALID_ROUTE']

    @property
    def OPERATION_FAILED(self):
        """Message constant 'OPERATION_FAILED'."""
        return Metaclass_ComputeAndTrackRoute_Result.__constants['OPERATION_FAILED']

    @property
    def INVALID_EDGE_SCORER_USE(self):
        """Message constant 'INVALID_EDGE_SCORER_USE'."""
        return Metaclass_ComputeAndTrackRoute_Result.__constants['INVALID_EDGE_SCORER_USE']


class ComputeAndTrackRoute_Result(metaclass=Metaclass_ComputeAndTrackRoute_Result):
    """
    Message class 'ComputeAndTrackRoute_Result'.

    Constants:
      NONE
      UNKNOWN
      TF_ERROR
      NO_VALID_GRAPH
      INDETERMINANT_NODES_ON_GRAPH
      TIMEOUT
      NO_VALID_ROUTE
      OPERATION_FAILED
      INVALID_EDGE_SCORER_USE
    """

    __slots__ = [
        '_execution_duration',
    ]

    _fields_and_field_types = {
        'execution_duration': 'builtin_interfaces/Duration',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Duration'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from builtin_interfaces.msg import Duration
        self.execution_duration = kwargs.get('execution_duration', Duration())

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
        if self.execution_duration != other.execution_duration:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def execution_duration(self):
        """Message field 'execution_duration'."""
        return self._execution_duration

    @execution_duration.setter
    def execution_duration(self, value):
        if __debug__:
            from builtin_interfaces.msg import Duration
            assert \
                isinstance(value, Duration), \
                "The 'execution_duration' field must be a sub message of type 'Duration'"
        self._execution_duration = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_ComputeAndTrackRoute_Feedback(type):
    """Metaclass of message 'ComputeAndTrackRoute_Feedback'."""

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
                'nav2_msgs.action.ComputeAndTrackRoute_Feedback')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__compute_and_track_route__feedback
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__compute_and_track_route__feedback
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__compute_and_track_route__feedback
            cls._TYPE_SUPPORT = module.type_support_msg__action__compute_and_track_route__feedback
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__compute_and_track_route__feedback

            from nav2_msgs.msg import Route
            if Route.__class__._TYPE_SUPPORT is None:
                Route.__class__.__import_type_support__()

            from nav_msgs.msg import Path
            if Path.__class__._TYPE_SUPPORT is None:
                Path.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ComputeAndTrackRoute_Feedback(metaclass=Metaclass_ComputeAndTrackRoute_Feedback):
    """Message class 'ComputeAndTrackRoute_Feedback'."""

    __slots__ = [
        '_last_node_id',
        '_next_node_id',
        '_current_edge_id',
        '_route',
        '_path',
        '_operations_triggered',
        '_rerouted',
    ]

    _fields_and_field_types = {
        'last_node_id': 'uint16',
        'next_node_id': 'uint16',
        'current_edge_id': 'uint16',
        'route': 'nav2_msgs/Route',
        'path': 'nav_msgs/Path',
        'operations_triggered': 'sequence<string>',
        'rerouted': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['nav2_msgs', 'msg'], 'Route'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['nav_msgs', 'msg'], 'Path'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.UnboundedString()),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.last_node_id = kwargs.get('last_node_id', int())
        self.next_node_id = kwargs.get('next_node_id', int())
        self.current_edge_id = kwargs.get('current_edge_id', int())
        from nav2_msgs.msg import Route
        self.route = kwargs.get('route', Route())
        from nav_msgs.msg import Path
        self.path = kwargs.get('path', Path())
        self.operations_triggered = kwargs.get('operations_triggered', [])
        self.rerouted = kwargs.get('rerouted', bool())

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
        if self.last_node_id != other.last_node_id:
            return False
        if self.next_node_id != other.next_node_id:
            return False
        if self.current_edge_id != other.current_edge_id:
            return False
        if self.route != other.route:
            return False
        if self.path != other.path:
            return False
        if self.operations_triggered != other.operations_triggered:
            return False
        if self.rerouted != other.rerouted:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def last_node_id(self):
        """Message field 'last_node_id'."""
        return self._last_node_id

    @last_node_id.setter
    def last_node_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'last_node_id' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'last_node_id' field must be an unsigned integer in [0, 65535]"
        self._last_node_id = value

    @builtins.property
    def next_node_id(self):
        """Message field 'next_node_id'."""
        return self._next_node_id

    @next_node_id.setter
    def next_node_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'next_node_id' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'next_node_id' field must be an unsigned integer in [0, 65535]"
        self._next_node_id = value

    @builtins.property
    def current_edge_id(self):
        """Message field 'current_edge_id'."""
        return self._current_edge_id

    @current_edge_id.setter
    def current_edge_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'current_edge_id' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'current_edge_id' field must be an unsigned integer in [0, 65535]"
        self._current_edge_id = value

    @builtins.property
    def route(self):
        """Message field 'route'."""
        return self._route

    @route.setter
    def route(self, value):
        if __debug__:
            from nav2_msgs.msg import Route
            assert \
                isinstance(value, Route), \
                "The 'route' field must be a sub message of type 'Route'"
        self._route = value

    @builtins.property
    def path(self):
        """Message field 'path'."""
        return self._path

    @path.setter
    def path(self, value):
        if __debug__:
            from nav_msgs.msg import Path
            assert \
                isinstance(value, Path), \
                "The 'path' field must be a sub message of type 'Path'"
        self._path = value

    @builtins.property
    def operations_triggered(self):
        """Message field 'operations_triggered'."""
        return self._operations_triggered

    @operations_triggered.setter
    def operations_triggered(self, value):
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
                 all(isinstance(v, str) for v in value) and
                 True), \
                "The 'operations_triggered' field must be a set or sequence and each value of type 'str'"
        self._operations_triggered = value

    @builtins.property
    def rerouted(self):
        """Message field 'rerouted'."""
        return self._rerouted

    @rerouted.setter
    def rerouted(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'rerouted' field must be of type 'bool'"
        self._rerouted = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_ComputeAndTrackRoute_SendGoal_Request(type):
    """Metaclass of message 'ComputeAndTrackRoute_SendGoal_Request'."""

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
                'nav2_msgs.action.ComputeAndTrackRoute_SendGoal_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__compute_and_track_route__send_goal__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__compute_and_track_route__send_goal__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__compute_and_track_route__send_goal__request
            cls._TYPE_SUPPORT = module.type_support_msg__action__compute_and_track_route__send_goal__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__compute_and_track_route__send_goal__request

            from nav2_msgs.action import ComputeAndTrackRoute
            if ComputeAndTrackRoute.Goal.__class__._TYPE_SUPPORT is None:
                ComputeAndTrackRoute.Goal.__class__.__import_type_support__()

            from unique_identifier_msgs.msg import UUID
            if UUID.__class__._TYPE_SUPPORT is None:
                UUID.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ComputeAndTrackRoute_SendGoal_Request(metaclass=Metaclass_ComputeAndTrackRoute_SendGoal_Request):
    """Message class 'ComputeAndTrackRoute_SendGoal_Request'."""

    __slots__ = [
        '_goal_id',
        '_goal',
    ]

    _fields_and_field_types = {
        'goal_id': 'unique_identifier_msgs/UUID',
        'goal': 'nav2_msgs/ComputeAndTrackRoute_Goal',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['unique_identifier_msgs', 'msg'], 'UUID'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['nav2_msgs', 'action'], 'ComputeAndTrackRoute_Goal'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from unique_identifier_msgs.msg import UUID
        self.goal_id = kwargs.get('goal_id', UUID())
        from nav2_msgs.action._compute_and_track_route import ComputeAndTrackRoute_Goal
        self.goal = kwargs.get('goal', ComputeAndTrackRoute_Goal())

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
        if self.goal_id != other.goal_id:
            return False
        if self.goal != other.goal:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def goal_id(self):
        """Message field 'goal_id'."""
        return self._goal_id

    @goal_id.setter
    def goal_id(self, value):
        if __debug__:
            from unique_identifier_msgs.msg import UUID
            assert \
                isinstance(value, UUID), \
                "The 'goal_id' field must be a sub message of type 'UUID'"
        self._goal_id = value

    @builtins.property
    def goal(self):
        """Message field 'goal'."""
        return self._goal

    @goal.setter
    def goal(self, value):
        if __debug__:
            from nav2_msgs.action._compute_and_track_route import ComputeAndTrackRoute_Goal
            assert \
                isinstance(value, ComputeAndTrackRoute_Goal), \
                "The 'goal' field must be a sub message of type 'ComputeAndTrackRoute_Goal'"
        self._goal = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_ComputeAndTrackRoute_SendGoal_Response(type):
    """Metaclass of message 'ComputeAndTrackRoute_SendGoal_Response'."""

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
                'nav2_msgs.action.ComputeAndTrackRoute_SendGoal_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__compute_and_track_route__send_goal__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__compute_and_track_route__send_goal__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__compute_and_track_route__send_goal__response
            cls._TYPE_SUPPORT = module.type_support_msg__action__compute_and_track_route__send_goal__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__compute_and_track_route__send_goal__response

            from builtin_interfaces.msg import Time
            if Time.__class__._TYPE_SUPPORT is None:
                Time.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ComputeAndTrackRoute_SendGoal_Response(metaclass=Metaclass_ComputeAndTrackRoute_SendGoal_Response):
    """Message class 'ComputeAndTrackRoute_SendGoal_Response'."""

    __slots__ = [
        '_accepted',
        '_stamp',
    ]

    _fields_and_field_types = {
        'accepted': 'boolean',
        'stamp': 'builtin_interfaces/Time',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.accepted = kwargs.get('accepted', bool())
        from builtin_interfaces.msg import Time
        self.stamp = kwargs.get('stamp', Time())

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
        if self.accepted != other.accepted:
            return False
        if self.stamp != other.stamp:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def accepted(self):
        """Message field 'accepted'."""
        return self._accepted

    @accepted.setter
    def accepted(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'accepted' field must be of type 'bool'"
        self._accepted = value

    @builtins.property
    def stamp(self):
        """Message field 'stamp'."""
        return self._stamp

    @stamp.setter
    def stamp(self, value):
        if __debug__:
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 'stamp' field must be a sub message of type 'Time'"
        self._stamp = value


class Metaclass_ComputeAndTrackRoute_SendGoal(type):
    """Metaclass of service 'ComputeAndTrackRoute_SendGoal'."""

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
                'nav2_msgs.action.ComputeAndTrackRoute_SendGoal')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__action__compute_and_track_route__send_goal

            from nav2_msgs.action import _compute_and_track_route
            if _compute_and_track_route.Metaclass_ComputeAndTrackRoute_SendGoal_Request._TYPE_SUPPORT is None:
                _compute_and_track_route.Metaclass_ComputeAndTrackRoute_SendGoal_Request.__import_type_support__()
            if _compute_and_track_route.Metaclass_ComputeAndTrackRoute_SendGoal_Response._TYPE_SUPPORT is None:
                _compute_and_track_route.Metaclass_ComputeAndTrackRoute_SendGoal_Response.__import_type_support__()


class ComputeAndTrackRoute_SendGoal(metaclass=Metaclass_ComputeAndTrackRoute_SendGoal):
    from nav2_msgs.action._compute_and_track_route import ComputeAndTrackRoute_SendGoal_Request as Request
    from nav2_msgs.action._compute_and_track_route import ComputeAndTrackRoute_SendGoal_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_ComputeAndTrackRoute_GetResult_Request(type):
    """Metaclass of message 'ComputeAndTrackRoute_GetResult_Request'."""

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
                'nav2_msgs.action.ComputeAndTrackRoute_GetResult_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__compute_and_track_route__get_result__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__compute_and_track_route__get_result__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__compute_and_track_route__get_result__request
            cls._TYPE_SUPPORT = module.type_support_msg__action__compute_and_track_route__get_result__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__compute_and_track_route__get_result__request

            from unique_identifier_msgs.msg import UUID
            if UUID.__class__._TYPE_SUPPORT is None:
                UUID.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ComputeAndTrackRoute_GetResult_Request(metaclass=Metaclass_ComputeAndTrackRoute_GetResult_Request):
    """Message class 'ComputeAndTrackRoute_GetResult_Request'."""

    __slots__ = [
        '_goal_id',
    ]

    _fields_and_field_types = {
        'goal_id': 'unique_identifier_msgs/UUID',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['unique_identifier_msgs', 'msg'], 'UUID'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from unique_identifier_msgs.msg import UUID
        self.goal_id = kwargs.get('goal_id', UUID())

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
        if self.goal_id != other.goal_id:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def goal_id(self):
        """Message field 'goal_id'."""
        return self._goal_id

    @goal_id.setter
    def goal_id(self, value):
        if __debug__:
            from unique_identifier_msgs.msg import UUID
            assert \
                isinstance(value, UUID), \
                "The 'goal_id' field must be a sub message of type 'UUID'"
        self._goal_id = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_ComputeAndTrackRoute_GetResult_Response(type):
    """Metaclass of message 'ComputeAndTrackRoute_GetResult_Response'."""

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
                'nav2_msgs.action.ComputeAndTrackRoute_GetResult_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__compute_and_track_route__get_result__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__compute_and_track_route__get_result__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__compute_and_track_route__get_result__response
            cls._TYPE_SUPPORT = module.type_support_msg__action__compute_and_track_route__get_result__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__compute_and_track_route__get_result__response

            from nav2_msgs.action import ComputeAndTrackRoute
            if ComputeAndTrackRoute.Result.__class__._TYPE_SUPPORT is None:
                ComputeAndTrackRoute.Result.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ComputeAndTrackRoute_GetResult_Response(metaclass=Metaclass_ComputeAndTrackRoute_GetResult_Response):
    """Message class 'ComputeAndTrackRoute_GetResult_Response'."""

    __slots__ = [
        '_status',
        '_result',
    ]

    _fields_and_field_types = {
        'status': 'int8',
        'result': 'nav2_msgs/ComputeAndTrackRoute_Result',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['nav2_msgs', 'action'], 'ComputeAndTrackRoute_Result'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.status = kwargs.get('status', int())
        from nav2_msgs.action._compute_and_track_route import ComputeAndTrackRoute_Result
        self.result = kwargs.get('result', ComputeAndTrackRoute_Result())

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
        if self.status != other.status:
            return False
        if self.result != other.result:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def status(self):
        """Message field 'status'."""
        return self._status

    @status.setter
    def status(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'status' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'status' field must be an integer in [-128, 127]"
        self._status = value

    @builtins.property
    def result(self):
        """Message field 'result'."""
        return self._result

    @result.setter
    def result(self, value):
        if __debug__:
            from nav2_msgs.action._compute_and_track_route import ComputeAndTrackRoute_Result
            assert \
                isinstance(value, ComputeAndTrackRoute_Result), \
                "The 'result' field must be a sub message of type 'ComputeAndTrackRoute_Result'"
        self._result = value


class Metaclass_ComputeAndTrackRoute_GetResult(type):
    """Metaclass of service 'ComputeAndTrackRoute_GetResult'."""

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
                'nav2_msgs.action.ComputeAndTrackRoute_GetResult')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__action__compute_and_track_route__get_result

            from nav2_msgs.action import _compute_and_track_route
            if _compute_and_track_route.Metaclass_ComputeAndTrackRoute_GetResult_Request._TYPE_SUPPORT is None:
                _compute_and_track_route.Metaclass_ComputeAndTrackRoute_GetResult_Request.__import_type_support__()
            if _compute_and_track_route.Metaclass_ComputeAndTrackRoute_GetResult_Response._TYPE_SUPPORT is None:
                _compute_and_track_route.Metaclass_ComputeAndTrackRoute_GetResult_Response.__import_type_support__()


class ComputeAndTrackRoute_GetResult(metaclass=Metaclass_ComputeAndTrackRoute_GetResult):
    from nav2_msgs.action._compute_and_track_route import ComputeAndTrackRoute_GetResult_Request as Request
    from nav2_msgs.action._compute_and_track_route import ComputeAndTrackRoute_GetResult_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_ComputeAndTrackRoute_FeedbackMessage(type):
    """Metaclass of message 'ComputeAndTrackRoute_FeedbackMessage'."""

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
                'nav2_msgs.action.ComputeAndTrackRoute_FeedbackMessage')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__compute_and_track_route__feedback_message
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__compute_and_track_route__feedback_message
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__compute_and_track_route__feedback_message
            cls._TYPE_SUPPORT = module.type_support_msg__action__compute_and_track_route__feedback_message
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__compute_and_track_route__feedback_message

            from nav2_msgs.action import ComputeAndTrackRoute
            if ComputeAndTrackRoute.Feedback.__class__._TYPE_SUPPORT is None:
                ComputeAndTrackRoute.Feedback.__class__.__import_type_support__()

            from unique_identifier_msgs.msg import UUID
            if UUID.__class__._TYPE_SUPPORT is None:
                UUID.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ComputeAndTrackRoute_FeedbackMessage(metaclass=Metaclass_ComputeAndTrackRoute_FeedbackMessage):
    """Message class 'ComputeAndTrackRoute_FeedbackMessage'."""

    __slots__ = [
        '_goal_id',
        '_feedback',
    ]

    _fields_and_field_types = {
        'goal_id': 'unique_identifier_msgs/UUID',
        'feedback': 'nav2_msgs/ComputeAndTrackRoute_Feedback',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['unique_identifier_msgs', 'msg'], 'UUID'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['nav2_msgs', 'action'], 'ComputeAndTrackRoute_Feedback'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from unique_identifier_msgs.msg import UUID
        self.goal_id = kwargs.get('goal_id', UUID())
        from nav2_msgs.action._compute_and_track_route import ComputeAndTrackRoute_Feedback
        self.feedback = kwargs.get('feedback', ComputeAndTrackRoute_Feedback())

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
        if self.goal_id != other.goal_id:
            return False
        if self.feedback != other.feedback:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def goal_id(self):
        """Message field 'goal_id'."""
        return self._goal_id

    @goal_id.setter
    def goal_id(self, value):
        if __debug__:
            from unique_identifier_msgs.msg import UUID
            assert \
                isinstance(value, UUID), \
                "The 'goal_id' field must be a sub message of type 'UUID'"
        self._goal_id = value

    @builtins.property
    def feedback(self):
        """Message field 'feedback'."""
        return self._feedback

    @feedback.setter
    def feedback(self, value):
        if __debug__:
            from nav2_msgs.action._compute_and_track_route import ComputeAndTrackRoute_Feedback
            assert \
                isinstance(value, ComputeAndTrackRoute_Feedback), \
                "The 'feedback' field must be a sub message of type 'ComputeAndTrackRoute_Feedback'"
        self._feedback = value


class Metaclass_ComputeAndTrackRoute(type):
    """Metaclass of action 'ComputeAndTrackRoute'."""

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
                'nav2_msgs.action.ComputeAndTrackRoute')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_action__action__compute_and_track_route

            from action_msgs.msg import _goal_status_array
            if _goal_status_array.Metaclass_GoalStatusArray._TYPE_SUPPORT is None:
                _goal_status_array.Metaclass_GoalStatusArray.__import_type_support__()
            from action_msgs.srv import _cancel_goal
            if _cancel_goal.Metaclass_CancelGoal._TYPE_SUPPORT is None:
                _cancel_goal.Metaclass_CancelGoal.__import_type_support__()

            from nav2_msgs.action import _compute_and_track_route
            if _compute_and_track_route.Metaclass_ComputeAndTrackRoute_SendGoal._TYPE_SUPPORT is None:
                _compute_and_track_route.Metaclass_ComputeAndTrackRoute_SendGoal.__import_type_support__()
            if _compute_and_track_route.Metaclass_ComputeAndTrackRoute_GetResult._TYPE_SUPPORT is None:
                _compute_and_track_route.Metaclass_ComputeAndTrackRoute_GetResult.__import_type_support__()
            if _compute_and_track_route.Metaclass_ComputeAndTrackRoute_FeedbackMessage._TYPE_SUPPORT is None:
                _compute_and_track_route.Metaclass_ComputeAndTrackRoute_FeedbackMessage.__import_type_support__()


class ComputeAndTrackRoute(metaclass=Metaclass_ComputeAndTrackRoute):

    # The goal message defined in the action definition.
    from nav2_msgs.action._compute_and_track_route import ComputeAndTrackRoute_Goal as Goal
    # The result message defined in the action definition.
    from nav2_msgs.action._compute_and_track_route import ComputeAndTrackRoute_Result as Result
    # The feedback message defined in the action definition.
    from nav2_msgs.action._compute_and_track_route import ComputeAndTrackRoute_Feedback as Feedback

    class Impl:

        # The send_goal service using a wrapped version of the goal message as a request.
        from nav2_msgs.action._compute_and_track_route import ComputeAndTrackRoute_SendGoal as SendGoalService
        # The get_result service using a wrapped version of the result message as a response.
        from nav2_msgs.action._compute_and_track_route import ComputeAndTrackRoute_GetResult as GetResultService
        # The feedback message with generic fields which wraps the feedback message.
        from nav2_msgs.action._compute_and_track_route import ComputeAndTrackRoute_FeedbackMessage as FeedbackMessage

        # The generic service to cancel a goal.
        from action_msgs.srv._cancel_goal import CancelGoal as CancelGoalService
        # The generic message for get the status of a goal.
        from action_msgs.msg._goal_status_array import GoalStatusArray as GoalStatusMessage

    def __init__(self):
        raise NotImplementedError('Action classes can not be instantiated')
