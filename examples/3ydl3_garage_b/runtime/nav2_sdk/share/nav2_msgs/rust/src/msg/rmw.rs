#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__CollisionMonitorState() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__msg__CollisionMonitorState__init(msg: *mut CollisionMonitorState) -> bool;
    fn nav2_msgs__msg__CollisionMonitorState__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CollisionMonitorState>, size: usize) -> bool;
    fn nav2_msgs__msg__CollisionMonitorState__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CollisionMonitorState>);
    fn nav2_msgs__msg__CollisionMonitorState__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CollisionMonitorState>, out_seq: *mut rosidl_runtime_rs::Sequence<CollisionMonitorState>) -> bool;
}

// Corresponds to nav2_msgs__msg__CollisionMonitorState
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// Action type for robot in Collision Monitor

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CollisionMonitorState {

    // This member is not documented.
    #[allow(missing_docs)]
    pub action_type: u8,

    /// Name of triggered polygon
    pub polygon_name: rosidl_runtime_rs::String,

}

impl CollisionMonitorState {
    /// No action
    pub const DO_NOTHING: u8 = 0;

    /// Stop the robot
    pub const STOP: u8 = 1;

    /// Slowdown in percentage from current operating speed
    pub const SLOWDOWN: u8 = 2;

    /// Keep constant time interval before collision
    pub const APPROACH: u8 = 3;

}


impl Default for CollisionMonitorState {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__msg__CollisionMonitorState__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__msg__CollisionMonitorState__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CollisionMonitorState {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__CollisionMonitorState__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__CollisionMonitorState__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__CollisionMonitorState__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CollisionMonitorState {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CollisionMonitorState where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/msg/CollisionMonitorState";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__CollisionMonitorState() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__Costmap() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__msg__Costmap__init(msg: *mut Costmap) -> bool;
    fn nav2_msgs__msg__Costmap__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Costmap>, size: usize) -> bool;
    fn nav2_msgs__msg__Costmap__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Costmap>);
    fn nav2_msgs__msg__Costmap__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Costmap>, out_seq: *mut rosidl_runtime_rs::Sequence<Costmap>) -> bool;
}

// Corresponds to nav2_msgs__msg__Costmap
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// This represents a 2-D grid map, in which each cell has an associated cost

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Costmap {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,

    /// MetaData for the map
    pub metadata: super::super::msg::rmw::CostmapMetaData,

    /// The cost data, in row-major order, starting with (0,0).
    pub data: rosidl_runtime_rs::Sequence<u8>,

}



impl Default for Costmap {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__msg__Costmap__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__msg__Costmap__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Costmap {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__Costmap__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__Costmap__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__Costmap__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Costmap {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Costmap where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/msg/Costmap";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__Costmap() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__CostmapMetaData() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__msg__CostmapMetaData__init(msg: *mut CostmapMetaData) -> bool;
    fn nav2_msgs__msg__CostmapMetaData__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CostmapMetaData>, size: usize) -> bool;
    fn nav2_msgs__msg__CostmapMetaData__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CostmapMetaData>);
    fn nav2_msgs__msg__CostmapMetaData__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CostmapMetaData>, out_seq: *mut rosidl_runtime_rs::Sequence<CostmapMetaData>) -> bool;
}

// Corresponds to nav2_msgs__msg__CostmapMetaData
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// This hold basic information about the characteristics of the Costmap

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CostmapMetaData {
    /// The time at which the static map was loaded
    pub map_load_time: builtin_interfaces::msg::rmw::Time,

    /// The time of the last update to costmap
    pub update_time: builtin_interfaces::msg::rmw::Time,

    /// The corresponding layer name
    pub layer: rosidl_runtime_rs::String,

    /// The map resolution
    pub resolution: f32,

    /// Number of cells in the horizontal direction
    pub size_x: u32,

    /// Number of cells in the vertical direction
    pub size_y: u32,

    /// The origin of the costmap [m, m, rad].
    /// This is the real-world pose of the cell (0,0) in the map.
    pub origin: geometry_msgs::msg::rmw::Pose,

}



impl Default for CostmapMetaData {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__msg__CostmapMetaData__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__msg__CostmapMetaData__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CostmapMetaData {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__CostmapMetaData__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__CostmapMetaData__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__CostmapMetaData__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CostmapMetaData {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CostmapMetaData where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/msg/CostmapMetaData";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__CostmapMetaData() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__CostmapFilterInfo() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__msg__CostmapFilterInfo__init(msg: *mut CostmapFilterInfo) -> bool;
    fn nav2_msgs__msg__CostmapFilterInfo__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CostmapFilterInfo>, size: usize) -> bool;
    fn nav2_msgs__msg__CostmapFilterInfo__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CostmapFilterInfo>);
    fn nav2_msgs__msg__CostmapFilterInfo__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CostmapFilterInfo>, out_seq: *mut rosidl_runtime_rs::Sequence<CostmapFilterInfo>) -> bool;
}

// Corresponds to nav2_msgs__msg__CostmapFilterInfo
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CostmapFilterInfo {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,

    /// Type of plugin used (keepout filter, speed limit in m/s, speed limit in percent, etc...)
    /// 0: keepout/lanes filter
    /// 1: speed limit filter in % of maximum speed
    /// 2: speed limit filter in absolute values (m/s)
    pub type_: u8,

    /// Name of filter mask topic
    pub filter_mask_topic: rosidl_runtime_rs::String,

    /// Multiplier base offset and multiplier coefficient for conversion of OccGrid.
    /// Used to convert OccupancyGrid data values to filter space values.
    /// data -> into some other number space:
    /// space = data * multiplier + base
    pub base: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub multiplier: f32,

}



impl Default for CostmapFilterInfo {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__msg__CostmapFilterInfo__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__msg__CostmapFilterInfo__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CostmapFilterInfo {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__CostmapFilterInfo__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__CostmapFilterInfo__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__CostmapFilterInfo__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CostmapFilterInfo {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CostmapFilterInfo where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/msg/CostmapFilterInfo";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__CostmapFilterInfo() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__SpeedLimit() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__msg__SpeedLimit__init(msg: *mut SpeedLimit) -> bool;
    fn nav2_msgs__msg__SpeedLimit__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SpeedLimit>, size: usize) -> bool;
    fn nav2_msgs__msg__SpeedLimit__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SpeedLimit>);
    fn nav2_msgs__msg__SpeedLimit__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SpeedLimit>, out_seq: *mut rosidl_runtime_rs::Sequence<SpeedLimit>) -> bool;
}

// Corresponds to nav2_msgs__msg__SpeedLimit
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SpeedLimit {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,

    /// Setting speed limit in percentage if true or in absolute values in false case
    pub percentage: bool,

    /// Maximum allowed speed (in percent of maximum robot speed or in m/s depending
    /// on "percentage" value). When no-limit it is set to 0.0
    pub speed_limit: f64,

}



impl Default for SpeedLimit {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__msg__SpeedLimit__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__msg__SpeedLimit__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SpeedLimit {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__SpeedLimit__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__SpeedLimit__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__SpeedLimit__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SpeedLimit {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SpeedLimit where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/msg/SpeedLimit";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__SpeedLimit() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__VoxelGrid() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__msg__VoxelGrid__init(msg: *mut VoxelGrid) -> bool;
    fn nav2_msgs__msg__VoxelGrid__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<VoxelGrid>, size: usize) -> bool;
    fn nav2_msgs__msg__VoxelGrid__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<VoxelGrid>);
    fn nav2_msgs__msg__VoxelGrid__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<VoxelGrid>, out_seq: *mut rosidl_runtime_rs::Sequence<VoxelGrid>) -> bool;
}

// Corresponds to nav2_msgs__msg__VoxelGrid
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct VoxelGrid {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub data: rosidl_runtime_rs::Sequence<u32>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub origin: geometry_msgs::msg::rmw::Point32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub resolutions: geometry_msgs::msg::rmw::Vector3,


    // This member is not documented.
    #[allow(missing_docs)]
    pub size_x: u32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub size_y: u32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub size_z: u32,

}



impl Default for VoxelGrid {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__msg__VoxelGrid__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__msg__VoxelGrid__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for VoxelGrid {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__VoxelGrid__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__VoxelGrid__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__VoxelGrid__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for VoxelGrid {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for VoxelGrid where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/msg/VoxelGrid";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__VoxelGrid() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__BehaviorTreeStatusChange() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__msg__BehaviorTreeStatusChange__init(msg: *mut BehaviorTreeStatusChange) -> bool;
    fn nav2_msgs__msg__BehaviorTreeStatusChange__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<BehaviorTreeStatusChange>, size: usize) -> bool;
    fn nav2_msgs__msg__BehaviorTreeStatusChange__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<BehaviorTreeStatusChange>);
    fn nav2_msgs__msg__BehaviorTreeStatusChange__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<BehaviorTreeStatusChange>, out_seq: *mut rosidl_runtime_rs::Sequence<BehaviorTreeStatusChange>) -> bool;
}

// Corresponds to nav2_msgs__msg__BehaviorTreeStatusChange
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BehaviorTreeStatusChange {
    /// internal behavior tree event timestamp. Typically this is wall clock time
    pub timestamp: builtin_interfaces::msg::rmw::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub node_name: rosidl_runtime_rs::String,

    /// IDLE, RUNNING, SUCCESS or FAILURE
    pub previous_status: rosidl_runtime_rs::String,

    /// IDLE, RUNNING, SUCCESS or FAILURE
    pub current_status: rosidl_runtime_rs::String,

}



impl Default for BehaviorTreeStatusChange {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__msg__BehaviorTreeStatusChange__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__msg__BehaviorTreeStatusChange__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for BehaviorTreeStatusChange {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__BehaviorTreeStatusChange__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__BehaviorTreeStatusChange__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__BehaviorTreeStatusChange__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for BehaviorTreeStatusChange {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for BehaviorTreeStatusChange where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/msg/BehaviorTreeStatusChange";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__BehaviorTreeStatusChange() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__BehaviorTreeLog() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__msg__BehaviorTreeLog__init(msg: *mut BehaviorTreeLog) -> bool;
    fn nav2_msgs__msg__BehaviorTreeLog__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<BehaviorTreeLog>, size: usize) -> bool;
    fn nav2_msgs__msg__BehaviorTreeLog__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<BehaviorTreeLog>);
    fn nav2_msgs__msg__BehaviorTreeLog__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<BehaviorTreeLog>, out_seq: *mut rosidl_runtime_rs::Sequence<BehaviorTreeLog>) -> bool;
}

// Corresponds to nav2_msgs__msg__BehaviorTreeLog
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BehaviorTreeLog {
    /// ROS time that this log message was sent.
    pub timestamp: builtin_interfaces::msg::rmw::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub event_log: rosidl_runtime_rs::Sequence<super::super::msg::rmw::BehaviorTreeStatusChange>,

}



impl Default for BehaviorTreeLog {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__msg__BehaviorTreeLog__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__msg__BehaviorTreeLog__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for BehaviorTreeLog {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__BehaviorTreeLog__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__BehaviorTreeLog__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__BehaviorTreeLog__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for BehaviorTreeLog {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for BehaviorTreeLog where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/msg/BehaviorTreeLog";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__BehaviorTreeLog() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__Particle() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__msg__Particle__init(msg: *mut Particle) -> bool;
    fn nav2_msgs__msg__Particle__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Particle>, size: usize) -> bool;
    fn nav2_msgs__msg__Particle__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Particle>);
    fn nav2_msgs__msg__Particle__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Particle>, out_seq: *mut rosidl_runtime_rs::Sequence<Particle>) -> bool;
}

// Corresponds to nav2_msgs__msg__Particle
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// This represents an individual particle with weight produced by a particle filter

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Particle {

    // This member is not documented.
    #[allow(missing_docs)]
    pub pose: geometry_msgs::msg::rmw::Pose,


    // This member is not documented.
    #[allow(missing_docs)]
    pub weight: f64,

}



impl Default for Particle {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__msg__Particle__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__msg__Particle__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Particle {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__Particle__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__Particle__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__Particle__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Particle {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Particle where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/msg/Particle";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__Particle() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__ParticleCloud() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__msg__ParticleCloud__init(msg: *mut ParticleCloud) -> bool;
    fn nav2_msgs__msg__ParticleCloud__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ParticleCloud>, size: usize) -> bool;
    fn nav2_msgs__msg__ParticleCloud__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ParticleCloud>);
    fn nav2_msgs__msg__ParticleCloud__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ParticleCloud>, out_seq: *mut rosidl_runtime_rs::Sequence<ParticleCloud>) -> bool;
}

// Corresponds to nav2_msgs__msg__ParticleCloud
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// This represents a particle cloud containing particle poses and weights

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ParticleCloud {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,

    /// Array of particles in the cloud
    pub particles: rosidl_runtime_rs::Sequence<super::super::msg::rmw::Particle>,

}



impl Default for ParticleCloud {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__msg__ParticleCloud__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__msg__ParticleCloud__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ParticleCloud {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__ParticleCloud__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__ParticleCloud__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__ParticleCloud__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ParticleCloud {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ParticleCloud where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/msg/ParticleCloud";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__ParticleCloud() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__Route() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__msg__Route__init(msg: *mut Route) -> bool;
    fn nav2_msgs__msg__Route__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Route>, size: usize) -> bool;
    fn nav2_msgs__msg__Route__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Route>);
    fn nav2_msgs__msg__Route__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Route>, out_seq: *mut rosidl_runtime_rs::Sequence<Route>) -> bool;
}

// Corresponds to nav2_msgs__msg__Route
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Route {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub route_cost: f32,

    /// ordered set of nodes of the route
    pub nodes: rosidl_runtime_rs::Sequence<super::super::msg::rmw::RouteNode>,

    /// ordered set of edges that connect nodes
    pub edges: rosidl_runtime_rs::Sequence<super::super::msg::rmw::RouteEdge>,

}



impl Default for Route {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__msg__Route__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__msg__Route__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Route {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__Route__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__Route__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__Route__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Route {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Route where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/msg/Route";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__Route() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__RouteNode() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__msg__RouteNode__init(msg: *mut RouteNode) -> bool;
    fn nav2_msgs__msg__RouteNode__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<RouteNode>, size: usize) -> bool;
    fn nav2_msgs__msg__RouteNode__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<RouteNode>);
    fn nav2_msgs__msg__RouteNode__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<RouteNode>, out_seq: *mut rosidl_runtime_rs::Sequence<RouteNode>) -> bool;
}

// Corresponds to nav2_msgs__msg__RouteNode
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RouteNode {

    // This member is not documented.
    #[allow(missing_docs)]
    pub nodeid: u16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub position: geometry_msgs::msg::rmw::Point,

}



impl Default for RouteNode {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__msg__RouteNode__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__msg__RouteNode__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for RouteNode {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__RouteNode__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__RouteNode__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__RouteNode__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for RouteNode {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for RouteNode where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/msg/RouteNode";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__RouteNode() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__RouteEdge() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__msg__RouteEdge__init(msg: *mut RouteEdge) -> bool;
    fn nav2_msgs__msg__RouteEdge__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<RouteEdge>, size: usize) -> bool;
    fn nav2_msgs__msg__RouteEdge__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<RouteEdge>);
    fn nav2_msgs__msg__RouteEdge__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<RouteEdge>, out_seq: *mut rosidl_runtime_rs::Sequence<RouteEdge>) -> bool;
}

// Corresponds to nav2_msgs__msg__RouteEdge
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RouteEdge {

    // This member is not documented.
    #[allow(missing_docs)]
    pub edgeid: u16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub start: geometry_msgs::msg::rmw::Point,


    // This member is not documented.
    #[allow(missing_docs)]
    pub end: geometry_msgs::msg::rmw::Point,

}



impl Default for RouteEdge {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__msg__RouteEdge__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__msg__RouteEdge__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for RouteEdge {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__RouteEdge__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__RouteEdge__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__RouteEdge__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for RouteEdge {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for RouteEdge where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/msg/RouteEdge";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__RouteEdge() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__EdgeCost() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__msg__EdgeCost__init(msg: *mut EdgeCost) -> bool;
    fn nav2_msgs__msg__EdgeCost__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<EdgeCost>, size: usize) -> bool;
    fn nav2_msgs__msg__EdgeCost__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<EdgeCost>);
    fn nav2_msgs__msg__EdgeCost__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<EdgeCost>, out_seq: *mut rosidl_runtime_rs::Sequence<EdgeCost>) -> bool;
}

// Corresponds to nav2_msgs__msg__EdgeCost
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// Edge cost to use with nav2_msgs/srv/DynamicEdges to adjust route edge costs

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct EdgeCost {

    // This member is not documented.
    #[allow(missing_docs)]
    pub edgeid: u16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub cost: f32,

}



impl Default for EdgeCost {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__msg__EdgeCost__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__msg__EdgeCost__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for EdgeCost {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__EdgeCost__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__EdgeCost__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__msg__EdgeCost__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for EdgeCost {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for EdgeCost where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/msg/EdgeCost";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__EdgeCost() }
  }
}


