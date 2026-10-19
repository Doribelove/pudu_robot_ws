#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__GetCostmap_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__srv__GetCostmap_Request__init(msg: *mut GetCostmap_Request) -> bool;
    fn nav2_msgs__srv__GetCostmap_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GetCostmap_Request>, size: usize) -> bool;
    fn nav2_msgs__srv__GetCostmap_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GetCostmap_Request>);
    fn nav2_msgs__srv__GetCostmap_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GetCostmap_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<GetCostmap_Request>) -> bool;
}

// Corresponds to nav2_msgs__srv__GetCostmap_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetCostmap_Request {
    /// Specifications for the requested costmap
    pub specs: super::super::msg::rmw::CostmapMetaData,

}



impl Default for GetCostmap_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__srv__GetCostmap_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__srv__GetCostmap_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GetCostmap_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__GetCostmap_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__GetCostmap_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__GetCostmap_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GetCostmap_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GetCostmap_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/srv/GetCostmap_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__GetCostmap_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__GetCostmap_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__srv__GetCostmap_Response__init(msg: *mut GetCostmap_Response) -> bool;
    fn nav2_msgs__srv__GetCostmap_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GetCostmap_Response>, size: usize) -> bool;
    fn nav2_msgs__srv__GetCostmap_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GetCostmap_Response>);
    fn nav2_msgs__srv__GetCostmap_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GetCostmap_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<GetCostmap_Response>) -> bool;
}

// Corresponds to nav2_msgs__srv__GetCostmap_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetCostmap_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub map: super::super::msg::rmw::Costmap,

}



impl Default for GetCostmap_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__srv__GetCostmap_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__srv__GetCostmap_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GetCostmap_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__GetCostmap_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__GetCostmap_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__GetCostmap_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GetCostmap_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GetCostmap_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/srv/GetCostmap_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__GetCostmap_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__IsPathValid_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__srv__IsPathValid_Request__init(msg: *mut IsPathValid_Request) -> bool;
    fn nav2_msgs__srv__IsPathValid_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<IsPathValid_Request>, size: usize) -> bool;
    fn nav2_msgs__srv__IsPathValid_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<IsPathValid_Request>);
    fn nav2_msgs__srv__IsPathValid_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<IsPathValid_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<IsPathValid_Request>) -> bool;
}

// Corresponds to nav2_msgs__srv__IsPathValid_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct IsPathValid_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub path: nav_msgs::msg::rmw::Path,

}



impl Default for IsPathValid_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__srv__IsPathValid_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__srv__IsPathValid_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for IsPathValid_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__IsPathValid_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__IsPathValid_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__IsPathValid_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for IsPathValid_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for IsPathValid_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/srv/IsPathValid_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__IsPathValid_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__IsPathValid_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__srv__IsPathValid_Response__init(msg: *mut IsPathValid_Response) -> bool;
    fn nav2_msgs__srv__IsPathValid_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<IsPathValid_Response>, size: usize) -> bool;
    fn nav2_msgs__srv__IsPathValid_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<IsPathValid_Response>);
    fn nav2_msgs__srv__IsPathValid_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<IsPathValid_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<IsPathValid_Response>) -> bool;
}

// Corresponds to nav2_msgs__srv__IsPathValid_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct IsPathValid_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub is_valid: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub invalid_pose_indices: rosidl_runtime_rs::Sequence<i32>,

}



impl Default for IsPathValid_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__srv__IsPathValid_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__srv__IsPathValid_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for IsPathValid_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__IsPathValid_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__IsPathValid_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__IsPathValid_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for IsPathValid_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for IsPathValid_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/srv/IsPathValid_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__IsPathValid_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__ClearCostmapExceptRegion_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__srv__ClearCostmapExceptRegion_Request__init(msg: *mut ClearCostmapExceptRegion_Request) -> bool;
    fn nav2_msgs__srv__ClearCostmapExceptRegion_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ClearCostmapExceptRegion_Request>, size: usize) -> bool;
    fn nav2_msgs__srv__ClearCostmapExceptRegion_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ClearCostmapExceptRegion_Request>);
    fn nav2_msgs__srv__ClearCostmapExceptRegion_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ClearCostmapExceptRegion_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<ClearCostmapExceptRegion_Request>) -> bool;
}

// Corresponds to nav2_msgs__srv__ClearCostmapExceptRegion_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ClearCostmapExceptRegion_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub reset_distance: f32,

}



impl Default for ClearCostmapExceptRegion_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__srv__ClearCostmapExceptRegion_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__srv__ClearCostmapExceptRegion_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ClearCostmapExceptRegion_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__ClearCostmapExceptRegion_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__ClearCostmapExceptRegion_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__ClearCostmapExceptRegion_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ClearCostmapExceptRegion_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ClearCostmapExceptRegion_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/srv/ClearCostmapExceptRegion_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__ClearCostmapExceptRegion_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__ClearCostmapExceptRegion_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__srv__ClearCostmapExceptRegion_Response__init(msg: *mut ClearCostmapExceptRegion_Response) -> bool;
    fn nav2_msgs__srv__ClearCostmapExceptRegion_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ClearCostmapExceptRegion_Response>, size: usize) -> bool;
    fn nav2_msgs__srv__ClearCostmapExceptRegion_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ClearCostmapExceptRegion_Response>);
    fn nav2_msgs__srv__ClearCostmapExceptRegion_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ClearCostmapExceptRegion_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<ClearCostmapExceptRegion_Response>) -> bool;
}

// Corresponds to nav2_msgs__srv__ClearCostmapExceptRegion_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ClearCostmapExceptRegion_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub response: std_msgs::msg::rmw::Empty,

}



impl Default for ClearCostmapExceptRegion_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__srv__ClearCostmapExceptRegion_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__srv__ClearCostmapExceptRegion_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ClearCostmapExceptRegion_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__ClearCostmapExceptRegion_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__ClearCostmapExceptRegion_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__ClearCostmapExceptRegion_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ClearCostmapExceptRegion_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ClearCostmapExceptRegion_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/srv/ClearCostmapExceptRegion_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__ClearCostmapExceptRegion_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__ClearCostmapAroundRobot_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__srv__ClearCostmapAroundRobot_Request__init(msg: *mut ClearCostmapAroundRobot_Request) -> bool;
    fn nav2_msgs__srv__ClearCostmapAroundRobot_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ClearCostmapAroundRobot_Request>, size: usize) -> bool;
    fn nav2_msgs__srv__ClearCostmapAroundRobot_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ClearCostmapAroundRobot_Request>);
    fn nav2_msgs__srv__ClearCostmapAroundRobot_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ClearCostmapAroundRobot_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<ClearCostmapAroundRobot_Request>) -> bool;
}

// Corresponds to nav2_msgs__srv__ClearCostmapAroundRobot_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ClearCostmapAroundRobot_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub reset_distance: f32,

}



impl Default for ClearCostmapAroundRobot_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__srv__ClearCostmapAroundRobot_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__srv__ClearCostmapAroundRobot_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ClearCostmapAroundRobot_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__ClearCostmapAroundRobot_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__ClearCostmapAroundRobot_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__ClearCostmapAroundRobot_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ClearCostmapAroundRobot_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ClearCostmapAroundRobot_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/srv/ClearCostmapAroundRobot_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__ClearCostmapAroundRobot_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__ClearCostmapAroundRobot_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__srv__ClearCostmapAroundRobot_Response__init(msg: *mut ClearCostmapAroundRobot_Response) -> bool;
    fn nav2_msgs__srv__ClearCostmapAroundRobot_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ClearCostmapAroundRobot_Response>, size: usize) -> bool;
    fn nav2_msgs__srv__ClearCostmapAroundRobot_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ClearCostmapAroundRobot_Response>);
    fn nav2_msgs__srv__ClearCostmapAroundRobot_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ClearCostmapAroundRobot_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<ClearCostmapAroundRobot_Response>) -> bool;
}

// Corresponds to nav2_msgs__srv__ClearCostmapAroundRobot_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ClearCostmapAroundRobot_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub response: std_msgs::msg::rmw::Empty,

}



impl Default for ClearCostmapAroundRobot_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__srv__ClearCostmapAroundRobot_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__srv__ClearCostmapAroundRobot_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ClearCostmapAroundRobot_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__ClearCostmapAroundRobot_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__ClearCostmapAroundRobot_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__ClearCostmapAroundRobot_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ClearCostmapAroundRobot_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ClearCostmapAroundRobot_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/srv/ClearCostmapAroundRobot_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__ClearCostmapAroundRobot_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__ClearEntireCostmap_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__srv__ClearEntireCostmap_Request__init(msg: *mut ClearEntireCostmap_Request) -> bool;
    fn nav2_msgs__srv__ClearEntireCostmap_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ClearEntireCostmap_Request>, size: usize) -> bool;
    fn nav2_msgs__srv__ClearEntireCostmap_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ClearEntireCostmap_Request>);
    fn nav2_msgs__srv__ClearEntireCostmap_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ClearEntireCostmap_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<ClearEntireCostmap_Request>) -> bool;
}

// Corresponds to nav2_msgs__srv__ClearEntireCostmap_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ClearEntireCostmap_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub request: std_msgs::msg::rmw::Empty,

}



impl Default for ClearEntireCostmap_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__srv__ClearEntireCostmap_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__srv__ClearEntireCostmap_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ClearEntireCostmap_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__ClearEntireCostmap_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__ClearEntireCostmap_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__ClearEntireCostmap_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ClearEntireCostmap_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ClearEntireCostmap_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/srv/ClearEntireCostmap_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__ClearEntireCostmap_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__ClearEntireCostmap_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__srv__ClearEntireCostmap_Response__init(msg: *mut ClearEntireCostmap_Response) -> bool;
    fn nav2_msgs__srv__ClearEntireCostmap_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ClearEntireCostmap_Response>, size: usize) -> bool;
    fn nav2_msgs__srv__ClearEntireCostmap_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ClearEntireCostmap_Response>);
    fn nav2_msgs__srv__ClearEntireCostmap_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ClearEntireCostmap_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<ClearEntireCostmap_Response>) -> bool;
}

// Corresponds to nav2_msgs__srv__ClearEntireCostmap_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ClearEntireCostmap_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub response: std_msgs::msg::rmw::Empty,

}



impl Default for ClearEntireCostmap_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__srv__ClearEntireCostmap_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__srv__ClearEntireCostmap_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ClearEntireCostmap_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__ClearEntireCostmap_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__ClearEntireCostmap_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__ClearEntireCostmap_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ClearEntireCostmap_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ClearEntireCostmap_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/srv/ClearEntireCostmap_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__ClearEntireCostmap_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__DynamicEdges_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__srv__DynamicEdges_Request__init(msg: *mut DynamicEdges_Request) -> bool;
    fn nav2_msgs__srv__DynamicEdges_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DynamicEdges_Request>, size: usize) -> bool;
    fn nav2_msgs__srv__DynamicEdges_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DynamicEdges_Request>);
    fn nav2_msgs__srv__DynamicEdges_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DynamicEdges_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<DynamicEdges_Request>) -> bool;
}

// Corresponds to nav2_msgs__srv__DynamicEdges_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DynamicEdges_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub closed_edges: rosidl_runtime_rs::Sequence<u16>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub opened_edges: rosidl_runtime_rs::Sequence<u16>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub adjust_edges: rosidl_runtime_rs::Sequence<super::super::msg::rmw::EdgeCost>,

}



impl Default for DynamicEdges_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__srv__DynamicEdges_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__srv__DynamicEdges_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DynamicEdges_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__DynamicEdges_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__DynamicEdges_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__DynamicEdges_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DynamicEdges_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DynamicEdges_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/srv/DynamicEdges_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__DynamicEdges_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__DynamicEdges_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__srv__DynamicEdges_Response__init(msg: *mut DynamicEdges_Response) -> bool;
    fn nav2_msgs__srv__DynamicEdges_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DynamicEdges_Response>, size: usize) -> bool;
    fn nav2_msgs__srv__DynamicEdges_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DynamicEdges_Response>);
    fn nav2_msgs__srv__DynamicEdges_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DynamicEdges_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<DynamicEdges_Response>) -> bool;
}

// Corresponds to nav2_msgs__srv__DynamicEdges_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DynamicEdges_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,

}



impl Default for DynamicEdges_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__srv__DynamicEdges_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__srv__DynamicEdges_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DynamicEdges_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__DynamicEdges_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__DynamicEdges_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__DynamicEdges_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DynamicEdges_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DynamicEdges_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/srv/DynamicEdges_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__DynamicEdges_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__ManageLifecycleNodes_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__srv__ManageLifecycleNodes_Request__init(msg: *mut ManageLifecycleNodes_Request) -> bool;
    fn nav2_msgs__srv__ManageLifecycleNodes_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ManageLifecycleNodes_Request>, size: usize) -> bool;
    fn nav2_msgs__srv__ManageLifecycleNodes_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ManageLifecycleNodes_Request>);
    fn nav2_msgs__srv__ManageLifecycleNodes_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ManageLifecycleNodes_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<ManageLifecycleNodes_Request>) -> bool;
}

// Corresponds to nav2_msgs__srv__ManageLifecycleNodes_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ManageLifecycleNodes_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub command: u8,

}

impl ManageLifecycleNodes_Request {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const STARTUP: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const PAUSE: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const RESUME: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const RESET: u8 = 3;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const SHUTDOWN: u8 = 4;

}


impl Default for ManageLifecycleNodes_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__srv__ManageLifecycleNodes_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__srv__ManageLifecycleNodes_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ManageLifecycleNodes_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__ManageLifecycleNodes_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__ManageLifecycleNodes_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__ManageLifecycleNodes_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ManageLifecycleNodes_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ManageLifecycleNodes_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/srv/ManageLifecycleNodes_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__ManageLifecycleNodes_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__ManageLifecycleNodes_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__srv__ManageLifecycleNodes_Response__init(msg: *mut ManageLifecycleNodes_Response) -> bool;
    fn nav2_msgs__srv__ManageLifecycleNodes_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ManageLifecycleNodes_Response>, size: usize) -> bool;
    fn nav2_msgs__srv__ManageLifecycleNodes_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ManageLifecycleNodes_Response>);
    fn nav2_msgs__srv__ManageLifecycleNodes_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ManageLifecycleNodes_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<ManageLifecycleNodes_Response>) -> bool;
}

// Corresponds to nav2_msgs__srv__ManageLifecycleNodes_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ManageLifecycleNodes_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,

}



impl Default for ManageLifecycleNodes_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__srv__ManageLifecycleNodes_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__srv__ManageLifecycleNodes_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ManageLifecycleNodes_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__ManageLifecycleNodes_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__ManageLifecycleNodes_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__ManageLifecycleNodes_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ManageLifecycleNodes_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ManageLifecycleNodes_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/srv/ManageLifecycleNodes_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__ManageLifecycleNodes_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__LoadMap_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__srv__LoadMap_Request__init(msg: *mut LoadMap_Request) -> bool;
    fn nav2_msgs__srv__LoadMap_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<LoadMap_Request>, size: usize) -> bool;
    fn nav2_msgs__srv__LoadMap_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<LoadMap_Request>);
    fn nav2_msgs__srv__LoadMap_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<LoadMap_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<LoadMap_Request>) -> bool;
}

// Corresponds to nav2_msgs__srv__LoadMap_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct LoadMap_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub map_url: rosidl_runtime_rs::String,

}



impl Default for LoadMap_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__srv__LoadMap_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__srv__LoadMap_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for LoadMap_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__LoadMap_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__LoadMap_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__LoadMap_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for LoadMap_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for LoadMap_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/srv/LoadMap_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__LoadMap_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__LoadMap_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__srv__LoadMap_Response__init(msg: *mut LoadMap_Response) -> bool;
    fn nav2_msgs__srv__LoadMap_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<LoadMap_Response>, size: usize) -> bool;
    fn nav2_msgs__srv__LoadMap_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<LoadMap_Response>);
    fn nav2_msgs__srv__LoadMap_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<LoadMap_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<LoadMap_Response>) -> bool;
}

// Corresponds to nav2_msgs__srv__LoadMap_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct LoadMap_Response {
    /// Returned map is only valid if result equals RESULT_SUCCESS
    pub map: nav_msgs::msg::rmw::OccupancyGrid,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: u8,

}

impl LoadMap_Response {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const RESULT_SUCCESS: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const RESULT_MAP_DOES_NOT_EXIST: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const RESULT_INVALID_MAP_DATA: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const RESULT_INVALID_MAP_METADATA: u8 = 3;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const RESULT_UNDEFINED_FAILURE: u8 = 255;

}


impl Default for LoadMap_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__srv__LoadMap_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__srv__LoadMap_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for LoadMap_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__LoadMap_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__LoadMap_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__LoadMap_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for LoadMap_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for LoadMap_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/srv/LoadMap_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__LoadMap_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__SaveMap_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__srv__SaveMap_Request__init(msg: *mut SaveMap_Request) -> bool;
    fn nav2_msgs__srv__SaveMap_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SaveMap_Request>, size: usize) -> bool;
    fn nav2_msgs__srv__SaveMap_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SaveMap_Request>);
    fn nav2_msgs__srv__SaveMap_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SaveMap_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<SaveMap_Request>) -> bool;
}

// Corresponds to nav2_msgs__srv__SaveMap_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SaveMap_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub map_topic: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub map_url: rosidl_runtime_rs::String,

    /// Constants for image_format. Supported formats: pgm, png, bmp
    pub image_format: rosidl_runtime_rs::String,

    /// Map modes: trinary, scale or raw
    pub map_mode: rosidl_runtime_rs::String,

    /// Thresholds. Values in range of
    pub free_thresh: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub occupied_thresh: f32,

}



impl Default for SaveMap_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__srv__SaveMap_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__srv__SaveMap_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SaveMap_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__SaveMap_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__SaveMap_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__SaveMap_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SaveMap_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SaveMap_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/srv/SaveMap_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__SaveMap_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__SaveMap_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__srv__SaveMap_Response__init(msg: *mut SaveMap_Response) -> bool;
    fn nav2_msgs__srv__SaveMap_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SaveMap_Response>, size: usize) -> bool;
    fn nav2_msgs__srv__SaveMap_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SaveMap_Response>);
    fn nav2_msgs__srv__SaveMap_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SaveMap_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<SaveMap_Response>) -> bool;
}

// Corresponds to nav2_msgs__srv__SaveMap_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SaveMap_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub result: bool,

}



impl Default for SaveMap_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__srv__SaveMap_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__srv__SaveMap_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SaveMap_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__SaveMap_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__SaveMap_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__SaveMap_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SaveMap_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SaveMap_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/srv/SaveMap_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__SaveMap_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__SetInitialPose_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__srv__SetInitialPose_Request__init(msg: *mut SetInitialPose_Request) -> bool;
    fn nav2_msgs__srv__SetInitialPose_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetInitialPose_Request>, size: usize) -> bool;
    fn nav2_msgs__srv__SetInitialPose_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetInitialPose_Request>);
    fn nav2_msgs__srv__SetInitialPose_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetInitialPose_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<SetInitialPose_Request>) -> bool;
}

// Corresponds to nav2_msgs__srv__SetInitialPose_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetInitialPose_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub pose: geometry_msgs::msg::rmw::PoseWithCovarianceStamped,

}



impl Default for SetInitialPose_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__srv__SetInitialPose_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__srv__SetInitialPose_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetInitialPose_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__SetInitialPose_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__SetInitialPose_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__SetInitialPose_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetInitialPose_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetInitialPose_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/srv/SetInitialPose_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__SetInitialPose_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__SetInitialPose_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__srv__SetInitialPose_Response__init(msg: *mut SetInitialPose_Response) -> bool;
    fn nav2_msgs__srv__SetInitialPose_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetInitialPose_Response>, size: usize) -> bool;
    fn nav2_msgs__srv__SetInitialPose_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetInitialPose_Response>);
    fn nav2_msgs__srv__SetInitialPose_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetInitialPose_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<SetInitialPose_Response>) -> bool;
}

// Corresponds to nav2_msgs__srv__SetInitialPose_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetInitialPose_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for SetInitialPose_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__srv__SetInitialPose_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__srv__SetInitialPose_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetInitialPose_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__SetInitialPose_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__SetInitialPose_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__SetInitialPose_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetInitialPose_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetInitialPose_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/srv/SetInitialPose_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__SetInitialPose_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__SetRouteGraph_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__srv__SetRouteGraph_Request__init(msg: *mut SetRouteGraph_Request) -> bool;
    fn nav2_msgs__srv__SetRouteGraph_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetRouteGraph_Request>, size: usize) -> bool;
    fn nav2_msgs__srv__SetRouteGraph_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetRouteGraph_Request>);
    fn nav2_msgs__srv__SetRouteGraph_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetRouteGraph_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<SetRouteGraph_Request>) -> bool;
}

// Corresponds to nav2_msgs__srv__SetRouteGraph_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetRouteGraph_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub graph_filepath: rosidl_runtime_rs::String,

}



impl Default for SetRouteGraph_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__srv__SetRouteGraph_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__srv__SetRouteGraph_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetRouteGraph_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__SetRouteGraph_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__SetRouteGraph_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__SetRouteGraph_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetRouteGraph_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetRouteGraph_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/srv/SetRouteGraph_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__SetRouteGraph_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__SetRouteGraph_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__srv__SetRouteGraph_Response__init(msg: *mut SetRouteGraph_Response) -> bool;
    fn nav2_msgs__srv__SetRouteGraph_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetRouteGraph_Response>, size: usize) -> bool;
    fn nav2_msgs__srv__SetRouteGraph_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetRouteGraph_Response>);
    fn nav2_msgs__srv__SetRouteGraph_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetRouteGraph_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<SetRouteGraph_Response>) -> bool;
}

// Corresponds to nav2_msgs__srv__SetRouteGraph_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetRouteGraph_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,

}



impl Default for SetRouteGraph_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__srv__SetRouteGraph_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__srv__SetRouteGraph_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetRouteGraph_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__SetRouteGraph_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__SetRouteGraph_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__srv__SetRouteGraph_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetRouteGraph_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetRouteGraph_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/srv/SetRouteGraph_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__SetRouteGraph_Response() }
  }
}






#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__GetCostmap() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__srv__GetCostmap
#[allow(missing_docs, non_camel_case_types)]
pub struct GetCostmap;

impl rosidl_runtime_rs::Service for GetCostmap {
    type Request = GetCostmap_Request;
    type Response = GetCostmap_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__GetCostmap() }
    }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__IsPathValid() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__srv__IsPathValid
#[allow(missing_docs, non_camel_case_types)]
pub struct IsPathValid;

impl rosidl_runtime_rs::Service for IsPathValid {
    type Request = IsPathValid_Request;
    type Response = IsPathValid_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__IsPathValid() }
    }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__ClearCostmapExceptRegion() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__srv__ClearCostmapExceptRegion
#[allow(missing_docs, non_camel_case_types)]
pub struct ClearCostmapExceptRegion;

impl rosidl_runtime_rs::Service for ClearCostmapExceptRegion {
    type Request = ClearCostmapExceptRegion_Request;
    type Response = ClearCostmapExceptRegion_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__ClearCostmapExceptRegion() }
    }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__ClearCostmapAroundRobot() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__srv__ClearCostmapAroundRobot
#[allow(missing_docs, non_camel_case_types)]
pub struct ClearCostmapAroundRobot;

impl rosidl_runtime_rs::Service for ClearCostmapAroundRobot {
    type Request = ClearCostmapAroundRobot_Request;
    type Response = ClearCostmapAroundRobot_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__ClearCostmapAroundRobot() }
    }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__ClearEntireCostmap() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__srv__ClearEntireCostmap
#[allow(missing_docs, non_camel_case_types)]
pub struct ClearEntireCostmap;

impl rosidl_runtime_rs::Service for ClearEntireCostmap {
    type Request = ClearEntireCostmap_Request;
    type Response = ClearEntireCostmap_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__ClearEntireCostmap() }
    }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__DynamicEdges() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__srv__DynamicEdges
#[allow(missing_docs, non_camel_case_types)]
pub struct DynamicEdges;

impl rosidl_runtime_rs::Service for DynamicEdges {
    type Request = DynamicEdges_Request;
    type Response = DynamicEdges_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__DynamicEdges() }
    }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__ManageLifecycleNodes() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__srv__ManageLifecycleNodes
#[allow(missing_docs, non_camel_case_types)]
pub struct ManageLifecycleNodes;

impl rosidl_runtime_rs::Service for ManageLifecycleNodes {
    type Request = ManageLifecycleNodes_Request;
    type Response = ManageLifecycleNodes_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__ManageLifecycleNodes() }
    }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__LoadMap() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__srv__LoadMap
#[allow(missing_docs, non_camel_case_types)]
pub struct LoadMap;

impl rosidl_runtime_rs::Service for LoadMap {
    type Request = LoadMap_Request;
    type Response = LoadMap_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__LoadMap() }
    }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__SaveMap() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__srv__SaveMap
#[allow(missing_docs, non_camel_case_types)]
pub struct SaveMap;

impl rosidl_runtime_rs::Service for SaveMap {
    type Request = SaveMap_Request;
    type Response = SaveMap_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__SaveMap() }
    }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__SetInitialPose() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__srv__SetInitialPose
#[allow(missing_docs, non_camel_case_types)]
pub struct SetInitialPose;

impl rosidl_runtime_rs::Service for SetInitialPose {
    type Request = SetInitialPose_Request;
    type Response = SetInitialPose_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__SetInitialPose() }
    }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__SetRouteGraph() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__srv__SetRouteGraph
#[allow(missing_docs, non_camel_case_types)]
pub struct SetRouteGraph;

impl rosidl_runtime_rs::Service for SetRouteGraph {
    type Request = SetRouteGraph_Request;
    type Response = SetRouteGraph_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__SetRouteGraph() }
    }
}


