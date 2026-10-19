
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__AssistedTeleop_Goal() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__AssistedTeleop_Goal__init(msg: *mut AssistedTeleop_Goal) -> bool;
    fn nav2_msgs__action__AssistedTeleop_Goal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<AssistedTeleop_Goal>, size: usize) -> bool;
    fn nav2_msgs__action__AssistedTeleop_Goal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<AssistedTeleop_Goal>);
    fn nav2_msgs__action__AssistedTeleop_Goal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<AssistedTeleop_Goal>, out_seq: *mut rosidl_runtime_rs::Sequence<AssistedTeleop_Goal>) -> bool;
}

// Corresponds to nav2_msgs__action__AssistedTeleop_Goal
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AssistedTeleop_Goal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub time_allowance: builtin_interfaces::msg::rmw::Duration,

}



impl Default for AssistedTeleop_Goal {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__AssistedTeleop_Goal__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__AssistedTeleop_Goal__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for AssistedTeleop_Goal {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__AssistedTeleop_Goal__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__AssistedTeleop_Goal__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__AssistedTeleop_Goal__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for AssistedTeleop_Goal {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for AssistedTeleop_Goal where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/AssistedTeleop_Goal";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__AssistedTeleop_Goal() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__AssistedTeleop_Result() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__AssistedTeleop_Result__init(msg: *mut AssistedTeleop_Result) -> bool;
    fn nav2_msgs__action__AssistedTeleop_Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<AssistedTeleop_Result>, size: usize) -> bool;
    fn nav2_msgs__action__AssistedTeleop_Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<AssistedTeleop_Result>);
    fn nav2_msgs__action__AssistedTeleop_Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<AssistedTeleop_Result>, out_seq: *mut rosidl_runtime_rs::Sequence<AssistedTeleop_Result>) -> bool;
}

// Corresponds to nav2_msgs__action__AssistedTeleop_Result
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AssistedTeleop_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub total_elapsed_time: builtin_interfaces::msg::rmw::Duration,

}



impl Default for AssistedTeleop_Result {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__AssistedTeleop_Result__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__AssistedTeleop_Result__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for AssistedTeleop_Result {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__AssistedTeleop_Result__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__AssistedTeleop_Result__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__AssistedTeleop_Result__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for AssistedTeleop_Result {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for AssistedTeleop_Result where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/AssistedTeleop_Result";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__AssistedTeleop_Result() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__AssistedTeleop_Feedback() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__AssistedTeleop_Feedback__init(msg: *mut AssistedTeleop_Feedback) -> bool;
    fn nav2_msgs__action__AssistedTeleop_Feedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<AssistedTeleop_Feedback>, size: usize) -> bool;
    fn nav2_msgs__action__AssistedTeleop_Feedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<AssistedTeleop_Feedback>);
    fn nav2_msgs__action__AssistedTeleop_Feedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<AssistedTeleop_Feedback>, out_seq: *mut rosidl_runtime_rs::Sequence<AssistedTeleop_Feedback>) -> bool;
}

// Corresponds to nav2_msgs__action__AssistedTeleop_Feedback
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AssistedTeleop_Feedback {

    // This member is not documented.
    #[allow(missing_docs)]
    pub current_teleop_duration: builtin_interfaces::msg::rmw::Duration,

}



impl Default for AssistedTeleop_Feedback {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__AssistedTeleop_Feedback__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__AssistedTeleop_Feedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for AssistedTeleop_Feedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__AssistedTeleop_Feedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__AssistedTeleop_Feedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__AssistedTeleop_Feedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for AssistedTeleop_Feedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for AssistedTeleop_Feedback where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/AssistedTeleop_Feedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__AssistedTeleop_Feedback() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__AssistedTeleop_FeedbackMessage() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__AssistedTeleop_FeedbackMessage__init(msg: *mut AssistedTeleop_FeedbackMessage) -> bool;
    fn nav2_msgs__action__AssistedTeleop_FeedbackMessage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<AssistedTeleop_FeedbackMessage>, size: usize) -> bool;
    fn nav2_msgs__action__AssistedTeleop_FeedbackMessage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<AssistedTeleop_FeedbackMessage>);
    fn nav2_msgs__action__AssistedTeleop_FeedbackMessage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<AssistedTeleop_FeedbackMessage>, out_seq: *mut rosidl_runtime_rs::Sequence<AssistedTeleop_FeedbackMessage>) -> bool;
}

// Corresponds to nav2_msgs__action__AssistedTeleop_FeedbackMessage
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AssistedTeleop_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::super::action::rmw::AssistedTeleop_Feedback,

}



impl Default for AssistedTeleop_FeedbackMessage {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__AssistedTeleop_FeedbackMessage__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__AssistedTeleop_FeedbackMessage__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for AssistedTeleop_FeedbackMessage {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__AssistedTeleop_FeedbackMessage__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__AssistedTeleop_FeedbackMessage__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__AssistedTeleop_FeedbackMessage__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for AssistedTeleop_FeedbackMessage {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for AssistedTeleop_FeedbackMessage where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/AssistedTeleop_FeedbackMessage";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__AssistedTeleop_FeedbackMessage() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__BackUp_Goal() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__BackUp_Goal__init(msg: *mut BackUp_Goal) -> bool;
    fn nav2_msgs__action__BackUp_Goal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<BackUp_Goal>, size: usize) -> bool;
    fn nav2_msgs__action__BackUp_Goal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<BackUp_Goal>);
    fn nav2_msgs__action__BackUp_Goal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<BackUp_Goal>, out_seq: *mut rosidl_runtime_rs::Sequence<BackUp_Goal>) -> bool;
}

// Corresponds to nav2_msgs__action__BackUp_Goal
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BackUp_Goal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub target: geometry_msgs::msg::rmw::Point,


    // This member is not documented.
    #[allow(missing_docs)]
    pub speed: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub time_allowance: builtin_interfaces::msg::rmw::Duration,

}



impl Default for BackUp_Goal {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__BackUp_Goal__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__BackUp_Goal__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for BackUp_Goal {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__BackUp_Goal__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__BackUp_Goal__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__BackUp_Goal__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for BackUp_Goal {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for BackUp_Goal where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/BackUp_Goal";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__BackUp_Goal() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__BackUp_Result() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__BackUp_Result__init(msg: *mut BackUp_Result) -> bool;
    fn nav2_msgs__action__BackUp_Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<BackUp_Result>, size: usize) -> bool;
    fn nav2_msgs__action__BackUp_Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<BackUp_Result>);
    fn nav2_msgs__action__BackUp_Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<BackUp_Result>, out_seq: *mut rosidl_runtime_rs::Sequence<BackUp_Result>) -> bool;
}

// Corresponds to nav2_msgs__action__BackUp_Result
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BackUp_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub total_elapsed_time: builtin_interfaces::msg::rmw::Duration,

}



impl Default for BackUp_Result {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__BackUp_Result__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__BackUp_Result__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for BackUp_Result {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__BackUp_Result__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__BackUp_Result__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__BackUp_Result__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for BackUp_Result {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for BackUp_Result where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/BackUp_Result";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__BackUp_Result() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__BackUp_Feedback() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__BackUp_Feedback__init(msg: *mut BackUp_Feedback) -> bool;
    fn nav2_msgs__action__BackUp_Feedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<BackUp_Feedback>, size: usize) -> bool;
    fn nav2_msgs__action__BackUp_Feedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<BackUp_Feedback>);
    fn nav2_msgs__action__BackUp_Feedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<BackUp_Feedback>, out_seq: *mut rosidl_runtime_rs::Sequence<BackUp_Feedback>) -> bool;
}

// Corresponds to nav2_msgs__action__BackUp_Feedback
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BackUp_Feedback {

    // This member is not documented.
    #[allow(missing_docs)]
    pub distance_traveled: f32,

}



impl Default for BackUp_Feedback {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__BackUp_Feedback__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__BackUp_Feedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for BackUp_Feedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__BackUp_Feedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__BackUp_Feedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__BackUp_Feedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for BackUp_Feedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for BackUp_Feedback where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/BackUp_Feedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__BackUp_Feedback() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__BackUp_FeedbackMessage() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__BackUp_FeedbackMessage__init(msg: *mut BackUp_FeedbackMessage) -> bool;
    fn nav2_msgs__action__BackUp_FeedbackMessage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<BackUp_FeedbackMessage>, size: usize) -> bool;
    fn nav2_msgs__action__BackUp_FeedbackMessage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<BackUp_FeedbackMessage>);
    fn nav2_msgs__action__BackUp_FeedbackMessage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<BackUp_FeedbackMessage>, out_seq: *mut rosidl_runtime_rs::Sequence<BackUp_FeedbackMessage>) -> bool;
}

// Corresponds to nav2_msgs__action__BackUp_FeedbackMessage
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BackUp_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::super::action::rmw::BackUp_Feedback,

}



impl Default for BackUp_FeedbackMessage {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__BackUp_FeedbackMessage__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__BackUp_FeedbackMessage__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for BackUp_FeedbackMessage {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__BackUp_FeedbackMessage__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__BackUp_FeedbackMessage__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__BackUp_FeedbackMessage__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for BackUp_FeedbackMessage {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for BackUp_FeedbackMessage where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/BackUp_FeedbackMessage";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__BackUp_FeedbackMessage() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathToPose_Goal() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__ComputePathToPose_Goal__init(msg: *mut ComputePathToPose_Goal) -> bool;
    fn nav2_msgs__action__ComputePathToPose_Goal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ComputePathToPose_Goal>, size: usize) -> bool;
    fn nav2_msgs__action__ComputePathToPose_Goal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ComputePathToPose_Goal>);
    fn nav2_msgs__action__ComputePathToPose_Goal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ComputePathToPose_Goal>, out_seq: *mut rosidl_runtime_rs::Sequence<ComputePathToPose_Goal>) -> bool;
}

// Corresponds to nav2_msgs__action__ComputePathToPose_Goal
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathToPose_Goal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: geometry_msgs::msg::rmw::PoseStamped,


    // This member is not documented.
    #[allow(missing_docs)]
    pub start: geometry_msgs::msg::rmw::PoseStamped,


    // This member is not documented.
    #[allow(missing_docs)]
    pub planner_id: rosidl_runtime_rs::String,

    /// If false, use current robot pose as path start, if true, use start above instead
    pub use_start: bool,

}



impl Default for ComputePathToPose_Goal {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__ComputePathToPose_Goal__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__ComputePathToPose_Goal__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ComputePathToPose_Goal {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathToPose_Goal__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathToPose_Goal__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathToPose_Goal__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ComputePathToPose_Goal {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ComputePathToPose_Goal where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/ComputePathToPose_Goal";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathToPose_Goal() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathToPose_Result() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__ComputePathToPose_Result__init(msg: *mut ComputePathToPose_Result) -> bool;
    fn nav2_msgs__action__ComputePathToPose_Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ComputePathToPose_Result>, size: usize) -> bool;
    fn nav2_msgs__action__ComputePathToPose_Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ComputePathToPose_Result>);
    fn nav2_msgs__action__ComputePathToPose_Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ComputePathToPose_Result>, out_seq: *mut rosidl_runtime_rs::Sequence<ComputePathToPose_Result>) -> bool;
}

// Corresponds to nav2_msgs__action__ComputePathToPose_Result
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathToPose_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub path: nav_msgs::msg::rmw::Path,


    // This member is not documented.
    #[allow(missing_docs)]
    pub planning_time: builtin_interfaces::msg::rmw::Duration,

}



impl Default for ComputePathToPose_Result {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__ComputePathToPose_Result__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__ComputePathToPose_Result__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ComputePathToPose_Result {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathToPose_Result__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathToPose_Result__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathToPose_Result__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ComputePathToPose_Result {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ComputePathToPose_Result where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/ComputePathToPose_Result";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathToPose_Result() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathToPose_Feedback() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__ComputePathToPose_Feedback__init(msg: *mut ComputePathToPose_Feedback) -> bool;
    fn nav2_msgs__action__ComputePathToPose_Feedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ComputePathToPose_Feedback>, size: usize) -> bool;
    fn nav2_msgs__action__ComputePathToPose_Feedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ComputePathToPose_Feedback>);
    fn nav2_msgs__action__ComputePathToPose_Feedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ComputePathToPose_Feedback>, out_seq: *mut rosidl_runtime_rs::Sequence<ComputePathToPose_Feedback>) -> bool;
}

// Corresponds to nav2_msgs__action__ComputePathToPose_Feedback
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathToPose_Feedback {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for ComputePathToPose_Feedback {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__ComputePathToPose_Feedback__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__ComputePathToPose_Feedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ComputePathToPose_Feedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathToPose_Feedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathToPose_Feedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathToPose_Feedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ComputePathToPose_Feedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ComputePathToPose_Feedback where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/ComputePathToPose_Feedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathToPose_Feedback() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathToPose_FeedbackMessage() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__ComputePathToPose_FeedbackMessage__init(msg: *mut ComputePathToPose_FeedbackMessage) -> bool;
    fn nav2_msgs__action__ComputePathToPose_FeedbackMessage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ComputePathToPose_FeedbackMessage>, size: usize) -> bool;
    fn nav2_msgs__action__ComputePathToPose_FeedbackMessage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ComputePathToPose_FeedbackMessage>);
    fn nav2_msgs__action__ComputePathToPose_FeedbackMessage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ComputePathToPose_FeedbackMessage>, out_seq: *mut rosidl_runtime_rs::Sequence<ComputePathToPose_FeedbackMessage>) -> bool;
}

// Corresponds to nav2_msgs__action__ComputePathToPose_FeedbackMessage
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathToPose_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::super::action::rmw::ComputePathToPose_Feedback,

}



impl Default for ComputePathToPose_FeedbackMessage {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__ComputePathToPose_FeedbackMessage__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__ComputePathToPose_FeedbackMessage__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ComputePathToPose_FeedbackMessage {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathToPose_FeedbackMessage__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathToPose_FeedbackMessage__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathToPose_FeedbackMessage__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ComputePathToPose_FeedbackMessage {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ComputePathToPose_FeedbackMessage where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/ComputePathToPose_FeedbackMessage";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathToPose_FeedbackMessage() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathThroughPoses_Goal() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__ComputePathThroughPoses_Goal__init(msg: *mut ComputePathThroughPoses_Goal) -> bool;
    fn nav2_msgs__action__ComputePathThroughPoses_Goal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ComputePathThroughPoses_Goal>, size: usize) -> bool;
    fn nav2_msgs__action__ComputePathThroughPoses_Goal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ComputePathThroughPoses_Goal>);
    fn nav2_msgs__action__ComputePathThroughPoses_Goal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ComputePathThroughPoses_Goal>, out_seq: *mut rosidl_runtime_rs::Sequence<ComputePathThroughPoses_Goal>) -> bool;
}

// Corresponds to nav2_msgs__action__ComputePathThroughPoses_Goal
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathThroughPoses_Goal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goals: rosidl_runtime_rs::Sequence<geometry_msgs::msg::rmw::PoseStamped>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub start: geometry_msgs::msg::rmw::PoseStamped,


    // This member is not documented.
    #[allow(missing_docs)]
    pub planner_id: rosidl_runtime_rs::String,

    /// If false, use current robot pose as path start, if true, use start above instead
    pub use_start: bool,

}



impl Default for ComputePathThroughPoses_Goal {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__ComputePathThroughPoses_Goal__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__ComputePathThroughPoses_Goal__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ComputePathThroughPoses_Goal {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathThroughPoses_Goal__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathThroughPoses_Goal__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathThroughPoses_Goal__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ComputePathThroughPoses_Goal {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ComputePathThroughPoses_Goal where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/ComputePathThroughPoses_Goal";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathThroughPoses_Goal() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathThroughPoses_Result() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__ComputePathThroughPoses_Result__init(msg: *mut ComputePathThroughPoses_Result) -> bool;
    fn nav2_msgs__action__ComputePathThroughPoses_Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ComputePathThroughPoses_Result>, size: usize) -> bool;
    fn nav2_msgs__action__ComputePathThroughPoses_Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ComputePathThroughPoses_Result>);
    fn nav2_msgs__action__ComputePathThroughPoses_Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ComputePathThroughPoses_Result>, out_seq: *mut rosidl_runtime_rs::Sequence<ComputePathThroughPoses_Result>) -> bool;
}

// Corresponds to nav2_msgs__action__ComputePathThroughPoses_Result
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathThroughPoses_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub path: nav_msgs::msg::rmw::Path,


    // This member is not documented.
    #[allow(missing_docs)]
    pub planning_time: builtin_interfaces::msg::rmw::Duration,

}



impl Default for ComputePathThroughPoses_Result {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__ComputePathThroughPoses_Result__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__ComputePathThroughPoses_Result__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ComputePathThroughPoses_Result {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathThroughPoses_Result__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathThroughPoses_Result__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathThroughPoses_Result__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ComputePathThroughPoses_Result {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ComputePathThroughPoses_Result where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/ComputePathThroughPoses_Result";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathThroughPoses_Result() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathThroughPoses_Feedback() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__ComputePathThroughPoses_Feedback__init(msg: *mut ComputePathThroughPoses_Feedback) -> bool;
    fn nav2_msgs__action__ComputePathThroughPoses_Feedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ComputePathThroughPoses_Feedback>, size: usize) -> bool;
    fn nav2_msgs__action__ComputePathThroughPoses_Feedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ComputePathThroughPoses_Feedback>);
    fn nav2_msgs__action__ComputePathThroughPoses_Feedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ComputePathThroughPoses_Feedback>, out_seq: *mut rosidl_runtime_rs::Sequence<ComputePathThroughPoses_Feedback>) -> bool;
}

// Corresponds to nav2_msgs__action__ComputePathThroughPoses_Feedback
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathThroughPoses_Feedback {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for ComputePathThroughPoses_Feedback {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__ComputePathThroughPoses_Feedback__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__ComputePathThroughPoses_Feedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ComputePathThroughPoses_Feedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathThroughPoses_Feedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathThroughPoses_Feedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathThroughPoses_Feedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ComputePathThroughPoses_Feedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ComputePathThroughPoses_Feedback where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/ComputePathThroughPoses_Feedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathThroughPoses_Feedback() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathThroughPoses_FeedbackMessage() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__ComputePathThroughPoses_FeedbackMessage__init(msg: *mut ComputePathThroughPoses_FeedbackMessage) -> bool;
    fn nav2_msgs__action__ComputePathThroughPoses_FeedbackMessage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ComputePathThroughPoses_FeedbackMessage>, size: usize) -> bool;
    fn nav2_msgs__action__ComputePathThroughPoses_FeedbackMessage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ComputePathThroughPoses_FeedbackMessage>);
    fn nav2_msgs__action__ComputePathThroughPoses_FeedbackMessage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ComputePathThroughPoses_FeedbackMessage>, out_seq: *mut rosidl_runtime_rs::Sequence<ComputePathThroughPoses_FeedbackMessage>) -> bool;
}

// Corresponds to nav2_msgs__action__ComputePathThroughPoses_FeedbackMessage
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathThroughPoses_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::super::action::rmw::ComputePathThroughPoses_Feedback,

}



impl Default for ComputePathThroughPoses_FeedbackMessage {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__ComputePathThroughPoses_FeedbackMessage__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__ComputePathThroughPoses_FeedbackMessage__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ComputePathThroughPoses_FeedbackMessage {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathThroughPoses_FeedbackMessage__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathThroughPoses_FeedbackMessage__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathThroughPoses_FeedbackMessage__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ComputePathThroughPoses_FeedbackMessage {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ComputePathThroughPoses_FeedbackMessage where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/ComputePathThroughPoses_FeedbackMessage";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathThroughPoses_FeedbackMessage() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputeRoute_Goal() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__ComputeRoute_Goal__init(msg: *mut ComputeRoute_Goal) -> bool;
    fn nav2_msgs__action__ComputeRoute_Goal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ComputeRoute_Goal>, size: usize) -> bool;
    fn nav2_msgs__action__ComputeRoute_Goal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ComputeRoute_Goal>);
    fn nav2_msgs__action__ComputeRoute_Goal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ComputeRoute_Goal>, out_seq: *mut rosidl_runtime_rs::Sequence<ComputeRoute_Goal>) -> bool;
}

// Corresponds to nav2_msgs__action__ComputeRoute_Goal
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputeRoute_Goal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub start_id: u16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub start: geometry_msgs::msg::rmw::PoseStamped,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: u16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: geometry_msgs::msg::rmw::PoseStamped,

    /// Whether to use the start field or find the start pose in TF
    pub use_start: bool,

    /// Whether to use the poses or the IDs fields for request
    pub use_poses: bool,

}



impl Default for ComputeRoute_Goal {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__ComputeRoute_Goal__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__ComputeRoute_Goal__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ComputeRoute_Goal {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputeRoute_Goal__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputeRoute_Goal__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputeRoute_Goal__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ComputeRoute_Goal {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ComputeRoute_Goal where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/ComputeRoute_Goal";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputeRoute_Goal() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputeRoute_Result() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__ComputeRoute_Result__init(msg: *mut ComputeRoute_Result) -> bool;
    fn nav2_msgs__action__ComputeRoute_Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ComputeRoute_Result>, size: usize) -> bool;
    fn nav2_msgs__action__ComputeRoute_Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ComputeRoute_Result>);
    fn nav2_msgs__action__ComputeRoute_Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ComputeRoute_Result>, out_seq: *mut rosidl_runtime_rs::Sequence<ComputeRoute_Result>) -> bool;
}

// Corresponds to nav2_msgs__action__ComputeRoute_Result
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputeRoute_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub planning_time: builtin_interfaces::msg::rmw::Duration,


    // This member is not documented.
    #[allow(missing_docs)]
    pub path: nav_msgs::msg::rmw::Path,


    // This member is not documented.
    #[allow(missing_docs)]
    pub route: super::super::msg::rmw::Route,

}

impl ComputeRoute_Result {
    /// Error codes
    pub const NONE: u16 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const UNKNOWN: u16 = 400;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const TF_ERROR: u16 = 401;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const NO_VALID_GRAPH: u16 = 402;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const INDETERMINANT_NODES_ON_GRAPH: u16 = 403;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const TIMEOUT: u16 = 404;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const NO_VALID_ROUTE: u16 = 405;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const INVALID_EDGE_SCORER_USE: u16 = 407;

}


impl Default for ComputeRoute_Result {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__ComputeRoute_Result__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__ComputeRoute_Result__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ComputeRoute_Result {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputeRoute_Result__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputeRoute_Result__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputeRoute_Result__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ComputeRoute_Result {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ComputeRoute_Result where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/ComputeRoute_Result";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputeRoute_Result() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputeRoute_Feedback() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__ComputeRoute_Feedback__init(msg: *mut ComputeRoute_Feedback) -> bool;
    fn nav2_msgs__action__ComputeRoute_Feedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ComputeRoute_Feedback>, size: usize) -> bool;
    fn nav2_msgs__action__ComputeRoute_Feedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ComputeRoute_Feedback>);
    fn nav2_msgs__action__ComputeRoute_Feedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ComputeRoute_Feedback>, out_seq: *mut rosidl_runtime_rs::Sequence<ComputeRoute_Feedback>) -> bool;
}

// Corresponds to nav2_msgs__action__ComputeRoute_Feedback
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputeRoute_Feedback {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for ComputeRoute_Feedback {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__ComputeRoute_Feedback__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__ComputeRoute_Feedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ComputeRoute_Feedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputeRoute_Feedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputeRoute_Feedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputeRoute_Feedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ComputeRoute_Feedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ComputeRoute_Feedback where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/ComputeRoute_Feedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputeRoute_Feedback() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputeRoute_FeedbackMessage() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__ComputeRoute_FeedbackMessage__init(msg: *mut ComputeRoute_FeedbackMessage) -> bool;
    fn nav2_msgs__action__ComputeRoute_FeedbackMessage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ComputeRoute_FeedbackMessage>, size: usize) -> bool;
    fn nav2_msgs__action__ComputeRoute_FeedbackMessage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ComputeRoute_FeedbackMessage>);
    fn nav2_msgs__action__ComputeRoute_FeedbackMessage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ComputeRoute_FeedbackMessage>, out_seq: *mut rosidl_runtime_rs::Sequence<ComputeRoute_FeedbackMessage>) -> bool;
}

// Corresponds to nav2_msgs__action__ComputeRoute_FeedbackMessage
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputeRoute_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::super::action::rmw::ComputeRoute_Feedback,

}



impl Default for ComputeRoute_FeedbackMessage {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__ComputeRoute_FeedbackMessage__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__ComputeRoute_FeedbackMessage__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ComputeRoute_FeedbackMessage {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputeRoute_FeedbackMessage__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputeRoute_FeedbackMessage__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputeRoute_FeedbackMessage__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ComputeRoute_FeedbackMessage {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ComputeRoute_FeedbackMessage where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/ComputeRoute_FeedbackMessage";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputeRoute_FeedbackMessage() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputeAndTrackRoute_Goal() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__ComputeAndTrackRoute_Goal__init(msg: *mut ComputeAndTrackRoute_Goal) -> bool;
    fn nav2_msgs__action__ComputeAndTrackRoute_Goal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ComputeAndTrackRoute_Goal>, size: usize) -> bool;
    fn nav2_msgs__action__ComputeAndTrackRoute_Goal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ComputeAndTrackRoute_Goal>);
    fn nav2_msgs__action__ComputeAndTrackRoute_Goal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ComputeAndTrackRoute_Goal>, out_seq: *mut rosidl_runtime_rs::Sequence<ComputeAndTrackRoute_Goal>) -> bool;
}

// Corresponds to nav2_msgs__action__ComputeAndTrackRoute_Goal
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputeAndTrackRoute_Goal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub start_id: u16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub start: geometry_msgs::msg::rmw::PoseStamped,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: u16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: geometry_msgs::msg::rmw::PoseStamped,

    /// Whether to use the start field or find the start pose in TF
    pub use_start: bool,

    /// Whether to use the poses or the IDs fields for request
    pub use_poses: bool,

}



impl Default for ComputeAndTrackRoute_Goal {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__ComputeAndTrackRoute_Goal__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__ComputeAndTrackRoute_Goal__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ComputeAndTrackRoute_Goal {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputeAndTrackRoute_Goal__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputeAndTrackRoute_Goal__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputeAndTrackRoute_Goal__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ComputeAndTrackRoute_Goal {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ComputeAndTrackRoute_Goal where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/ComputeAndTrackRoute_Goal";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputeAndTrackRoute_Goal() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputeAndTrackRoute_Result() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__ComputeAndTrackRoute_Result__init(msg: *mut ComputeAndTrackRoute_Result) -> bool;
    fn nav2_msgs__action__ComputeAndTrackRoute_Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ComputeAndTrackRoute_Result>, size: usize) -> bool;
    fn nav2_msgs__action__ComputeAndTrackRoute_Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ComputeAndTrackRoute_Result>);
    fn nav2_msgs__action__ComputeAndTrackRoute_Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ComputeAndTrackRoute_Result>, out_seq: *mut rosidl_runtime_rs::Sequence<ComputeAndTrackRoute_Result>) -> bool;
}

// Corresponds to nav2_msgs__action__ComputeAndTrackRoute_Result
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputeAndTrackRoute_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub execution_duration: builtin_interfaces::msg::rmw::Duration,

}

impl ComputeAndTrackRoute_Result {
    /// Error codes
    pub const NONE: u16 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const UNKNOWN: u16 = 400;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const TF_ERROR: u16 = 401;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const NO_VALID_GRAPH: u16 = 402;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const INDETERMINANT_NODES_ON_GRAPH: u16 = 403;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const TIMEOUT: u16 = 404;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const NO_VALID_ROUTE: u16 = 405;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const OPERATION_FAILED: u16 = 406;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const INVALID_EDGE_SCORER_USE: u16 = 407;

}


impl Default for ComputeAndTrackRoute_Result {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__ComputeAndTrackRoute_Result__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__ComputeAndTrackRoute_Result__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ComputeAndTrackRoute_Result {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputeAndTrackRoute_Result__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputeAndTrackRoute_Result__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputeAndTrackRoute_Result__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ComputeAndTrackRoute_Result {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ComputeAndTrackRoute_Result where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/ComputeAndTrackRoute_Result";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputeAndTrackRoute_Result() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputeAndTrackRoute_Feedback() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__ComputeAndTrackRoute_Feedback__init(msg: *mut ComputeAndTrackRoute_Feedback) -> bool;
    fn nav2_msgs__action__ComputeAndTrackRoute_Feedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ComputeAndTrackRoute_Feedback>, size: usize) -> bool;
    fn nav2_msgs__action__ComputeAndTrackRoute_Feedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ComputeAndTrackRoute_Feedback>);
    fn nav2_msgs__action__ComputeAndTrackRoute_Feedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ComputeAndTrackRoute_Feedback>, out_seq: *mut rosidl_runtime_rs::Sequence<ComputeAndTrackRoute_Feedback>) -> bool;
}

// Corresponds to nav2_msgs__action__ComputeAndTrackRoute_Feedback
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputeAndTrackRoute_Feedback {

    // This member is not documented.
    #[allow(missing_docs)]
    pub last_node_id: u16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub next_node_id: u16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub current_edge_id: u16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub route: super::super::msg::rmw::Route,


    // This member is not documented.
    #[allow(missing_docs)]
    pub path: nav_msgs::msg::rmw::Path,


    // This member is not documented.
    #[allow(missing_docs)]
    pub operations_triggered: rosidl_runtime_rs::Sequence<rosidl_runtime_rs::String>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub rerouted: bool,

}



impl Default for ComputeAndTrackRoute_Feedback {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__ComputeAndTrackRoute_Feedback__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__ComputeAndTrackRoute_Feedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ComputeAndTrackRoute_Feedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputeAndTrackRoute_Feedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputeAndTrackRoute_Feedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputeAndTrackRoute_Feedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ComputeAndTrackRoute_Feedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ComputeAndTrackRoute_Feedback where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/ComputeAndTrackRoute_Feedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputeAndTrackRoute_Feedback() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputeAndTrackRoute_FeedbackMessage() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__ComputeAndTrackRoute_FeedbackMessage__init(msg: *mut ComputeAndTrackRoute_FeedbackMessage) -> bool;
    fn nav2_msgs__action__ComputeAndTrackRoute_FeedbackMessage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ComputeAndTrackRoute_FeedbackMessage>, size: usize) -> bool;
    fn nav2_msgs__action__ComputeAndTrackRoute_FeedbackMessage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ComputeAndTrackRoute_FeedbackMessage>);
    fn nav2_msgs__action__ComputeAndTrackRoute_FeedbackMessage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ComputeAndTrackRoute_FeedbackMessage>, out_seq: *mut rosidl_runtime_rs::Sequence<ComputeAndTrackRoute_FeedbackMessage>) -> bool;
}

// Corresponds to nav2_msgs__action__ComputeAndTrackRoute_FeedbackMessage
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputeAndTrackRoute_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::super::action::rmw::ComputeAndTrackRoute_Feedback,

}



impl Default for ComputeAndTrackRoute_FeedbackMessage {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__ComputeAndTrackRoute_FeedbackMessage__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__ComputeAndTrackRoute_FeedbackMessage__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ComputeAndTrackRoute_FeedbackMessage {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputeAndTrackRoute_FeedbackMessage__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputeAndTrackRoute_FeedbackMessage__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputeAndTrackRoute_FeedbackMessage__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ComputeAndTrackRoute_FeedbackMessage {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ComputeAndTrackRoute_FeedbackMessage where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/ComputeAndTrackRoute_FeedbackMessage";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputeAndTrackRoute_FeedbackMessage() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DriveOnHeading_Goal() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__DriveOnHeading_Goal__init(msg: *mut DriveOnHeading_Goal) -> bool;
    fn nav2_msgs__action__DriveOnHeading_Goal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DriveOnHeading_Goal>, size: usize) -> bool;
    fn nav2_msgs__action__DriveOnHeading_Goal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DriveOnHeading_Goal>);
    fn nav2_msgs__action__DriveOnHeading_Goal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DriveOnHeading_Goal>, out_seq: *mut rosidl_runtime_rs::Sequence<DriveOnHeading_Goal>) -> bool;
}

// Corresponds to nav2_msgs__action__DriveOnHeading_Goal
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DriveOnHeading_Goal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub target: geometry_msgs::msg::rmw::Point,


    // This member is not documented.
    #[allow(missing_docs)]
    pub speed: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub time_allowance: builtin_interfaces::msg::rmw::Duration,

}



impl Default for DriveOnHeading_Goal {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__DriveOnHeading_Goal__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__DriveOnHeading_Goal__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DriveOnHeading_Goal {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DriveOnHeading_Goal__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DriveOnHeading_Goal__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DriveOnHeading_Goal__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DriveOnHeading_Goal {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DriveOnHeading_Goal where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/DriveOnHeading_Goal";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DriveOnHeading_Goal() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DriveOnHeading_Result() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__DriveOnHeading_Result__init(msg: *mut DriveOnHeading_Result) -> bool;
    fn nav2_msgs__action__DriveOnHeading_Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DriveOnHeading_Result>, size: usize) -> bool;
    fn nav2_msgs__action__DriveOnHeading_Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DriveOnHeading_Result>);
    fn nav2_msgs__action__DriveOnHeading_Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DriveOnHeading_Result>, out_seq: *mut rosidl_runtime_rs::Sequence<DriveOnHeading_Result>) -> bool;
}

// Corresponds to nav2_msgs__action__DriveOnHeading_Result
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DriveOnHeading_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub total_elapsed_time: builtin_interfaces::msg::rmw::Duration,

}



impl Default for DriveOnHeading_Result {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__DriveOnHeading_Result__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__DriveOnHeading_Result__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DriveOnHeading_Result {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DriveOnHeading_Result__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DriveOnHeading_Result__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DriveOnHeading_Result__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DriveOnHeading_Result {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DriveOnHeading_Result where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/DriveOnHeading_Result";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DriveOnHeading_Result() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DriveOnHeading_Feedback() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__DriveOnHeading_Feedback__init(msg: *mut DriveOnHeading_Feedback) -> bool;
    fn nav2_msgs__action__DriveOnHeading_Feedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DriveOnHeading_Feedback>, size: usize) -> bool;
    fn nav2_msgs__action__DriveOnHeading_Feedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DriveOnHeading_Feedback>);
    fn nav2_msgs__action__DriveOnHeading_Feedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DriveOnHeading_Feedback>, out_seq: *mut rosidl_runtime_rs::Sequence<DriveOnHeading_Feedback>) -> bool;
}

// Corresponds to nav2_msgs__action__DriveOnHeading_Feedback
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DriveOnHeading_Feedback {

    // This member is not documented.
    #[allow(missing_docs)]
    pub distance_traveled: f32,

}



impl Default for DriveOnHeading_Feedback {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__DriveOnHeading_Feedback__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__DriveOnHeading_Feedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DriveOnHeading_Feedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DriveOnHeading_Feedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DriveOnHeading_Feedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DriveOnHeading_Feedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DriveOnHeading_Feedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DriveOnHeading_Feedback where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/DriveOnHeading_Feedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DriveOnHeading_Feedback() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DriveOnHeading_FeedbackMessage() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__DriveOnHeading_FeedbackMessage__init(msg: *mut DriveOnHeading_FeedbackMessage) -> bool;
    fn nav2_msgs__action__DriveOnHeading_FeedbackMessage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DriveOnHeading_FeedbackMessage>, size: usize) -> bool;
    fn nav2_msgs__action__DriveOnHeading_FeedbackMessage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DriveOnHeading_FeedbackMessage>);
    fn nav2_msgs__action__DriveOnHeading_FeedbackMessage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DriveOnHeading_FeedbackMessage>, out_seq: *mut rosidl_runtime_rs::Sequence<DriveOnHeading_FeedbackMessage>) -> bool;
}

// Corresponds to nav2_msgs__action__DriveOnHeading_FeedbackMessage
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DriveOnHeading_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::super::action::rmw::DriveOnHeading_Feedback,

}



impl Default for DriveOnHeading_FeedbackMessage {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__DriveOnHeading_FeedbackMessage__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__DriveOnHeading_FeedbackMessage__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DriveOnHeading_FeedbackMessage {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DriveOnHeading_FeedbackMessage__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DriveOnHeading_FeedbackMessage__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DriveOnHeading_FeedbackMessage__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DriveOnHeading_FeedbackMessage {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DriveOnHeading_FeedbackMessage where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/DriveOnHeading_FeedbackMessage";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DriveOnHeading_FeedbackMessage() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__SmoothPath_Goal() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__SmoothPath_Goal__init(msg: *mut SmoothPath_Goal) -> bool;
    fn nav2_msgs__action__SmoothPath_Goal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SmoothPath_Goal>, size: usize) -> bool;
    fn nav2_msgs__action__SmoothPath_Goal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SmoothPath_Goal>);
    fn nav2_msgs__action__SmoothPath_Goal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SmoothPath_Goal>, out_seq: *mut rosidl_runtime_rs::Sequence<SmoothPath_Goal>) -> bool;
}

// Corresponds to nav2_msgs__action__SmoothPath_Goal
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SmoothPath_Goal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub path: nav_msgs::msg::rmw::Path,


    // This member is not documented.
    #[allow(missing_docs)]
    pub smoother_id: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub max_smoothing_duration: builtin_interfaces::msg::rmw::Duration,


    // This member is not documented.
    #[allow(missing_docs)]
    pub check_for_collisions: bool,

}



impl Default for SmoothPath_Goal {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__SmoothPath_Goal__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__SmoothPath_Goal__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SmoothPath_Goal {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__SmoothPath_Goal__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__SmoothPath_Goal__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__SmoothPath_Goal__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SmoothPath_Goal {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SmoothPath_Goal where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/SmoothPath_Goal";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__SmoothPath_Goal() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__SmoothPath_Result() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__SmoothPath_Result__init(msg: *mut SmoothPath_Result) -> bool;
    fn nav2_msgs__action__SmoothPath_Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SmoothPath_Result>, size: usize) -> bool;
    fn nav2_msgs__action__SmoothPath_Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SmoothPath_Result>);
    fn nav2_msgs__action__SmoothPath_Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SmoothPath_Result>, out_seq: *mut rosidl_runtime_rs::Sequence<SmoothPath_Result>) -> bool;
}

// Corresponds to nav2_msgs__action__SmoothPath_Result
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SmoothPath_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub path: nav_msgs::msg::rmw::Path,


    // This member is not documented.
    #[allow(missing_docs)]
    pub smoothing_duration: builtin_interfaces::msg::rmw::Duration,


    // This member is not documented.
    #[allow(missing_docs)]
    pub was_completed: bool,

}



impl Default for SmoothPath_Result {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__SmoothPath_Result__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__SmoothPath_Result__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SmoothPath_Result {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__SmoothPath_Result__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__SmoothPath_Result__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__SmoothPath_Result__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SmoothPath_Result {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SmoothPath_Result where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/SmoothPath_Result";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__SmoothPath_Result() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__SmoothPath_Feedback() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__SmoothPath_Feedback__init(msg: *mut SmoothPath_Feedback) -> bool;
    fn nav2_msgs__action__SmoothPath_Feedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SmoothPath_Feedback>, size: usize) -> bool;
    fn nav2_msgs__action__SmoothPath_Feedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SmoothPath_Feedback>);
    fn nav2_msgs__action__SmoothPath_Feedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SmoothPath_Feedback>, out_seq: *mut rosidl_runtime_rs::Sequence<SmoothPath_Feedback>) -> bool;
}

// Corresponds to nav2_msgs__action__SmoothPath_Feedback
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SmoothPath_Feedback {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for SmoothPath_Feedback {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__SmoothPath_Feedback__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__SmoothPath_Feedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SmoothPath_Feedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__SmoothPath_Feedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__SmoothPath_Feedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__SmoothPath_Feedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SmoothPath_Feedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SmoothPath_Feedback where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/SmoothPath_Feedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__SmoothPath_Feedback() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__SmoothPath_FeedbackMessage() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__SmoothPath_FeedbackMessage__init(msg: *mut SmoothPath_FeedbackMessage) -> bool;
    fn nav2_msgs__action__SmoothPath_FeedbackMessage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SmoothPath_FeedbackMessage>, size: usize) -> bool;
    fn nav2_msgs__action__SmoothPath_FeedbackMessage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SmoothPath_FeedbackMessage>);
    fn nav2_msgs__action__SmoothPath_FeedbackMessage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SmoothPath_FeedbackMessage>, out_seq: *mut rosidl_runtime_rs::Sequence<SmoothPath_FeedbackMessage>) -> bool;
}

// Corresponds to nav2_msgs__action__SmoothPath_FeedbackMessage
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SmoothPath_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::super::action::rmw::SmoothPath_Feedback,

}



impl Default for SmoothPath_FeedbackMessage {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__SmoothPath_FeedbackMessage__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__SmoothPath_FeedbackMessage__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SmoothPath_FeedbackMessage {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__SmoothPath_FeedbackMessage__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__SmoothPath_FeedbackMessage__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__SmoothPath_FeedbackMessage__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SmoothPath_FeedbackMessage {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SmoothPath_FeedbackMessage where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/SmoothPath_FeedbackMessage";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__SmoothPath_FeedbackMessage() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowPath_Goal() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__FollowPath_Goal__init(msg: *mut FollowPath_Goal) -> bool;
    fn nav2_msgs__action__FollowPath_Goal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FollowPath_Goal>, size: usize) -> bool;
    fn nav2_msgs__action__FollowPath_Goal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FollowPath_Goal>);
    fn nav2_msgs__action__FollowPath_Goal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FollowPath_Goal>, out_seq: *mut rosidl_runtime_rs::Sequence<FollowPath_Goal>) -> bool;
}

// Corresponds to nav2_msgs__action__FollowPath_Goal
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowPath_Goal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub path: nav_msgs::msg::rmw::Path,


    // This member is not documented.
    #[allow(missing_docs)]
    pub controller_id: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_checker_id: rosidl_runtime_rs::String,

}



impl Default for FollowPath_Goal {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__FollowPath_Goal__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__FollowPath_Goal__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FollowPath_Goal {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowPath_Goal__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowPath_Goal__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowPath_Goal__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FollowPath_Goal {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FollowPath_Goal where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/FollowPath_Goal";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowPath_Goal() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowPath_Result() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__FollowPath_Result__init(msg: *mut FollowPath_Result) -> bool;
    fn nav2_msgs__action__FollowPath_Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FollowPath_Result>, size: usize) -> bool;
    fn nav2_msgs__action__FollowPath_Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FollowPath_Result>);
    fn nav2_msgs__action__FollowPath_Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FollowPath_Result>, out_seq: *mut rosidl_runtime_rs::Sequence<FollowPath_Result>) -> bool;
}

// Corresponds to nav2_msgs__action__FollowPath_Result
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowPath_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub result: std_msgs::msg::rmw::Empty,

}



impl Default for FollowPath_Result {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__FollowPath_Result__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__FollowPath_Result__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FollowPath_Result {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowPath_Result__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowPath_Result__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowPath_Result__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FollowPath_Result {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FollowPath_Result where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/FollowPath_Result";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowPath_Result() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowPath_Feedback() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__FollowPath_Feedback__init(msg: *mut FollowPath_Feedback) -> bool;
    fn nav2_msgs__action__FollowPath_Feedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FollowPath_Feedback>, size: usize) -> bool;
    fn nav2_msgs__action__FollowPath_Feedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FollowPath_Feedback>);
    fn nav2_msgs__action__FollowPath_Feedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FollowPath_Feedback>, out_seq: *mut rosidl_runtime_rs::Sequence<FollowPath_Feedback>) -> bool;
}

// Corresponds to nav2_msgs__action__FollowPath_Feedback
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowPath_Feedback {

    // This member is not documented.
    #[allow(missing_docs)]
    pub distance_to_goal: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub speed: f32,

}



impl Default for FollowPath_Feedback {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__FollowPath_Feedback__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__FollowPath_Feedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FollowPath_Feedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowPath_Feedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowPath_Feedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowPath_Feedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FollowPath_Feedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FollowPath_Feedback where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/FollowPath_Feedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowPath_Feedback() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowPath_FeedbackMessage() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__FollowPath_FeedbackMessage__init(msg: *mut FollowPath_FeedbackMessage) -> bool;
    fn nav2_msgs__action__FollowPath_FeedbackMessage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FollowPath_FeedbackMessage>, size: usize) -> bool;
    fn nav2_msgs__action__FollowPath_FeedbackMessage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FollowPath_FeedbackMessage>);
    fn nav2_msgs__action__FollowPath_FeedbackMessage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FollowPath_FeedbackMessage>, out_seq: *mut rosidl_runtime_rs::Sequence<FollowPath_FeedbackMessage>) -> bool;
}

// Corresponds to nav2_msgs__action__FollowPath_FeedbackMessage
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowPath_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::super::action::rmw::FollowPath_Feedback,

}



impl Default for FollowPath_FeedbackMessage {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__FollowPath_FeedbackMessage__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__FollowPath_FeedbackMessage__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FollowPath_FeedbackMessage {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowPath_FeedbackMessage__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowPath_FeedbackMessage__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowPath_FeedbackMessage__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FollowPath_FeedbackMessage {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FollowPath_FeedbackMessage where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/FollowPath_FeedbackMessage";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowPath_FeedbackMessage() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateToPose_Goal() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__NavigateToPose_Goal__init(msg: *mut NavigateToPose_Goal) -> bool;
    fn nav2_msgs__action__NavigateToPose_Goal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavigateToPose_Goal>, size: usize) -> bool;
    fn nav2_msgs__action__NavigateToPose_Goal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavigateToPose_Goal>);
    fn nav2_msgs__action__NavigateToPose_Goal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavigateToPose_Goal>, out_seq: *mut rosidl_runtime_rs::Sequence<NavigateToPose_Goal>) -> bool;
}

// Corresponds to nav2_msgs__action__NavigateToPose_Goal
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateToPose_Goal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub pose: geometry_msgs::msg::rmw::PoseStamped,


    // This member is not documented.
    #[allow(missing_docs)]
    pub behavior_tree: rosidl_runtime_rs::String,

}



impl Default for NavigateToPose_Goal {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__NavigateToPose_Goal__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__NavigateToPose_Goal__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavigateToPose_Goal {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateToPose_Goal__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateToPose_Goal__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateToPose_Goal__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavigateToPose_Goal {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavigateToPose_Goal where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/NavigateToPose_Goal";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateToPose_Goal() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateToPose_Result() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__NavigateToPose_Result__init(msg: *mut NavigateToPose_Result) -> bool;
    fn nav2_msgs__action__NavigateToPose_Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavigateToPose_Result>, size: usize) -> bool;
    fn nav2_msgs__action__NavigateToPose_Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavigateToPose_Result>);
    fn nav2_msgs__action__NavigateToPose_Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavigateToPose_Result>, out_seq: *mut rosidl_runtime_rs::Sequence<NavigateToPose_Result>) -> bool;
}

// Corresponds to nav2_msgs__action__NavigateToPose_Result
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateToPose_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub result: std_msgs::msg::rmw::Empty,

}



impl Default for NavigateToPose_Result {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__NavigateToPose_Result__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__NavigateToPose_Result__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavigateToPose_Result {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateToPose_Result__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateToPose_Result__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateToPose_Result__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavigateToPose_Result {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavigateToPose_Result where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/NavigateToPose_Result";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateToPose_Result() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateToPose_Feedback() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__NavigateToPose_Feedback__init(msg: *mut NavigateToPose_Feedback) -> bool;
    fn nav2_msgs__action__NavigateToPose_Feedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavigateToPose_Feedback>, size: usize) -> bool;
    fn nav2_msgs__action__NavigateToPose_Feedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavigateToPose_Feedback>);
    fn nav2_msgs__action__NavigateToPose_Feedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavigateToPose_Feedback>, out_seq: *mut rosidl_runtime_rs::Sequence<NavigateToPose_Feedback>) -> bool;
}

// Corresponds to nav2_msgs__action__NavigateToPose_Feedback
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateToPose_Feedback {

    // This member is not documented.
    #[allow(missing_docs)]
    pub current_pose: geometry_msgs::msg::rmw::PoseStamped,


    // This member is not documented.
    #[allow(missing_docs)]
    pub navigation_time: builtin_interfaces::msg::rmw::Duration,


    // This member is not documented.
    #[allow(missing_docs)]
    pub estimated_time_remaining: builtin_interfaces::msg::rmw::Duration,


    // This member is not documented.
    #[allow(missing_docs)]
    pub number_of_recoveries: i16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub distance_remaining: f32,

}



impl Default for NavigateToPose_Feedback {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__NavigateToPose_Feedback__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__NavigateToPose_Feedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavigateToPose_Feedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateToPose_Feedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateToPose_Feedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateToPose_Feedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavigateToPose_Feedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavigateToPose_Feedback where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/NavigateToPose_Feedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateToPose_Feedback() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateToPose_FeedbackMessage() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__NavigateToPose_FeedbackMessage__init(msg: *mut NavigateToPose_FeedbackMessage) -> bool;
    fn nav2_msgs__action__NavigateToPose_FeedbackMessage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavigateToPose_FeedbackMessage>, size: usize) -> bool;
    fn nav2_msgs__action__NavigateToPose_FeedbackMessage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavigateToPose_FeedbackMessage>);
    fn nav2_msgs__action__NavigateToPose_FeedbackMessage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavigateToPose_FeedbackMessage>, out_seq: *mut rosidl_runtime_rs::Sequence<NavigateToPose_FeedbackMessage>) -> bool;
}

// Corresponds to nav2_msgs__action__NavigateToPose_FeedbackMessage
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateToPose_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::super::action::rmw::NavigateToPose_Feedback,

}



impl Default for NavigateToPose_FeedbackMessage {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__NavigateToPose_FeedbackMessage__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__NavigateToPose_FeedbackMessage__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavigateToPose_FeedbackMessage {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateToPose_FeedbackMessage__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateToPose_FeedbackMessage__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateToPose_FeedbackMessage__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavigateToPose_FeedbackMessage {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavigateToPose_FeedbackMessage where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/NavigateToPose_FeedbackMessage";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateToPose_FeedbackMessage() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateThroughPoses_Goal() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__NavigateThroughPoses_Goal__init(msg: *mut NavigateThroughPoses_Goal) -> bool;
    fn nav2_msgs__action__NavigateThroughPoses_Goal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavigateThroughPoses_Goal>, size: usize) -> bool;
    fn nav2_msgs__action__NavigateThroughPoses_Goal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavigateThroughPoses_Goal>);
    fn nav2_msgs__action__NavigateThroughPoses_Goal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavigateThroughPoses_Goal>, out_seq: *mut rosidl_runtime_rs::Sequence<NavigateThroughPoses_Goal>) -> bool;
}

// Corresponds to nav2_msgs__action__NavigateThroughPoses_Goal
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateThroughPoses_Goal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub poses: rosidl_runtime_rs::Sequence<geometry_msgs::msg::rmw::PoseStamped>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub behavior_tree: rosidl_runtime_rs::String,

}



impl Default for NavigateThroughPoses_Goal {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__NavigateThroughPoses_Goal__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__NavigateThroughPoses_Goal__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavigateThroughPoses_Goal {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateThroughPoses_Goal__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateThroughPoses_Goal__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateThroughPoses_Goal__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavigateThroughPoses_Goal {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavigateThroughPoses_Goal where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/NavigateThroughPoses_Goal";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateThroughPoses_Goal() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateThroughPoses_Result() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__NavigateThroughPoses_Result__init(msg: *mut NavigateThroughPoses_Result) -> bool;
    fn nav2_msgs__action__NavigateThroughPoses_Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavigateThroughPoses_Result>, size: usize) -> bool;
    fn nav2_msgs__action__NavigateThroughPoses_Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavigateThroughPoses_Result>);
    fn nav2_msgs__action__NavigateThroughPoses_Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavigateThroughPoses_Result>, out_seq: *mut rosidl_runtime_rs::Sequence<NavigateThroughPoses_Result>) -> bool;
}

// Corresponds to nav2_msgs__action__NavigateThroughPoses_Result
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateThroughPoses_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub result: std_msgs::msg::rmw::Empty,

}



impl Default for NavigateThroughPoses_Result {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__NavigateThroughPoses_Result__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__NavigateThroughPoses_Result__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavigateThroughPoses_Result {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateThroughPoses_Result__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateThroughPoses_Result__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateThroughPoses_Result__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavigateThroughPoses_Result {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavigateThroughPoses_Result where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/NavigateThroughPoses_Result";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateThroughPoses_Result() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateThroughPoses_Feedback() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__NavigateThroughPoses_Feedback__init(msg: *mut NavigateThroughPoses_Feedback) -> bool;
    fn nav2_msgs__action__NavigateThroughPoses_Feedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavigateThroughPoses_Feedback>, size: usize) -> bool;
    fn nav2_msgs__action__NavigateThroughPoses_Feedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavigateThroughPoses_Feedback>);
    fn nav2_msgs__action__NavigateThroughPoses_Feedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavigateThroughPoses_Feedback>, out_seq: *mut rosidl_runtime_rs::Sequence<NavigateThroughPoses_Feedback>) -> bool;
}

// Corresponds to nav2_msgs__action__NavigateThroughPoses_Feedback
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateThroughPoses_Feedback {

    // This member is not documented.
    #[allow(missing_docs)]
    pub current_pose: geometry_msgs::msg::rmw::PoseStamped,


    // This member is not documented.
    #[allow(missing_docs)]
    pub navigation_time: builtin_interfaces::msg::rmw::Duration,


    // This member is not documented.
    #[allow(missing_docs)]
    pub estimated_time_remaining: builtin_interfaces::msg::rmw::Duration,


    // This member is not documented.
    #[allow(missing_docs)]
    pub number_of_recoveries: i16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub distance_remaining: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub number_of_poses_remaining: i16,

}



impl Default for NavigateThroughPoses_Feedback {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__NavigateThroughPoses_Feedback__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__NavigateThroughPoses_Feedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavigateThroughPoses_Feedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateThroughPoses_Feedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateThroughPoses_Feedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateThroughPoses_Feedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavigateThroughPoses_Feedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavigateThroughPoses_Feedback where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/NavigateThroughPoses_Feedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateThroughPoses_Feedback() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateThroughPoses_FeedbackMessage() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__NavigateThroughPoses_FeedbackMessage__init(msg: *mut NavigateThroughPoses_FeedbackMessage) -> bool;
    fn nav2_msgs__action__NavigateThroughPoses_FeedbackMessage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavigateThroughPoses_FeedbackMessage>, size: usize) -> bool;
    fn nav2_msgs__action__NavigateThroughPoses_FeedbackMessage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavigateThroughPoses_FeedbackMessage>);
    fn nav2_msgs__action__NavigateThroughPoses_FeedbackMessage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavigateThroughPoses_FeedbackMessage>, out_seq: *mut rosidl_runtime_rs::Sequence<NavigateThroughPoses_FeedbackMessage>) -> bool;
}

// Corresponds to nav2_msgs__action__NavigateThroughPoses_FeedbackMessage
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateThroughPoses_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::super::action::rmw::NavigateThroughPoses_Feedback,

}



impl Default for NavigateThroughPoses_FeedbackMessage {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__NavigateThroughPoses_FeedbackMessage__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__NavigateThroughPoses_FeedbackMessage__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavigateThroughPoses_FeedbackMessage {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateThroughPoses_FeedbackMessage__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateThroughPoses_FeedbackMessage__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateThroughPoses_FeedbackMessage__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavigateThroughPoses_FeedbackMessage {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavigateThroughPoses_FeedbackMessage where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/NavigateThroughPoses_FeedbackMessage";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateThroughPoses_FeedbackMessage() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Wait_Goal() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__Wait_Goal__init(msg: *mut Wait_Goal) -> bool;
    fn nav2_msgs__action__Wait_Goal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Wait_Goal>, size: usize) -> bool;
    fn nav2_msgs__action__Wait_Goal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Wait_Goal>);
    fn nav2_msgs__action__Wait_Goal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Wait_Goal>, out_seq: *mut rosidl_runtime_rs::Sequence<Wait_Goal>) -> bool;
}

// Corresponds to nav2_msgs__action__Wait_Goal
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Wait_Goal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub time: builtin_interfaces::msg::rmw::Duration,

}



impl Default for Wait_Goal {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__Wait_Goal__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__Wait_Goal__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Wait_Goal {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Wait_Goal__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Wait_Goal__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Wait_Goal__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Wait_Goal {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Wait_Goal where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/Wait_Goal";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Wait_Goal() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Wait_Result() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__Wait_Result__init(msg: *mut Wait_Result) -> bool;
    fn nav2_msgs__action__Wait_Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Wait_Result>, size: usize) -> bool;
    fn nav2_msgs__action__Wait_Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Wait_Result>);
    fn nav2_msgs__action__Wait_Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Wait_Result>, out_seq: *mut rosidl_runtime_rs::Sequence<Wait_Result>) -> bool;
}

// Corresponds to nav2_msgs__action__Wait_Result
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Wait_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub total_elapsed_time: builtin_interfaces::msg::rmw::Duration,

}



impl Default for Wait_Result {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__Wait_Result__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__Wait_Result__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Wait_Result {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Wait_Result__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Wait_Result__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Wait_Result__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Wait_Result {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Wait_Result where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/Wait_Result";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Wait_Result() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Wait_Feedback() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__Wait_Feedback__init(msg: *mut Wait_Feedback) -> bool;
    fn nav2_msgs__action__Wait_Feedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Wait_Feedback>, size: usize) -> bool;
    fn nav2_msgs__action__Wait_Feedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Wait_Feedback>);
    fn nav2_msgs__action__Wait_Feedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Wait_Feedback>, out_seq: *mut rosidl_runtime_rs::Sequence<Wait_Feedback>) -> bool;
}

// Corresponds to nav2_msgs__action__Wait_Feedback
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Wait_Feedback {

    // This member is not documented.
    #[allow(missing_docs)]
    pub time_left: builtin_interfaces::msg::rmw::Duration,

}



impl Default for Wait_Feedback {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__Wait_Feedback__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__Wait_Feedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Wait_Feedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Wait_Feedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Wait_Feedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Wait_Feedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Wait_Feedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Wait_Feedback where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/Wait_Feedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Wait_Feedback() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Wait_FeedbackMessage() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__Wait_FeedbackMessage__init(msg: *mut Wait_FeedbackMessage) -> bool;
    fn nav2_msgs__action__Wait_FeedbackMessage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Wait_FeedbackMessage>, size: usize) -> bool;
    fn nav2_msgs__action__Wait_FeedbackMessage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Wait_FeedbackMessage>);
    fn nav2_msgs__action__Wait_FeedbackMessage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Wait_FeedbackMessage>, out_seq: *mut rosidl_runtime_rs::Sequence<Wait_FeedbackMessage>) -> bool;
}

// Corresponds to nav2_msgs__action__Wait_FeedbackMessage
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Wait_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::super::action::rmw::Wait_Feedback,

}



impl Default for Wait_FeedbackMessage {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__Wait_FeedbackMessage__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__Wait_FeedbackMessage__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Wait_FeedbackMessage {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Wait_FeedbackMessage__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Wait_FeedbackMessage__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Wait_FeedbackMessage__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Wait_FeedbackMessage {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Wait_FeedbackMessage where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/Wait_FeedbackMessage";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Wait_FeedbackMessage() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Spin_Goal() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__Spin_Goal__init(msg: *mut Spin_Goal) -> bool;
    fn nav2_msgs__action__Spin_Goal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Spin_Goal>, size: usize) -> bool;
    fn nav2_msgs__action__Spin_Goal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Spin_Goal>);
    fn nav2_msgs__action__Spin_Goal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Spin_Goal>, out_seq: *mut rosidl_runtime_rs::Sequence<Spin_Goal>) -> bool;
}

// Corresponds to nav2_msgs__action__Spin_Goal
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Spin_Goal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub target_yaw: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub time_allowance: builtin_interfaces::msg::rmw::Duration,

}



impl Default for Spin_Goal {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__Spin_Goal__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__Spin_Goal__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Spin_Goal {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Spin_Goal__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Spin_Goal__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Spin_Goal__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Spin_Goal {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Spin_Goal where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/Spin_Goal";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Spin_Goal() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Spin_Result() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__Spin_Result__init(msg: *mut Spin_Result) -> bool;
    fn nav2_msgs__action__Spin_Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Spin_Result>, size: usize) -> bool;
    fn nav2_msgs__action__Spin_Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Spin_Result>);
    fn nav2_msgs__action__Spin_Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Spin_Result>, out_seq: *mut rosidl_runtime_rs::Sequence<Spin_Result>) -> bool;
}

// Corresponds to nav2_msgs__action__Spin_Result
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Spin_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub total_elapsed_time: builtin_interfaces::msg::rmw::Duration,

}



impl Default for Spin_Result {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__Spin_Result__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__Spin_Result__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Spin_Result {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Spin_Result__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Spin_Result__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Spin_Result__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Spin_Result {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Spin_Result where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/Spin_Result";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Spin_Result() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Spin_Feedback() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__Spin_Feedback__init(msg: *mut Spin_Feedback) -> bool;
    fn nav2_msgs__action__Spin_Feedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Spin_Feedback>, size: usize) -> bool;
    fn nav2_msgs__action__Spin_Feedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Spin_Feedback>);
    fn nav2_msgs__action__Spin_Feedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Spin_Feedback>, out_seq: *mut rosidl_runtime_rs::Sequence<Spin_Feedback>) -> bool;
}

// Corresponds to nav2_msgs__action__Spin_Feedback
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Spin_Feedback {

    // This member is not documented.
    #[allow(missing_docs)]
    pub angular_distance_traveled: f32,

}



impl Default for Spin_Feedback {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__Spin_Feedback__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__Spin_Feedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Spin_Feedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Spin_Feedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Spin_Feedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Spin_Feedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Spin_Feedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Spin_Feedback where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/Spin_Feedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Spin_Feedback() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Spin_FeedbackMessage() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__Spin_FeedbackMessage__init(msg: *mut Spin_FeedbackMessage) -> bool;
    fn nav2_msgs__action__Spin_FeedbackMessage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Spin_FeedbackMessage>, size: usize) -> bool;
    fn nav2_msgs__action__Spin_FeedbackMessage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Spin_FeedbackMessage>);
    fn nav2_msgs__action__Spin_FeedbackMessage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Spin_FeedbackMessage>, out_seq: *mut rosidl_runtime_rs::Sequence<Spin_FeedbackMessage>) -> bool;
}

// Corresponds to nav2_msgs__action__Spin_FeedbackMessage
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Spin_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::super::action::rmw::Spin_Feedback,

}



impl Default for Spin_FeedbackMessage {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__Spin_FeedbackMessage__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__Spin_FeedbackMessage__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Spin_FeedbackMessage {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Spin_FeedbackMessage__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Spin_FeedbackMessage__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Spin_FeedbackMessage__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Spin_FeedbackMessage {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Spin_FeedbackMessage where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/Spin_FeedbackMessage";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Spin_FeedbackMessage() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DummyBehavior_Goal() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__DummyBehavior_Goal__init(msg: *mut DummyBehavior_Goal) -> bool;
    fn nav2_msgs__action__DummyBehavior_Goal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DummyBehavior_Goal>, size: usize) -> bool;
    fn nav2_msgs__action__DummyBehavior_Goal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DummyBehavior_Goal>);
    fn nav2_msgs__action__DummyBehavior_Goal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DummyBehavior_Goal>, out_seq: *mut rosidl_runtime_rs::Sequence<DummyBehavior_Goal>) -> bool;
}

// Corresponds to nav2_msgs__action__DummyBehavior_Goal
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DummyBehavior_Goal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub command: std_msgs::msg::rmw::String,

}



impl Default for DummyBehavior_Goal {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__DummyBehavior_Goal__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__DummyBehavior_Goal__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DummyBehavior_Goal {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DummyBehavior_Goal__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DummyBehavior_Goal__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DummyBehavior_Goal__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DummyBehavior_Goal {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DummyBehavior_Goal where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/DummyBehavior_Goal";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DummyBehavior_Goal() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DummyBehavior_Result() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__DummyBehavior_Result__init(msg: *mut DummyBehavior_Result) -> bool;
    fn nav2_msgs__action__DummyBehavior_Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DummyBehavior_Result>, size: usize) -> bool;
    fn nav2_msgs__action__DummyBehavior_Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DummyBehavior_Result>);
    fn nav2_msgs__action__DummyBehavior_Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DummyBehavior_Result>, out_seq: *mut rosidl_runtime_rs::Sequence<DummyBehavior_Result>) -> bool;
}

// Corresponds to nav2_msgs__action__DummyBehavior_Result
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DummyBehavior_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub total_elapsed_time: builtin_interfaces::msg::rmw::Duration,

}



impl Default for DummyBehavior_Result {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__DummyBehavior_Result__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__DummyBehavior_Result__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DummyBehavior_Result {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DummyBehavior_Result__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DummyBehavior_Result__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DummyBehavior_Result__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DummyBehavior_Result {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DummyBehavior_Result where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/DummyBehavior_Result";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DummyBehavior_Result() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DummyBehavior_Feedback() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__DummyBehavior_Feedback__init(msg: *mut DummyBehavior_Feedback) -> bool;
    fn nav2_msgs__action__DummyBehavior_Feedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DummyBehavior_Feedback>, size: usize) -> bool;
    fn nav2_msgs__action__DummyBehavior_Feedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DummyBehavior_Feedback>);
    fn nav2_msgs__action__DummyBehavior_Feedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DummyBehavior_Feedback>, out_seq: *mut rosidl_runtime_rs::Sequence<DummyBehavior_Feedback>) -> bool;
}

// Corresponds to nav2_msgs__action__DummyBehavior_Feedback
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DummyBehavior_Feedback {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for DummyBehavior_Feedback {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__DummyBehavior_Feedback__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__DummyBehavior_Feedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DummyBehavior_Feedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DummyBehavior_Feedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DummyBehavior_Feedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DummyBehavior_Feedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DummyBehavior_Feedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DummyBehavior_Feedback where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/DummyBehavior_Feedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DummyBehavior_Feedback() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DummyBehavior_FeedbackMessage() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__DummyBehavior_FeedbackMessage__init(msg: *mut DummyBehavior_FeedbackMessage) -> bool;
    fn nav2_msgs__action__DummyBehavior_FeedbackMessage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DummyBehavior_FeedbackMessage>, size: usize) -> bool;
    fn nav2_msgs__action__DummyBehavior_FeedbackMessage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DummyBehavior_FeedbackMessage>);
    fn nav2_msgs__action__DummyBehavior_FeedbackMessage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DummyBehavior_FeedbackMessage>, out_seq: *mut rosidl_runtime_rs::Sequence<DummyBehavior_FeedbackMessage>) -> bool;
}

// Corresponds to nav2_msgs__action__DummyBehavior_FeedbackMessage
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DummyBehavior_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::super::action::rmw::DummyBehavior_Feedback,

}



impl Default for DummyBehavior_FeedbackMessage {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__DummyBehavior_FeedbackMessage__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__DummyBehavior_FeedbackMessage__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DummyBehavior_FeedbackMessage {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DummyBehavior_FeedbackMessage__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DummyBehavior_FeedbackMessage__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DummyBehavior_FeedbackMessage__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DummyBehavior_FeedbackMessage {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DummyBehavior_FeedbackMessage where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/DummyBehavior_FeedbackMessage";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DummyBehavior_FeedbackMessage() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowWaypoints_Goal() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__FollowWaypoints_Goal__init(msg: *mut FollowWaypoints_Goal) -> bool;
    fn nav2_msgs__action__FollowWaypoints_Goal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FollowWaypoints_Goal>, size: usize) -> bool;
    fn nav2_msgs__action__FollowWaypoints_Goal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FollowWaypoints_Goal>);
    fn nav2_msgs__action__FollowWaypoints_Goal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FollowWaypoints_Goal>, out_seq: *mut rosidl_runtime_rs::Sequence<FollowWaypoints_Goal>) -> bool;
}

// Corresponds to nav2_msgs__action__FollowWaypoints_Goal
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowWaypoints_Goal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub poses: rosidl_runtime_rs::Sequence<geometry_msgs::msg::rmw::PoseStamped>,

}



impl Default for FollowWaypoints_Goal {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__FollowWaypoints_Goal__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__FollowWaypoints_Goal__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FollowWaypoints_Goal {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowWaypoints_Goal__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowWaypoints_Goal__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowWaypoints_Goal__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FollowWaypoints_Goal {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FollowWaypoints_Goal where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/FollowWaypoints_Goal";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowWaypoints_Goal() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowWaypoints_Result() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__FollowWaypoints_Result__init(msg: *mut FollowWaypoints_Result) -> bool;
    fn nav2_msgs__action__FollowWaypoints_Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FollowWaypoints_Result>, size: usize) -> bool;
    fn nav2_msgs__action__FollowWaypoints_Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FollowWaypoints_Result>);
    fn nav2_msgs__action__FollowWaypoints_Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FollowWaypoints_Result>, out_seq: *mut rosidl_runtime_rs::Sequence<FollowWaypoints_Result>) -> bool;
}

// Corresponds to nav2_msgs__action__FollowWaypoints_Result
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowWaypoints_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub missed_waypoints: rosidl_runtime_rs::Sequence<i32>,

}



impl Default for FollowWaypoints_Result {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__FollowWaypoints_Result__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__FollowWaypoints_Result__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FollowWaypoints_Result {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowWaypoints_Result__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowWaypoints_Result__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowWaypoints_Result__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FollowWaypoints_Result {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FollowWaypoints_Result where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/FollowWaypoints_Result";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowWaypoints_Result() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowWaypoints_Feedback() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__FollowWaypoints_Feedback__init(msg: *mut FollowWaypoints_Feedback) -> bool;
    fn nav2_msgs__action__FollowWaypoints_Feedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FollowWaypoints_Feedback>, size: usize) -> bool;
    fn nav2_msgs__action__FollowWaypoints_Feedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FollowWaypoints_Feedback>);
    fn nav2_msgs__action__FollowWaypoints_Feedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FollowWaypoints_Feedback>, out_seq: *mut rosidl_runtime_rs::Sequence<FollowWaypoints_Feedback>) -> bool;
}

// Corresponds to nav2_msgs__action__FollowWaypoints_Feedback
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowWaypoints_Feedback {

    // This member is not documented.
    #[allow(missing_docs)]
    pub current_waypoint: u32,

}



impl Default for FollowWaypoints_Feedback {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__FollowWaypoints_Feedback__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__FollowWaypoints_Feedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FollowWaypoints_Feedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowWaypoints_Feedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowWaypoints_Feedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowWaypoints_Feedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FollowWaypoints_Feedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FollowWaypoints_Feedback where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/FollowWaypoints_Feedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowWaypoints_Feedback() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowWaypoints_FeedbackMessage() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__FollowWaypoints_FeedbackMessage__init(msg: *mut FollowWaypoints_FeedbackMessage) -> bool;
    fn nav2_msgs__action__FollowWaypoints_FeedbackMessage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FollowWaypoints_FeedbackMessage>, size: usize) -> bool;
    fn nav2_msgs__action__FollowWaypoints_FeedbackMessage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FollowWaypoints_FeedbackMessage>);
    fn nav2_msgs__action__FollowWaypoints_FeedbackMessage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FollowWaypoints_FeedbackMessage>, out_seq: *mut rosidl_runtime_rs::Sequence<FollowWaypoints_FeedbackMessage>) -> bool;
}

// Corresponds to nav2_msgs__action__FollowWaypoints_FeedbackMessage
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowWaypoints_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::super::action::rmw::FollowWaypoints_Feedback,

}



impl Default for FollowWaypoints_FeedbackMessage {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__FollowWaypoints_FeedbackMessage__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__FollowWaypoints_FeedbackMessage__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FollowWaypoints_FeedbackMessage {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowWaypoints_FeedbackMessage__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowWaypoints_FeedbackMessage__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowWaypoints_FeedbackMessage__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FollowWaypoints_FeedbackMessage {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FollowWaypoints_FeedbackMessage where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/FollowWaypoints_FeedbackMessage";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowWaypoints_FeedbackMessage() }
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__AssistedTeleop_SendGoal_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__AssistedTeleop_SendGoal_Request__init(msg: *mut AssistedTeleop_SendGoal_Request) -> bool;
    fn nav2_msgs__action__AssistedTeleop_SendGoal_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<AssistedTeleop_SendGoal_Request>, size: usize) -> bool;
    fn nav2_msgs__action__AssistedTeleop_SendGoal_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<AssistedTeleop_SendGoal_Request>);
    fn nav2_msgs__action__AssistedTeleop_SendGoal_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<AssistedTeleop_SendGoal_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<AssistedTeleop_SendGoal_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__AssistedTeleop_SendGoal_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AssistedTeleop_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::super::action::rmw::AssistedTeleop_Goal,

}



impl Default for AssistedTeleop_SendGoal_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__AssistedTeleop_SendGoal_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__AssistedTeleop_SendGoal_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for AssistedTeleop_SendGoal_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__AssistedTeleop_SendGoal_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__AssistedTeleop_SendGoal_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__AssistedTeleop_SendGoal_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for AssistedTeleop_SendGoal_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for AssistedTeleop_SendGoal_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/AssistedTeleop_SendGoal_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__AssistedTeleop_SendGoal_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__AssistedTeleop_SendGoal_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__AssistedTeleop_SendGoal_Response__init(msg: *mut AssistedTeleop_SendGoal_Response) -> bool;
    fn nav2_msgs__action__AssistedTeleop_SendGoal_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<AssistedTeleop_SendGoal_Response>, size: usize) -> bool;
    fn nav2_msgs__action__AssistedTeleop_SendGoal_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<AssistedTeleop_SendGoal_Response>);
    fn nav2_msgs__action__AssistedTeleop_SendGoal_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<AssistedTeleop_SendGoal_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<AssistedTeleop_SendGoal_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__AssistedTeleop_SendGoal_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AssistedTeleop_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,

}



impl Default for AssistedTeleop_SendGoal_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__AssistedTeleop_SendGoal_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__AssistedTeleop_SendGoal_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for AssistedTeleop_SendGoal_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__AssistedTeleop_SendGoal_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__AssistedTeleop_SendGoal_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__AssistedTeleop_SendGoal_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for AssistedTeleop_SendGoal_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for AssistedTeleop_SendGoal_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/AssistedTeleop_SendGoal_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__AssistedTeleop_SendGoal_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__AssistedTeleop_GetResult_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__AssistedTeleop_GetResult_Request__init(msg: *mut AssistedTeleop_GetResult_Request) -> bool;
    fn nav2_msgs__action__AssistedTeleop_GetResult_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<AssistedTeleop_GetResult_Request>, size: usize) -> bool;
    fn nav2_msgs__action__AssistedTeleop_GetResult_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<AssistedTeleop_GetResult_Request>);
    fn nav2_msgs__action__AssistedTeleop_GetResult_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<AssistedTeleop_GetResult_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<AssistedTeleop_GetResult_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__AssistedTeleop_GetResult_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AssistedTeleop_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,

}



impl Default for AssistedTeleop_GetResult_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__AssistedTeleop_GetResult_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__AssistedTeleop_GetResult_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for AssistedTeleop_GetResult_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__AssistedTeleop_GetResult_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__AssistedTeleop_GetResult_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__AssistedTeleop_GetResult_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for AssistedTeleop_GetResult_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for AssistedTeleop_GetResult_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/AssistedTeleop_GetResult_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__AssistedTeleop_GetResult_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__AssistedTeleop_GetResult_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__AssistedTeleop_GetResult_Response__init(msg: *mut AssistedTeleop_GetResult_Response) -> bool;
    fn nav2_msgs__action__AssistedTeleop_GetResult_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<AssistedTeleop_GetResult_Response>, size: usize) -> bool;
    fn nav2_msgs__action__AssistedTeleop_GetResult_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<AssistedTeleop_GetResult_Response>);
    fn nav2_msgs__action__AssistedTeleop_GetResult_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<AssistedTeleop_GetResult_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<AssistedTeleop_GetResult_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__AssistedTeleop_GetResult_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AssistedTeleop_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::super::action::rmw::AssistedTeleop_Result,

}



impl Default for AssistedTeleop_GetResult_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__AssistedTeleop_GetResult_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__AssistedTeleop_GetResult_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for AssistedTeleop_GetResult_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__AssistedTeleop_GetResult_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__AssistedTeleop_GetResult_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__AssistedTeleop_GetResult_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for AssistedTeleop_GetResult_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for AssistedTeleop_GetResult_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/AssistedTeleop_GetResult_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__AssistedTeleop_GetResult_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__BackUp_SendGoal_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__BackUp_SendGoal_Request__init(msg: *mut BackUp_SendGoal_Request) -> bool;
    fn nav2_msgs__action__BackUp_SendGoal_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<BackUp_SendGoal_Request>, size: usize) -> bool;
    fn nav2_msgs__action__BackUp_SendGoal_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<BackUp_SendGoal_Request>);
    fn nav2_msgs__action__BackUp_SendGoal_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<BackUp_SendGoal_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<BackUp_SendGoal_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__BackUp_SendGoal_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BackUp_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::super::action::rmw::BackUp_Goal,

}



impl Default for BackUp_SendGoal_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__BackUp_SendGoal_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__BackUp_SendGoal_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for BackUp_SendGoal_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__BackUp_SendGoal_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__BackUp_SendGoal_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__BackUp_SendGoal_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for BackUp_SendGoal_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for BackUp_SendGoal_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/BackUp_SendGoal_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__BackUp_SendGoal_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__BackUp_SendGoal_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__BackUp_SendGoal_Response__init(msg: *mut BackUp_SendGoal_Response) -> bool;
    fn nav2_msgs__action__BackUp_SendGoal_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<BackUp_SendGoal_Response>, size: usize) -> bool;
    fn nav2_msgs__action__BackUp_SendGoal_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<BackUp_SendGoal_Response>);
    fn nav2_msgs__action__BackUp_SendGoal_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<BackUp_SendGoal_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<BackUp_SendGoal_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__BackUp_SendGoal_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BackUp_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,

}



impl Default for BackUp_SendGoal_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__BackUp_SendGoal_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__BackUp_SendGoal_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for BackUp_SendGoal_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__BackUp_SendGoal_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__BackUp_SendGoal_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__BackUp_SendGoal_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for BackUp_SendGoal_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for BackUp_SendGoal_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/BackUp_SendGoal_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__BackUp_SendGoal_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__BackUp_GetResult_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__BackUp_GetResult_Request__init(msg: *mut BackUp_GetResult_Request) -> bool;
    fn nav2_msgs__action__BackUp_GetResult_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<BackUp_GetResult_Request>, size: usize) -> bool;
    fn nav2_msgs__action__BackUp_GetResult_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<BackUp_GetResult_Request>);
    fn nav2_msgs__action__BackUp_GetResult_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<BackUp_GetResult_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<BackUp_GetResult_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__BackUp_GetResult_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BackUp_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,

}



impl Default for BackUp_GetResult_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__BackUp_GetResult_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__BackUp_GetResult_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for BackUp_GetResult_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__BackUp_GetResult_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__BackUp_GetResult_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__BackUp_GetResult_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for BackUp_GetResult_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for BackUp_GetResult_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/BackUp_GetResult_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__BackUp_GetResult_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__BackUp_GetResult_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__BackUp_GetResult_Response__init(msg: *mut BackUp_GetResult_Response) -> bool;
    fn nav2_msgs__action__BackUp_GetResult_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<BackUp_GetResult_Response>, size: usize) -> bool;
    fn nav2_msgs__action__BackUp_GetResult_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<BackUp_GetResult_Response>);
    fn nav2_msgs__action__BackUp_GetResult_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<BackUp_GetResult_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<BackUp_GetResult_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__BackUp_GetResult_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BackUp_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::super::action::rmw::BackUp_Result,

}



impl Default for BackUp_GetResult_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__BackUp_GetResult_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__BackUp_GetResult_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for BackUp_GetResult_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__BackUp_GetResult_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__BackUp_GetResult_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__BackUp_GetResult_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for BackUp_GetResult_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for BackUp_GetResult_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/BackUp_GetResult_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__BackUp_GetResult_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathToPose_SendGoal_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__ComputePathToPose_SendGoal_Request__init(msg: *mut ComputePathToPose_SendGoal_Request) -> bool;
    fn nav2_msgs__action__ComputePathToPose_SendGoal_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ComputePathToPose_SendGoal_Request>, size: usize) -> bool;
    fn nav2_msgs__action__ComputePathToPose_SendGoal_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ComputePathToPose_SendGoal_Request>);
    fn nav2_msgs__action__ComputePathToPose_SendGoal_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ComputePathToPose_SendGoal_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<ComputePathToPose_SendGoal_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__ComputePathToPose_SendGoal_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathToPose_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::super::action::rmw::ComputePathToPose_Goal,

}



impl Default for ComputePathToPose_SendGoal_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__ComputePathToPose_SendGoal_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__ComputePathToPose_SendGoal_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ComputePathToPose_SendGoal_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathToPose_SendGoal_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathToPose_SendGoal_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathToPose_SendGoal_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ComputePathToPose_SendGoal_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ComputePathToPose_SendGoal_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/ComputePathToPose_SendGoal_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathToPose_SendGoal_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathToPose_SendGoal_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__ComputePathToPose_SendGoal_Response__init(msg: *mut ComputePathToPose_SendGoal_Response) -> bool;
    fn nav2_msgs__action__ComputePathToPose_SendGoal_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ComputePathToPose_SendGoal_Response>, size: usize) -> bool;
    fn nav2_msgs__action__ComputePathToPose_SendGoal_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ComputePathToPose_SendGoal_Response>);
    fn nav2_msgs__action__ComputePathToPose_SendGoal_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ComputePathToPose_SendGoal_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<ComputePathToPose_SendGoal_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__ComputePathToPose_SendGoal_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathToPose_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,

}



impl Default for ComputePathToPose_SendGoal_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__ComputePathToPose_SendGoal_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__ComputePathToPose_SendGoal_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ComputePathToPose_SendGoal_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathToPose_SendGoal_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathToPose_SendGoal_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathToPose_SendGoal_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ComputePathToPose_SendGoal_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ComputePathToPose_SendGoal_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/ComputePathToPose_SendGoal_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathToPose_SendGoal_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathToPose_GetResult_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__ComputePathToPose_GetResult_Request__init(msg: *mut ComputePathToPose_GetResult_Request) -> bool;
    fn nav2_msgs__action__ComputePathToPose_GetResult_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ComputePathToPose_GetResult_Request>, size: usize) -> bool;
    fn nav2_msgs__action__ComputePathToPose_GetResult_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ComputePathToPose_GetResult_Request>);
    fn nav2_msgs__action__ComputePathToPose_GetResult_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ComputePathToPose_GetResult_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<ComputePathToPose_GetResult_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__ComputePathToPose_GetResult_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathToPose_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,

}



impl Default for ComputePathToPose_GetResult_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__ComputePathToPose_GetResult_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__ComputePathToPose_GetResult_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ComputePathToPose_GetResult_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathToPose_GetResult_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathToPose_GetResult_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathToPose_GetResult_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ComputePathToPose_GetResult_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ComputePathToPose_GetResult_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/ComputePathToPose_GetResult_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathToPose_GetResult_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathToPose_GetResult_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__ComputePathToPose_GetResult_Response__init(msg: *mut ComputePathToPose_GetResult_Response) -> bool;
    fn nav2_msgs__action__ComputePathToPose_GetResult_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ComputePathToPose_GetResult_Response>, size: usize) -> bool;
    fn nav2_msgs__action__ComputePathToPose_GetResult_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ComputePathToPose_GetResult_Response>);
    fn nav2_msgs__action__ComputePathToPose_GetResult_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ComputePathToPose_GetResult_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<ComputePathToPose_GetResult_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__ComputePathToPose_GetResult_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathToPose_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::super::action::rmw::ComputePathToPose_Result,

}



impl Default for ComputePathToPose_GetResult_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__ComputePathToPose_GetResult_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__ComputePathToPose_GetResult_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ComputePathToPose_GetResult_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathToPose_GetResult_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathToPose_GetResult_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathToPose_GetResult_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ComputePathToPose_GetResult_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ComputePathToPose_GetResult_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/ComputePathToPose_GetResult_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathToPose_GetResult_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathThroughPoses_SendGoal_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__ComputePathThroughPoses_SendGoal_Request__init(msg: *mut ComputePathThroughPoses_SendGoal_Request) -> bool;
    fn nav2_msgs__action__ComputePathThroughPoses_SendGoal_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ComputePathThroughPoses_SendGoal_Request>, size: usize) -> bool;
    fn nav2_msgs__action__ComputePathThroughPoses_SendGoal_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ComputePathThroughPoses_SendGoal_Request>);
    fn nav2_msgs__action__ComputePathThroughPoses_SendGoal_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ComputePathThroughPoses_SendGoal_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<ComputePathThroughPoses_SendGoal_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__ComputePathThroughPoses_SendGoal_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathThroughPoses_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::super::action::rmw::ComputePathThroughPoses_Goal,

}



impl Default for ComputePathThroughPoses_SendGoal_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__ComputePathThroughPoses_SendGoal_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__ComputePathThroughPoses_SendGoal_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ComputePathThroughPoses_SendGoal_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathThroughPoses_SendGoal_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathThroughPoses_SendGoal_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathThroughPoses_SendGoal_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ComputePathThroughPoses_SendGoal_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ComputePathThroughPoses_SendGoal_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/ComputePathThroughPoses_SendGoal_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathThroughPoses_SendGoal_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathThroughPoses_SendGoal_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__ComputePathThroughPoses_SendGoal_Response__init(msg: *mut ComputePathThroughPoses_SendGoal_Response) -> bool;
    fn nav2_msgs__action__ComputePathThroughPoses_SendGoal_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ComputePathThroughPoses_SendGoal_Response>, size: usize) -> bool;
    fn nav2_msgs__action__ComputePathThroughPoses_SendGoal_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ComputePathThroughPoses_SendGoal_Response>);
    fn nav2_msgs__action__ComputePathThroughPoses_SendGoal_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ComputePathThroughPoses_SendGoal_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<ComputePathThroughPoses_SendGoal_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__ComputePathThroughPoses_SendGoal_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathThroughPoses_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,

}



impl Default for ComputePathThroughPoses_SendGoal_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__ComputePathThroughPoses_SendGoal_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__ComputePathThroughPoses_SendGoal_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ComputePathThroughPoses_SendGoal_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathThroughPoses_SendGoal_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathThroughPoses_SendGoal_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathThroughPoses_SendGoal_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ComputePathThroughPoses_SendGoal_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ComputePathThroughPoses_SendGoal_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/ComputePathThroughPoses_SendGoal_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathThroughPoses_SendGoal_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathThroughPoses_GetResult_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__ComputePathThroughPoses_GetResult_Request__init(msg: *mut ComputePathThroughPoses_GetResult_Request) -> bool;
    fn nav2_msgs__action__ComputePathThroughPoses_GetResult_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ComputePathThroughPoses_GetResult_Request>, size: usize) -> bool;
    fn nav2_msgs__action__ComputePathThroughPoses_GetResult_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ComputePathThroughPoses_GetResult_Request>);
    fn nav2_msgs__action__ComputePathThroughPoses_GetResult_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ComputePathThroughPoses_GetResult_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<ComputePathThroughPoses_GetResult_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__ComputePathThroughPoses_GetResult_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathThroughPoses_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,

}



impl Default for ComputePathThroughPoses_GetResult_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__ComputePathThroughPoses_GetResult_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__ComputePathThroughPoses_GetResult_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ComputePathThroughPoses_GetResult_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathThroughPoses_GetResult_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathThroughPoses_GetResult_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathThroughPoses_GetResult_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ComputePathThroughPoses_GetResult_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ComputePathThroughPoses_GetResult_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/ComputePathThroughPoses_GetResult_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathThroughPoses_GetResult_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathThroughPoses_GetResult_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__ComputePathThroughPoses_GetResult_Response__init(msg: *mut ComputePathThroughPoses_GetResult_Response) -> bool;
    fn nav2_msgs__action__ComputePathThroughPoses_GetResult_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ComputePathThroughPoses_GetResult_Response>, size: usize) -> bool;
    fn nav2_msgs__action__ComputePathThroughPoses_GetResult_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ComputePathThroughPoses_GetResult_Response>);
    fn nav2_msgs__action__ComputePathThroughPoses_GetResult_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ComputePathThroughPoses_GetResult_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<ComputePathThroughPoses_GetResult_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__ComputePathThroughPoses_GetResult_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathThroughPoses_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::super::action::rmw::ComputePathThroughPoses_Result,

}



impl Default for ComputePathThroughPoses_GetResult_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__ComputePathThroughPoses_GetResult_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__ComputePathThroughPoses_GetResult_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ComputePathThroughPoses_GetResult_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathThroughPoses_GetResult_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathThroughPoses_GetResult_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputePathThroughPoses_GetResult_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ComputePathThroughPoses_GetResult_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ComputePathThroughPoses_GetResult_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/ComputePathThroughPoses_GetResult_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathThroughPoses_GetResult_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputeRoute_SendGoal_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__ComputeRoute_SendGoal_Request__init(msg: *mut ComputeRoute_SendGoal_Request) -> bool;
    fn nav2_msgs__action__ComputeRoute_SendGoal_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ComputeRoute_SendGoal_Request>, size: usize) -> bool;
    fn nav2_msgs__action__ComputeRoute_SendGoal_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ComputeRoute_SendGoal_Request>);
    fn nav2_msgs__action__ComputeRoute_SendGoal_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ComputeRoute_SendGoal_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<ComputeRoute_SendGoal_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__ComputeRoute_SendGoal_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputeRoute_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::super::action::rmw::ComputeRoute_Goal,

}



impl Default for ComputeRoute_SendGoal_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__ComputeRoute_SendGoal_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__ComputeRoute_SendGoal_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ComputeRoute_SendGoal_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputeRoute_SendGoal_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputeRoute_SendGoal_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputeRoute_SendGoal_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ComputeRoute_SendGoal_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ComputeRoute_SendGoal_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/ComputeRoute_SendGoal_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputeRoute_SendGoal_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputeRoute_SendGoal_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__ComputeRoute_SendGoal_Response__init(msg: *mut ComputeRoute_SendGoal_Response) -> bool;
    fn nav2_msgs__action__ComputeRoute_SendGoal_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ComputeRoute_SendGoal_Response>, size: usize) -> bool;
    fn nav2_msgs__action__ComputeRoute_SendGoal_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ComputeRoute_SendGoal_Response>);
    fn nav2_msgs__action__ComputeRoute_SendGoal_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ComputeRoute_SendGoal_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<ComputeRoute_SendGoal_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__ComputeRoute_SendGoal_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputeRoute_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,

}



impl Default for ComputeRoute_SendGoal_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__ComputeRoute_SendGoal_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__ComputeRoute_SendGoal_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ComputeRoute_SendGoal_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputeRoute_SendGoal_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputeRoute_SendGoal_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputeRoute_SendGoal_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ComputeRoute_SendGoal_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ComputeRoute_SendGoal_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/ComputeRoute_SendGoal_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputeRoute_SendGoal_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputeRoute_GetResult_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__ComputeRoute_GetResult_Request__init(msg: *mut ComputeRoute_GetResult_Request) -> bool;
    fn nav2_msgs__action__ComputeRoute_GetResult_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ComputeRoute_GetResult_Request>, size: usize) -> bool;
    fn nav2_msgs__action__ComputeRoute_GetResult_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ComputeRoute_GetResult_Request>);
    fn nav2_msgs__action__ComputeRoute_GetResult_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ComputeRoute_GetResult_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<ComputeRoute_GetResult_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__ComputeRoute_GetResult_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputeRoute_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,

}



impl Default for ComputeRoute_GetResult_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__ComputeRoute_GetResult_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__ComputeRoute_GetResult_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ComputeRoute_GetResult_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputeRoute_GetResult_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputeRoute_GetResult_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputeRoute_GetResult_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ComputeRoute_GetResult_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ComputeRoute_GetResult_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/ComputeRoute_GetResult_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputeRoute_GetResult_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputeRoute_GetResult_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__ComputeRoute_GetResult_Response__init(msg: *mut ComputeRoute_GetResult_Response) -> bool;
    fn nav2_msgs__action__ComputeRoute_GetResult_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ComputeRoute_GetResult_Response>, size: usize) -> bool;
    fn nav2_msgs__action__ComputeRoute_GetResult_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ComputeRoute_GetResult_Response>);
    fn nav2_msgs__action__ComputeRoute_GetResult_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ComputeRoute_GetResult_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<ComputeRoute_GetResult_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__ComputeRoute_GetResult_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputeRoute_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::super::action::rmw::ComputeRoute_Result,

}



impl Default for ComputeRoute_GetResult_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__ComputeRoute_GetResult_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__ComputeRoute_GetResult_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ComputeRoute_GetResult_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputeRoute_GetResult_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputeRoute_GetResult_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputeRoute_GetResult_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ComputeRoute_GetResult_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ComputeRoute_GetResult_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/ComputeRoute_GetResult_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputeRoute_GetResult_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputeAndTrackRoute_SendGoal_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__ComputeAndTrackRoute_SendGoal_Request__init(msg: *mut ComputeAndTrackRoute_SendGoal_Request) -> bool;
    fn nav2_msgs__action__ComputeAndTrackRoute_SendGoal_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ComputeAndTrackRoute_SendGoal_Request>, size: usize) -> bool;
    fn nav2_msgs__action__ComputeAndTrackRoute_SendGoal_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ComputeAndTrackRoute_SendGoal_Request>);
    fn nav2_msgs__action__ComputeAndTrackRoute_SendGoal_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ComputeAndTrackRoute_SendGoal_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<ComputeAndTrackRoute_SendGoal_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__ComputeAndTrackRoute_SendGoal_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputeAndTrackRoute_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::super::action::rmw::ComputeAndTrackRoute_Goal,

}



impl Default for ComputeAndTrackRoute_SendGoal_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__ComputeAndTrackRoute_SendGoal_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__ComputeAndTrackRoute_SendGoal_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ComputeAndTrackRoute_SendGoal_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputeAndTrackRoute_SendGoal_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputeAndTrackRoute_SendGoal_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputeAndTrackRoute_SendGoal_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ComputeAndTrackRoute_SendGoal_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ComputeAndTrackRoute_SendGoal_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/ComputeAndTrackRoute_SendGoal_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputeAndTrackRoute_SendGoal_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputeAndTrackRoute_SendGoal_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__ComputeAndTrackRoute_SendGoal_Response__init(msg: *mut ComputeAndTrackRoute_SendGoal_Response) -> bool;
    fn nav2_msgs__action__ComputeAndTrackRoute_SendGoal_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ComputeAndTrackRoute_SendGoal_Response>, size: usize) -> bool;
    fn nav2_msgs__action__ComputeAndTrackRoute_SendGoal_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ComputeAndTrackRoute_SendGoal_Response>);
    fn nav2_msgs__action__ComputeAndTrackRoute_SendGoal_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ComputeAndTrackRoute_SendGoal_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<ComputeAndTrackRoute_SendGoal_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__ComputeAndTrackRoute_SendGoal_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputeAndTrackRoute_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,

}



impl Default for ComputeAndTrackRoute_SendGoal_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__ComputeAndTrackRoute_SendGoal_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__ComputeAndTrackRoute_SendGoal_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ComputeAndTrackRoute_SendGoal_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputeAndTrackRoute_SendGoal_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputeAndTrackRoute_SendGoal_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputeAndTrackRoute_SendGoal_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ComputeAndTrackRoute_SendGoal_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ComputeAndTrackRoute_SendGoal_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/ComputeAndTrackRoute_SendGoal_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputeAndTrackRoute_SendGoal_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputeAndTrackRoute_GetResult_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__ComputeAndTrackRoute_GetResult_Request__init(msg: *mut ComputeAndTrackRoute_GetResult_Request) -> bool;
    fn nav2_msgs__action__ComputeAndTrackRoute_GetResult_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ComputeAndTrackRoute_GetResult_Request>, size: usize) -> bool;
    fn nav2_msgs__action__ComputeAndTrackRoute_GetResult_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ComputeAndTrackRoute_GetResult_Request>);
    fn nav2_msgs__action__ComputeAndTrackRoute_GetResult_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ComputeAndTrackRoute_GetResult_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<ComputeAndTrackRoute_GetResult_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__ComputeAndTrackRoute_GetResult_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputeAndTrackRoute_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,

}



impl Default for ComputeAndTrackRoute_GetResult_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__ComputeAndTrackRoute_GetResult_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__ComputeAndTrackRoute_GetResult_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ComputeAndTrackRoute_GetResult_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputeAndTrackRoute_GetResult_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputeAndTrackRoute_GetResult_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputeAndTrackRoute_GetResult_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ComputeAndTrackRoute_GetResult_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ComputeAndTrackRoute_GetResult_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/ComputeAndTrackRoute_GetResult_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputeAndTrackRoute_GetResult_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputeAndTrackRoute_GetResult_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__ComputeAndTrackRoute_GetResult_Response__init(msg: *mut ComputeAndTrackRoute_GetResult_Response) -> bool;
    fn nav2_msgs__action__ComputeAndTrackRoute_GetResult_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ComputeAndTrackRoute_GetResult_Response>, size: usize) -> bool;
    fn nav2_msgs__action__ComputeAndTrackRoute_GetResult_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ComputeAndTrackRoute_GetResult_Response>);
    fn nav2_msgs__action__ComputeAndTrackRoute_GetResult_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ComputeAndTrackRoute_GetResult_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<ComputeAndTrackRoute_GetResult_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__ComputeAndTrackRoute_GetResult_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputeAndTrackRoute_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::super::action::rmw::ComputeAndTrackRoute_Result,

}



impl Default for ComputeAndTrackRoute_GetResult_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__ComputeAndTrackRoute_GetResult_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__ComputeAndTrackRoute_GetResult_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ComputeAndTrackRoute_GetResult_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputeAndTrackRoute_GetResult_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputeAndTrackRoute_GetResult_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__ComputeAndTrackRoute_GetResult_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ComputeAndTrackRoute_GetResult_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ComputeAndTrackRoute_GetResult_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/ComputeAndTrackRoute_GetResult_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputeAndTrackRoute_GetResult_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DriveOnHeading_SendGoal_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__DriveOnHeading_SendGoal_Request__init(msg: *mut DriveOnHeading_SendGoal_Request) -> bool;
    fn nav2_msgs__action__DriveOnHeading_SendGoal_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DriveOnHeading_SendGoal_Request>, size: usize) -> bool;
    fn nav2_msgs__action__DriveOnHeading_SendGoal_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DriveOnHeading_SendGoal_Request>);
    fn nav2_msgs__action__DriveOnHeading_SendGoal_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DriveOnHeading_SendGoal_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<DriveOnHeading_SendGoal_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__DriveOnHeading_SendGoal_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DriveOnHeading_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::super::action::rmw::DriveOnHeading_Goal,

}



impl Default for DriveOnHeading_SendGoal_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__DriveOnHeading_SendGoal_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__DriveOnHeading_SendGoal_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DriveOnHeading_SendGoal_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DriveOnHeading_SendGoal_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DriveOnHeading_SendGoal_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DriveOnHeading_SendGoal_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DriveOnHeading_SendGoal_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DriveOnHeading_SendGoal_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/DriveOnHeading_SendGoal_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DriveOnHeading_SendGoal_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DriveOnHeading_SendGoal_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__DriveOnHeading_SendGoal_Response__init(msg: *mut DriveOnHeading_SendGoal_Response) -> bool;
    fn nav2_msgs__action__DriveOnHeading_SendGoal_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DriveOnHeading_SendGoal_Response>, size: usize) -> bool;
    fn nav2_msgs__action__DriveOnHeading_SendGoal_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DriveOnHeading_SendGoal_Response>);
    fn nav2_msgs__action__DriveOnHeading_SendGoal_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DriveOnHeading_SendGoal_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<DriveOnHeading_SendGoal_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__DriveOnHeading_SendGoal_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DriveOnHeading_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,

}



impl Default for DriveOnHeading_SendGoal_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__DriveOnHeading_SendGoal_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__DriveOnHeading_SendGoal_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DriveOnHeading_SendGoal_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DriveOnHeading_SendGoal_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DriveOnHeading_SendGoal_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DriveOnHeading_SendGoal_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DriveOnHeading_SendGoal_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DriveOnHeading_SendGoal_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/DriveOnHeading_SendGoal_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DriveOnHeading_SendGoal_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DriveOnHeading_GetResult_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__DriveOnHeading_GetResult_Request__init(msg: *mut DriveOnHeading_GetResult_Request) -> bool;
    fn nav2_msgs__action__DriveOnHeading_GetResult_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DriveOnHeading_GetResult_Request>, size: usize) -> bool;
    fn nav2_msgs__action__DriveOnHeading_GetResult_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DriveOnHeading_GetResult_Request>);
    fn nav2_msgs__action__DriveOnHeading_GetResult_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DriveOnHeading_GetResult_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<DriveOnHeading_GetResult_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__DriveOnHeading_GetResult_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DriveOnHeading_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,

}



impl Default for DriveOnHeading_GetResult_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__DriveOnHeading_GetResult_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__DriveOnHeading_GetResult_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DriveOnHeading_GetResult_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DriveOnHeading_GetResult_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DriveOnHeading_GetResult_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DriveOnHeading_GetResult_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DriveOnHeading_GetResult_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DriveOnHeading_GetResult_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/DriveOnHeading_GetResult_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DriveOnHeading_GetResult_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DriveOnHeading_GetResult_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__DriveOnHeading_GetResult_Response__init(msg: *mut DriveOnHeading_GetResult_Response) -> bool;
    fn nav2_msgs__action__DriveOnHeading_GetResult_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DriveOnHeading_GetResult_Response>, size: usize) -> bool;
    fn nav2_msgs__action__DriveOnHeading_GetResult_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DriveOnHeading_GetResult_Response>);
    fn nav2_msgs__action__DriveOnHeading_GetResult_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DriveOnHeading_GetResult_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<DriveOnHeading_GetResult_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__DriveOnHeading_GetResult_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DriveOnHeading_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::super::action::rmw::DriveOnHeading_Result,

}



impl Default for DriveOnHeading_GetResult_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__DriveOnHeading_GetResult_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__DriveOnHeading_GetResult_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DriveOnHeading_GetResult_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DriveOnHeading_GetResult_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DriveOnHeading_GetResult_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DriveOnHeading_GetResult_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DriveOnHeading_GetResult_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DriveOnHeading_GetResult_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/DriveOnHeading_GetResult_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DriveOnHeading_GetResult_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__SmoothPath_SendGoal_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__SmoothPath_SendGoal_Request__init(msg: *mut SmoothPath_SendGoal_Request) -> bool;
    fn nav2_msgs__action__SmoothPath_SendGoal_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SmoothPath_SendGoal_Request>, size: usize) -> bool;
    fn nav2_msgs__action__SmoothPath_SendGoal_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SmoothPath_SendGoal_Request>);
    fn nav2_msgs__action__SmoothPath_SendGoal_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SmoothPath_SendGoal_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<SmoothPath_SendGoal_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__SmoothPath_SendGoal_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SmoothPath_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::super::action::rmw::SmoothPath_Goal,

}



impl Default for SmoothPath_SendGoal_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__SmoothPath_SendGoal_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__SmoothPath_SendGoal_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SmoothPath_SendGoal_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__SmoothPath_SendGoal_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__SmoothPath_SendGoal_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__SmoothPath_SendGoal_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SmoothPath_SendGoal_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SmoothPath_SendGoal_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/SmoothPath_SendGoal_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__SmoothPath_SendGoal_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__SmoothPath_SendGoal_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__SmoothPath_SendGoal_Response__init(msg: *mut SmoothPath_SendGoal_Response) -> bool;
    fn nav2_msgs__action__SmoothPath_SendGoal_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SmoothPath_SendGoal_Response>, size: usize) -> bool;
    fn nav2_msgs__action__SmoothPath_SendGoal_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SmoothPath_SendGoal_Response>);
    fn nav2_msgs__action__SmoothPath_SendGoal_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SmoothPath_SendGoal_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<SmoothPath_SendGoal_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__SmoothPath_SendGoal_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SmoothPath_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,

}



impl Default for SmoothPath_SendGoal_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__SmoothPath_SendGoal_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__SmoothPath_SendGoal_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SmoothPath_SendGoal_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__SmoothPath_SendGoal_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__SmoothPath_SendGoal_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__SmoothPath_SendGoal_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SmoothPath_SendGoal_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SmoothPath_SendGoal_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/SmoothPath_SendGoal_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__SmoothPath_SendGoal_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__SmoothPath_GetResult_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__SmoothPath_GetResult_Request__init(msg: *mut SmoothPath_GetResult_Request) -> bool;
    fn nav2_msgs__action__SmoothPath_GetResult_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SmoothPath_GetResult_Request>, size: usize) -> bool;
    fn nav2_msgs__action__SmoothPath_GetResult_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SmoothPath_GetResult_Request>);
    fn nav2_msgs__action__SmoothPath_GetResult_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SmoothPath_GetResult_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<SmoothPath_GetResult_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__SmoothPath_GetResult_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SmoothPath_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,

}



impl Default for SmoothPath_GetResult_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__SmoothPath_GetResult_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__SmoothPath_GetResult_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SmoothPath_GetResult_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__SmoothPath_GetResult_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__SmoothPath_GetResult_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__SmoothPath_GetResult_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SmoothPath_GetResult_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SmoothPath_GetResult_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/SmoothPath_GetResult_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__SmoothPath_GetResult_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__SmoothPath_GetResult_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__SmoothPath_GetResult_Response__init(msg: *mut SmoothPath_GetResult_Response) -> bool;
    fn nav2_msgs__action__SmoothPath_GetResult_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SmoothPath_GetResult_Response>, size: usize) -> bool;
    fn nav2_msgs__action__SmoothPath_GetResult_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SmoothPath_GetResult_Response>);
    fn nav2_msgs__action__SmoothPath_GetResult_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SmoothPath_GetResult_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<SmoothPath_GetResult_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__SmoothPath_GetResult_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SmoothPath_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::super::action::rmw::SmoothPath_Result,

}



impl Default for SmoothPath_GetResult_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__SmoothPath_GetResult_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__SmoothPath_GetResult_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SmoothPath_GetResult_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__SmoothPath_GetResult_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__SmoothPath_GetResult_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__SmoothPath_GetResult_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SmoothPath_GetResult_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SmoothPath_GetResult_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/SmoothPath_GetResult_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__SmoothPath_GetResult_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowPath_SendGoal_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__FollowPath_SendGoal_Request__init(msg: *mut FollowPath_SendGoal_Request) -> bool;
    fn nav2_msgs__action__FollowPath_SendGoal_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FollowPath_SendGoal_Request>, size: usize) -> bool;
    fn nav2_msgs__action__FollowPath_SendGoal_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FollowPath_SendGoal_Request>);
    fn nav2_msgs__action__FollowPath_SendGoal_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FollowPath_SendGoal_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<FollowPath_SendGoal_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__FollowPath_SendGoal_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowPath_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::super::action::rmw::FollowPath_Goal,

}



impl Default for FollowPath_SendGoal_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__FollowPath_SendGoal_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__FollowPath_SendGoal_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FollowPath_SendGoal_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowPath_SendGoal_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowPath_SendGoal_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowPath_SendGoal_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FollowPath_SendGoal_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FollowPath_SendGoal_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/FollowPath_SendGoal_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowPath_SendGoal_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowPath_SendGoal_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__FollowPath_SendGoal_Response__init(msg: *mut FollowPath_SendGoal_Response) -> bool;
    fn nav2_msgs__action__FollowPath_SendGoal_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FollowPath_SendGoal_Response>, size: usize) -> bool;
    fn nav2_msgs__action__FollowPath_SendGoal_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FollowPath_SendGoal_Response>);
    fn nav2_msgs__action__FollowPath_SendGoal_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FollowPath_SendGoal_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<FollowPath_SendGoal_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__FollowPath_SendGoal_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowPath_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,

}



impl Default for FollowPath_SendGoal_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__FollowPath_SendGoal_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__FollowPath_SendGoal_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FollowPath_SendGoal_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowPath_SendGoal_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowPath_SendGoal_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowPath_SendGoal_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FollowPath_SendGoal_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FollowPath_SendGoal_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/FollowPath_SendGoal_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowPath_SendGoal_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowPath_GetResult_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__FollowPath_GetResult_Request__init(msg: *mut FollowPath_GetResult_Request) -> bool;
    fn nav2_msgs__action__FollowPath_GetResult_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FollowPath_GetResult_Request>, size: usize) -> bool;
    fn nav2_msgs__action__FollowPath_GetResult_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FollowPath_GetResult_Request>);
    fn nav2_msgs__action__FollowPath_GetResult_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FollowPath_GetResult_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<FollowPath_GetResult_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__FollowPath_GetResult_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowPath_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,

}



impl Default for FollowPath_GetResult_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__FollowPath_GetResult_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__FollowPath_GetResult_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FollowPath_GetResult_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowPath_GetResult_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowPath_GetResult_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowPath_GetResult_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FollowPath_GetResult_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FollowPath_GetResult_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/FollowPath_GetResult_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowPath_GetResult_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowPath_GetResult_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__FollowPath_GetResult_Response__init(msg: *mut FollowPath_GetResult_Response) -> bool;
    fn nav2_msgs__action__FollowPath_GetResult_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FollowPath_GetResult_Response>, size: usize) -> bool;
    fn nav2_msgs__action__FollowPath_GetResult_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FollowPath_GetResult_Response>);
    fn nav2_msgs__action__FollowPath_GetResult_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FollowPath_GetResult_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<FollowPath_GetResult_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__FollowPath_GetResult_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowPath_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::super::action::rmw::FollowPath_Result,

}



impl Default for FollowPath_GetResult_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__FollowPath_GetResult_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__FollowPath_GetResult_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FollowPath_GetResult_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowPath_GetResult_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowPath_GetResult_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowPath_GetResult_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FollowPath_GetResult_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FollowPath_GetResult_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/FollowPath_GetResult_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowPath_GetResult_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateToPose_SendGoal_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__NavigateToPose_SendGoal_Request__init(msg: *mut NavigateToPose_SendGoal_Request) -> bool;
    fn nav2_msgs__action__NavigateToPose_SendGoal_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavigateToPose_SendGoal_Request>, size: usize) -> bool;
    fn nav2_msgs__action__NavigateToPose_SendGoal_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavigateToPose_SendGoal_Request>);
    fn nav2_msgs__action__NavigateToPose_SendGoal_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavigateToPose_SendGoal_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<NavigateToPose_SendGoal_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__NavigateToPose_SendGoal_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateToPose_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::super::action::rmw::NavigateToPose_Goal,

}



impl Default for NavigateToPose_SendGoal_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__NavigateToPose_SendGoal_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__NavigateToPose_SendGoal_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavigateToPose_SendGoal_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateToPose_SendGoal_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateToPose_SendGoal_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateToPose_SendGoal_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavigateToPose_SendGoal_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavigateToPose_SendGoal_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/NavigateToPose_SendGoal_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateToPose_SendGoal_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateToPose_SendGoal_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__NavigateToPose_SendGoal_Response__init(msg: *mut NavigateToPose_SendGoal_Response) -> bool;
    fn nav2_msgs__action__NavigateToPose_SendGoal_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavigateToPose_SendGoal_Response>, size: usize) -> bool;
    fn nav2_msgs__action__NavigateToPose_SendGoal_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavigateToPose_SendGoal_Response>);
    fn nav2_msgs__action__NavigateToPose_SendGoal_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavigateToPose_SendGoal_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<NavigateToPose_SendGoal_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__NavigateToPose_SendGoal_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateToPose_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,

}



impl Default for NavigateToPose_SendGoal_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__NavigateToPose_SendGoal_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__NavigateToPose_SendGoal_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavigateToPose_SendGoal_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateToPose_SendGoal_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateToPose_SendGoal_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateToPose_SendGoal_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavigateToPose_SendGoal_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavigateToPose_SendGoal_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/NavigateToPose_SendGoal_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateToPose_SendGoal_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateToPose_GetResult_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__NavigateToPose_GetResult_Request__init(msg: *mut NavigateToPose_GetResult_Request) -> bool;
    fn nav2_msgs__action__NavigateToPose_GetResult_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavigateToPose_GetResult_Request>, size: usize) -> bool;
    fn nav2_msgs__action__NavigateToPose_GetResult_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavigateToPose_GetResult_Request>);
    fn nav2_msgs__action__NavigateToPose_GetResult_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavigateToPose_GetResult_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<NavigateToPose_GetResult_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__NavigateToPose_GetResult_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateToPose_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,

}



impl Default for NavigateToPose_GetResult_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__NavigateToPose_GetResult_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__NavigateToPose_GetResult_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavigateToPose_GetResult_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateToPose_GetResult_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateToPose_GetResult_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateToPose_GetResult_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavigateToPose_GetResult_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavigateToPose_GetResult_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/NavigateToPose_GetResult_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateToPose_GetResult_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateToPose_GetResult_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__NavigateToPose_GetResult_Response__init(msg: *mut NavigateToPose_GetResult_Response) -> bool;
    fn nav2_msgs__action__NavigateToPose_GetResult_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavigateToPose_GetResult_Response>, size: usize) -> bool;
    fn nav2_msgs__action__NavigateToPose_GetResult_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavigateToPose_GetResult_Response>);
    fn nav2_msgs__action__NavigateToPose_GetResult_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavigateToPose_GetResult_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<NavigateToPose_GetResult_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__NavigateToPose_GetResult_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateToPose_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::super::action::rmw::NavigateToPose_Result,

}



impl Default for NavigateToPose_GetResult_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__NavigateToPose_GetResult_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__NavigateToPose_GetResult_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavigateToPose_GetResult_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateToPose_GetResult_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateToPose_GetResult_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateToPose_GetResult_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavigateToPose_GetResult_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavigateToPose_GetResult_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/NavigateToPose_GetResult_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateToPose_GetResult_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateThroughPoses_SendGoal_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__NavigateThroughPoses_SendGoal_Request__init(msg: *mut NavigateThroughPoses_SendGoal_Request) -> bool;
    fn nav2_msgs__action__NavigateThroughPoses_SendGoal_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavigateThroughPoses_SendGoal_Request>, size: usize) -> bool;
    fn nav2_msgs__action__NavigateThroughPoses_SendGoal_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavigateThroughPoses_SendGoal_Request>);
    fn nav2_msgs__action__NavigateThroughPoses_SendGoal_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavigateThroughPoses_SendGoal_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<NavigateThroughPoses_SendGoal_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__NavigateThroughPoses_SendGoal_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateThroughPoses_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::super::action::rmw::NavigateThroughPoses_Goal,

}



impl Default for NavigateThroughPoses_SendGoal_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__NavigateThroughPoses_SendGoal_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__NavigateThroughPoses_SendGoal_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavigateThroughPoses_SendGoal_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateThroughPoses_SendGoal_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateThroughPoses_SendGoal_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateThroughPoses_SendGoal_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavigateThroughPoses_SendGoal_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavigateThroughPoses_SendGoal_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/NavigateThroughPoses_SendGoal_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateThroughPoses_SendGoal_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateThroughPoses_SendGoal_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__NavigateThroughPoses_SendGoal_Response__init(msg: *mut NavigateThroughPoses_SendGoal_Response) -> bool;
    fn nav2_msgs__action__NavigateThroughPoses_SendGoal_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavigateThroughPoses_SendGoal_Response>, size: usize) -> bool;
    fn nav2_msgs__action__NavigateThroughPoses_SendGoal_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavigateThroughPoses_SendGoal_Response>);
    fn nav2_msgs__action__NavigateThroughPoses_SendGoal_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavigateThroughPoses_SendGoal_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<NavigateThroughPoses_SendGoal_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__NavigateThroughPoses_SendGoal_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateThroughPoses_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,

}



impl Default for NavigateThroughPoses_SendGoal_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__NavigateThroughPoses_SendGoal_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__NavigateThroughPoses_SendGoal_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavigateThroughPoses_SendGoal_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateThroughPoses_SendGoal_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateThroughPoses_SendGoal_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateThroughPoses_SendGoal_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavigateThroughPoses_SendGoal_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavigateThroughPoses_SendGoal_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/NavigateThroughPoses_SendGoal_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateThroughPoses_SendGoal_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateThroughPoses_GetResult_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__NavigateThroughPoses_GetResult_Request__init(msg: *mut NavigateThroughPoses_GetResult_Request) -> bool;
    fn nav2_msgs__action__NavigateThroughPoses_GetResult_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavigateThroughPoses_GetResult_Request>, size: usize) -> bool;
    fn nav2_msgs__action__NavigateThroughPoses_GetResult_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavigateThroughPoses_GetResult_Request>);
    fn nav2_msgs__action__NavigateThroughPoses_GetResult_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavigateThroughPoses_GetResult_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<NavigateThroughPoses_GetResult_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__NavigateThroughPoses_GetResult_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateThroughPoses_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,

}



impl Default for NavigateThroughPoses_GetResult_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__NavigateThroughPoses_GetResult_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__NavigateThroughPoses_GetResult_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavigateThroughPoses_GetResult_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateThroughPoses_GetResult_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateThroughPoses_GetResult_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateThroughPoses_GetResult_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavigateThroughPoses_GetResult_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavigateThroughPoses_GetResult_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/NavigateThroughPoses_GetResult_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateThroughPoses_GetResult_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateThroughPoses_GetResult_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__NavigateThroughPoses_GetResult_Response__init(msg: *mut NavigateThroughPoses_GetResult_Response) -> bool;
    fn nav2_msgs__action__NavigateThroughPoses_GetResult_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavigateThroughPoses_GetResult_Response>, size: usize) -> bool;
    fn nav2_msgs__action__NavigateThroughPoses_GetResult_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavigateThroughPoses_GetResult_Response>);
    fn nav2_msgs__action__NavigateThroughPoses_GetResult_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavigateThroughPoses_GetResult_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<NavigateThroughPoses_GetResult_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__NavigateThroughPoses_GetResult_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateThroughPoses_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::super::action::rmw::NavigateThroughPoses_Result,

}



impl Default for NavigateThroughPoses_GetResult_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__NavigateThroughPoses_GetResult_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__NavigateThroughPoses_GetResult_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavigateThroughPoses_GetResult_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateThroughPoses_GetResult_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateThroughPoses_GetResult_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__NavigateThroughPoses_GetResult_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavigateThroughPoses_GetResult_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavigateThroughPoses_GetResult_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/NavigateThroughPoses_GetResult_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateThroughPoses_GetResult_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Wait_SendGoal_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__Wait_SendGoal_Request__init(msg: *mut Wait_SendGoal_Request) -> bool;
    fn nav2_msgs__action__Wait_SendGoal_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Wait_SendGoal_Request>, size: usize) -> bool;
    fn nav2_msgs__action__Wait_SendGoal_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Wait_SendGoal_Request>);
    fn nav2_msgs__action__Wait_SendGoal_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Wait_SendGoal_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<Wait_SendGoal_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__Wait_SendGoal_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Wait_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::super::action::rmw::Wait_Goal,

}



impl Default for Wait_SendGoal_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__Wait_SendGoal_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__Wait_SendGoal_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Wait_SendGoal_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Wait_SendGoal_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Wait_SendGoal_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Wait_SendGoal_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Wait_SendGoal_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Wait_SendGoal_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/Wait_SendGoal_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Wait_SendGoal_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Wait_SendGoal_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__Wait_SendGoal_Response__init(msg: *mut Wait_SendGoal_Response) -> bool;
    fn nav2_msgs__action__Wait_SendGoal_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Wait_SendGoal_Response>, size: usize) -> bool;
    fn nav2_msgs__action__Wait_SendGoal_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Wait_SendGoal_Response>);
    fn nav2_msgs__action__Wait_SendGoal_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Wait_SendGoal_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<Wait_SendGoal_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__Wait_SendGoal_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Wait_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,

}



impl Default for Wait_SendGoal_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__Wait_SendGoal_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__Wait_SendGoal_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Wait_SendGoal_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Wait_SendGoal_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Wait_SendGoal_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Wait_SendGoal_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Wait_SendGoal_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Wait_SendGoal_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/Wait_SendGoal_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Wait_SendGoal_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Wait_GetResult_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__Wait_GetResult_Request__init(msg: *mut Wait_GetResult_Request) -> bool;
    fn nav2_msgs__action__Wait_GetResult_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Wait_GetResult_Request>, size: usize) -> bool;
    fn nav2_msgs__action__Wait_GetResult_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Wait_GetResult_Request>);
    fn nav2_msgs__action__Wait_GetResult_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Wait_GetResult_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<Wait_GetResult_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__Wait_GetResult_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Wait_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,

}



impl Default for Wait_GetResult_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__Wait_GetResult_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__Wait_GetResult_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Wait_GetResult_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Wait_GetResult_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Wait_GetResult_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Wait_GetResult_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Wait_GetResult_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Wait_GetResult_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/Wait_GetResult_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Wait_GetResult_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Wait_GetResult_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__Wait_GetResult_Response__init(msg: *mut Wait_GetResult_Response) -> bool;
    fn nav2_msgs__action__Wait_GetResult_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Wait_GetResult_Response>, size: usize) -> bool;
    fn nav2_msgs__action__Wait_GetResult_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Wait_GetResult_Response>);
    fn nav2_msgs__action__Wait_GetResult_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Wait_GetResult_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<Wait_GetResult_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__Wait_GetResult_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Wait_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::super::action::rmw::Wait_Result,

}



impl Default for Wait_GetResult_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__Wait_GetResult_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__Wait_GetResult_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Wait_GetResult_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Wait_GetResult_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Wait_GetResult_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Wait_GetResult_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Wait_GetResult_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Wait_GetResult_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/Wait_GetResult_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Wait_GetResult_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Spin_SendGoal_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__Spin_SendGoal_Request__init(msg: *mut Spin_SendGoal_Request) -> bool;
    fn nav2_msgs__action__Spin_SendGoal_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Spin_SendGoal_Request>, size: usize) -> bool;
    fn nav2_msgs__action__Spin_SendGoal_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Spin_SendGoal_Request>);
    fn nav2_msgs__action__Spin_SendGoal_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Spin_SendGoal_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<Spin_SendGoal_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__Spin_SendGoal_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Spin_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::super::action::rmw::Spin_Goal,

}



impl Default for Spin_SendGoal_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__Spin_SendGoal_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__Spin_SendGoal_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Spin_SendGoal_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Spin_SendGoal_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Spin_SendGoal_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Spin_SendGoal_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Spin_SendGoal_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Spin_SendGoal_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/Spin_SendGoal_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Spin_SendGoal_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Spin_SendGoal_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__Spin_SendGoal_Response__init(msg: *mut Spin_SendGoal_Response) -> bool;
    fn nav2_msgs__action__Spin_SendGoal_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Spin_SendGoal_Response>, size: usize) -> bool;
    fn nav2_msgs__action__Spin_SendGoal_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Spin_SendGoal_Response>);
    fn nav2_msgs__action__Spin_SendGoal_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Spin_SendGoal_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<Spin_SendGoal_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__Spin_SendGoal_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Spin_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,

}



impl Default for Spin_SendGoal_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__Spin_SendGoal_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__Spin_SendGoal_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Spin_SendGoal_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Spin_SendGoal_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Spin_SendGoal_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Spin_SendGoal_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Spin_SendGoal_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Spin_SendGoal_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/Spin_SendGoal_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Spin_SendGoal_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Spin_GetResult_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__Spin_GetResult_Request__init(msg: *mut Spin_GetResult_Request) -> bool;
    fn nav2_msgs__action__Spin_GetResult_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Spin_GetResult_Request>, size: usize) -> bool;
    fn nav2_msgs__action__Spin_GetResult_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Spin_GetResult_Request>);
    fn nav2_msgs__action__Spin_GetResult_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Spin_GetResult_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<Spin_GetResult_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__Spin_GetResult_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Spin_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,

}



impl Default for Spin_GetResult_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__Spin_GetResult_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__Spin_GetResult_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Spin_GetResult_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Spin_GetResult_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Spin_GetResult_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Spin_GetResult_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Spin_GetResult_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Spin_GetResult_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/Spin_GetResult_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Spin_GetResult_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Spin_GetResult_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__Spin_GetResult_Response__init(msg: *mut Spin_GetResult_Response) -> bool;
    fn nav2_msgs__action__Spin_GetResult_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Spin_GetResult_Response>, size: usize) -> bool;
    fn nav2_msgs__action__Spin_GetResult_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Spin_GetResult_Response>);
    fn nav2_msgs__action__Spin_GetResult_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Spin_GetResult_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<Spin_GetResult_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__Spin_GetResult_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Spin_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::super::action::rmw::Spin_Result,

}



impl Default for Spin_GetResult_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__Spin_GetResult_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__Spin_GetResult_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Spin_GetResult_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Spin_GetResult_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Spin_GetResult_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__Spin_GetResult_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Spin_GetResult_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Spin_GetResult_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/Spin_GetResult_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Spin_GetResult_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DummyBehavior_SendGoal_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__DummyBehavior_SendGoal_Request__init(msg: *mut DummyBehavior_SendGoal_Request) -> bool;
    fn nav2_msgs__action__DummyBehavior_SendGoal_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DummyBehavior_SendGoal_Request>, size: usize) -> bool;
    fn nav2_msgs__action__DummyBehavior_SendGoal_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DummyBehavior_SendGoal_Request>);
    fn nav2_msgs__action__DummyBehavior_SendGoal_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DummyBehavior_SendGoal_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<DummyBehavior_SendGoal_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__DummyBehavior_SendGoal_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DummyBehavior_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::super::action::rmw::DummyBehavior_Goal,

}



impl Default for DummyBehavior_SendGoal_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__DummyBehavior_SendGoal_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__DummyBehavior_SendGoal_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DummyBehavior_SendGoal_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DummyBehavior_SendGoal_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DummyBehavior_SendGoal_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DummyBehavior_SendGoal_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DummyBehavior_SendGoal_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DummyBehavior_SendGoal_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/DummyBehavior_SendGoal_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DummyBehavior_SendGoal_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DummyBehavior_SendGoal_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__DummyBehavior_SendGoal_Response__init(msg: *mut DummyBehavior_SendGoal_Response) -> bool;
    fn nav2_msgs__action__DummyBehavior_SendGoal_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DummyBehavior_SendGoal_Response>, size: usize) -> bool;
    fn nav2_msgs__action__DummyBehavior_SendGoal_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DummyBehavior_SendGoal_Response>);
    fn nav2_msgs__action__DummyBehavior_SendGoal_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DummyBehavior_SendGoal_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<DummyBehavior_SendGoal_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__DummyBehavior_SendGoal_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DummyBehavior_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,

}



impl Default for DummyBehavior_SendGoal_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__DummyBehavior_SendGoal_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__DummyBehavior_SendGoal_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DummyBehavior_SendGoal_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DummyBehavior_SendGoal_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DummyBehavior_SendGoal_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DummyBehavior_SendGoal_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DummyBehavior_SendGoal_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DummyBehavior_SendGoal_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/DummyBehavior_SendGoal_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DummyBehavior_SendGoal_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DummyBehavior_GetResult_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__DummyBehavior_GetResult_Request__init(msg: *mut DummyBehavior_GetResult_Request) -> bool;
    fn nav2_msgs__action__DummyBehavior_GetResult_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DummyBehavior_GetResult_Request>, size: usize) -> bool;
    fn nav2_msgs__action__DummyBehavior_GetResult_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DummyBehavior_GetResult_Request>);
    fn nav2_msgs__action__DummyBehavior_GetResult_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DummyBehavior_GetResult_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<DummyBehavior_GetResult_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__DummyBehavior_GetResult_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DummyBehavior_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,

}



impl Default for DummyBehavior_GetResult_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__DummyBehavior_GetResult_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__DummyBehavior_GetResult_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DummyBehavior_GetResult_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DummyBehavior_GetResult_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DummyBehavior_GetResult_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DummyBehavior_GetResult_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DummyBehavior_GetResult_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DummyBehavior_GetResult_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/DummyBehavior_GetResult_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DummyBehavior_GetResult_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DummyBehavior_GetResult_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__DummyBehavior_GetResult_Response__init(msg: *mut DummyBehavior_GetResult_Response) -> bool;
    fn nav2_msgs__action__DummyBehavior_GetResult_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DummyBehavior_GetResult_Response>, size: usize) -> bool;
    fn nav2_msgs__action__DummyBehavior_GetResult_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DummyBehavior_GetResult_Response>);
    fn nav2_msgs__action__DummyBehavior_GetResult_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DummyBehavior_GetResult_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<DummyBehavior_GetResult_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__DummyBehavior_GetResult_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DummyBehavior_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::super::action::rmw::DummyBehavior_Result,

}



impl Default for DummyBehavior_GetResult_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__DummyBehavior_GetResult_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__DummyBehavior_GetResult_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DummyBehavior_GetResult_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DummyBehavior_GetResult_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DummyBehavior_GetResult_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__DummyBehavior_GetResult_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DummyBehavior_GetResult_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DummyBehavior_GetResult_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/DummyBehavior_GetResult_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DummyBehavior_GetResult_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowWaypoints_SendGoal_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__FollowWaypoints_SendGoal_Request__init(msg: *mut FollowWaypoints_SendGoal_Request) -> bool;
    fn nav2_msgs__action__FollowWaypoints_SendGoal_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FollowWaypoints_SendGoal_Request>, size: usize) -> bool;
    fn nav2_msgs__action__FollowWaypoints_SendGoal_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FollowWaypoints_SendGoal_Request>);
    fn nav2_msgs__action__FollowWaypoints_SendGoal_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FollowWaypoints_SendGoal_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<FollowWaypoints_SendGoal_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__FollowWaypoints_SendGoal_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowWaypoints_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::super::action::rmw::FollowWaypoints_Goal,

}



impl Default for FollowWaypoints_SendGoal_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__FollowWaypoints_SendGoal_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__FollowWaypoints_SendGoal_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FollowWaypoints_SendGoal_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowWaypoints_SendGoal_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowWaypoints_SendGoal_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowWaypoints_SendGoal_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FollowWaypoints_SendGoal_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FollowWaypoints_SendGoal_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/FollowWaypoints_SendGoal_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowWaypoints_SendGoal_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowWaypoints_SendGoal_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__FollowWaypoints_SendGoal_Response__init(msg: *mut FollowWaypoints_SendGoal_Response) -> bool;
    fn nav2_msgs__action__FollowWaypoints_SendGoal_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FollowWaypoints_SendGoal_Response>, size: usize) -> bool;
    fn nav2_msgs__action__FollowWaypoints_SendGoal_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FollowWaypoints_SendGoal_Response>);
    fn nav2_msgs__action__FollowWaypoints_SendGoal_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FollowWaypoints_SendGoal_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<FollowWaypoints_SendGoal_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__FollowWaypoints_SendGoal_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowWaypoints_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,

}



impl Default for FollowWaypoints_SendGoal_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__FollowWaypoints_SendGoal_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__FollowWaypoints_SendGoal_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FollowWaypoints_SendGoal_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowWaypoints_SendGoal_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowWaypoints_SendGoal_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowWaypoints_SendGoal_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FollowWaypoints_SendGoal_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FollowWaypoints_SendGoal_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/FollowWaypoints_SendGoal_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowWaypoints_SendGoal_Response() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowWaypoints_GetResult_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__FollowWaypoints_GetResult_Request__init(msg: *mut FollowWaypoints_GetResult_Request) -> bool;
    fn nav2_msgs__action__FollowWaypoints_GetResult_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FollowWaypoints_GetResult_Request>, size: usize) -> bool;
    fn nav2_msgs__action__FollowWaypoints_GetResult_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FollowWaypoints_GetResult_Request>);
    fn nav2_msgs__action__FollowWaypoints_GetResult_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FollowWaypoints_GetResult_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<FollowWaypoints_GetResult_Request>) -> bool;
}

// Corresponds to nav2_msgs__action__FollowWaypoints_GetResult_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowWaypoints_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,

}



impl Default for FollowWaypoints_GetResult_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__FollowWaypoints_GetResult_Request__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__FollowWaypoints_GetResult_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FollowWaypoints_GetResult_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowWaypoints_GetResult_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowWaypoints_GetResult_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowWaypoints_GetResult_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FollowWaypoints_GetResult_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FollowWaypoints_GetResult_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/FollowWaypoints_GetResult_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowWaypoints_GetResult_Request() }
  }
}


#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowWaypoints_GetResult_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav2_msgs__rosidl_generator_c")]
extern "C" {
    fn nav2_msgs__action__FollowWaypoints_GetResult_Response__init(msg: *mut FollowWaypoints_GetResult_Response) -> bool;
    fn nav2_msgs__action__FollowWaypoints_GetResult_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FollowWaypoints_GetResult_Response>, size: usize) -> bool;
    fn nav2_msgs__action__FollowWaypoints_GetResult_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FollowWaypoints_GetResult_Response>);
    fn nav2_msgs__action__FollowWaypoints_GetResult_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FollowWaypoints_GetResult_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<FollowWaypoints_GetResult_Response>) -> bool;
}

// Corresponds to nav2_msgs__action__FollowWaypoints_GetResult_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowWaypoints_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::super::action::rmw::FollowWaypoints_Result,

}



impl Default for FollowWaypoints_GetResult_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav2_msgs__action__FollowWaypoints_GetResult_Response__init(&mut msg as *mut _) {
        panic!("Call to nav2_msgs__action__FollowWaypoints_GetResult_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FollowWaypoints_GetResult_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowWaypoints_GetResult_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowWaypoints_GetResult_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav2_msgs__action__FollowWaypoints_GetResult_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FollowWaypoints_GetResult_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FollowWaypoints_GetResult_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav2_msgs/action/FollowWaypoints_GetResult_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowWaypoints_GetResult_Response() }
  }
}






#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__AssistedTeleop_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__AssistedTeleop_SendGoal
#[allow(missing_docs, non_camel_case_types)]
pub struct AssistedTeleop_SendGoal;

impl rosidl_runtime_rs::Service for AssistedTeleop_SendGoal {
    type Request = AssistedTeleop_SendGoal_Request;
    type Response = AssistedTeleop_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__AssistedTeleop_SendGoal() }
    }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__AssistedTeleop_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__AssistedTeleop_GetResult
#[allow(missing_docs, non_camel_case_types)]
pub struct AssistedTeleop_GetResult;

impl rosidl_runtime_rs::Service for AssistedTeleop_GetResult {
    type Request = AssistedTeleop_GetResult_Request;
    type Response = AssistedTeleop_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__AssistedTeleop_GetResult() }
    }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__BackUp_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__BackUp_SendGoal
#[allow(missing_docs, non_camel_case_types)]
pub struct BackUp_SendGoal;

impl rosidl_runtime_rs::Service for BackUp_SendGoal {
    type Request = BackUp_SendGoal_Request;
    type Response = BackUp_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__BackUp_SendGoal() }
    }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__BackUp_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__BackUp_GetResult
#[allow(missing_docs, non_camel_case_types)]
pub struct BackUp_GetResult;

impl rosidl_runtime_rs::Service for BackUp_GetResult {
    type Request = BackUp_GetResult_Request;
    type Response = BackUp_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__BackUp_GetResult() }
    }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__ComputePathToPose_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__ComputePathToPose_SendGoal
#[allow(missing_docs, non_camel_case_types)]
pub struct ComputePathToPose_SendGoal;

impl rosidl_runtime_rs::Service for ComputePathToPose_SendGoal {
    type Request = ComputePathToPose_SendGoal_Request;
    type Response = ComputePathToPose_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__ComputePathToPose_SendGoal() }
    }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__ComputePathToPose_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__ComputePathToPose_GetResult
#[allow(missing_docs, non_camel_case_types)]
pub struct ComputePathToPose_GetResult;

impl rosidl_runtime_rs::Service for ComputePathToPose_GetResult {
    type Request = ComputePathToPose_GetResult_Request;
    type Response = ComputePathToPose_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__ComputePathToPose_GetResult() }
    }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__ComputePathThroughPoses_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__ComputePathThroughPoses_SendGoal
#[allow(missing_docs, non_camel_case_types)]
pub struct ComputePathThroughPoses_SendGoal;

impl rosidl_runtime_rs::Service for ComputePathThroughPoses_SendGoal {
    type Request = ComputePathThroughPoses_SendGoal_Request;
    type Response = ComputePathThroughPoses_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__ComputePathThroughPoses_SendGoal() }
    }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__ComputePathThroughPoses_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__ComputePathThroughPoses_GetResult
#[allow(missing_docs, non_camel_case_types)]
pub struct ComputePathThroughPoses_GetResult;

impl rosidl_runtime_rs::Service for ComputePathThroughPoses_GetResult {
    type Request = ComputePathThroughPoses_GetResult_Request;
    type Response = ComputePathThroughPoses_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__ComputePathThroughPoses_GetResult() }
    }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__ComputeRoute_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__ComputeRoute_SendGoal
#[allow(missing_docs, non_camel_case_types)]
pub struct ComputeRoute_SendGoal;

impl rosidl_runtime_rs::Service for ComputeRoute_SendGoal {
    type Request = ComputeRoute_SendGoal_Request;
    type Response = ComputeRoute_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__ComputeRoute_SendGoal() }
    }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__ComputeRoute_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__ComputeRoute_GetResult
#[allow(missing_docs, non_camel_case_types)]
pub struct ComputeRoute_GetResult;

impl rosidl_runtime_rs::Service for ComputeRoute_GetResult {
    type Request = ComputeRoute_GetResult_Request;
    type Response = ComputeRoute_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__ComputeRoute_GetResult() }
    }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__ComputeAndTrackRoute_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__ComputeAndTrackRoute_SendGoal
#[allow(missing_docs, non_camel_case_types)]
pub struct ComputeAndTrackRoute_SendGoal;

impl rosidl_runtime_rs::Service for ComputeAndTrackRoute_SendGoal {
    type Request = ComputeAndTrackRoute_SendGoal_Request;
    type Response = ComputeAndTrackRoute_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__ComputeAndTrackRoute_SendGoal() }
    }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__ComputeAndTrackRoute_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__ComputeAndTrackRoute_GetResult
#[allow(missing_docs, non_camel_case_types)]
pub struct ComputeAndTrackRoute_GetResult;

impl rosidl_runtime_rs::Service for ComputeAndTrackRoute_GetResult {
    type Request = ComputeAndTrackRoute_GetResult_Request;
    type Response = ComputeAndTrackRoute_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__ComputeAndTrackRoute_GetResult() }
    }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__DriveOnHeading_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__DriveOnHeading_SendGoal
#[allow(missing_docs, non_camel_case_types)]
pub struct DriveOnHeading_SendGoal;

impl rosidl_runtime_rs::Service for DriveOnHeading_SendGoal {
    type Request = DriveOnHeading_SendGoal_Request;
    type Response = DriveOnHeading_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__DriveOnHeading_SendGoal() }
    }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__DriveOnHeading_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__DriveOnHeading_GetResult
#[allow(missing_docs, non_camel_case_types)]
pub struct DriveOnHeading_GetResult;

impl rosidl_runtime_rs::Service for DriveOnHeading_GetResult {
    type Request = DriveOnHeading_GetResult_Request;
    type Response = DriveOnHeading_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__DriveOnHeading_GetResult() }
    }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__SmoothPath_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__SmoothPath_SendGoal
#[allow(missing_docs, non_camel_case_types)]
pub struct SmoothPath_SendGoal;

impl rosidl_runtime_rs::Service for SmoothPath_SendGoal {
    type Request = SmoothPath_SendGoal_Request;
    type Response = SmoothPath_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__SmoothPath_SendGoal() }
    }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__SmoothPath_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__SmoothPath_GetResult
#[allow(missing_docs, non_camel_case_types)]
pub struct SmoothPath_GetResult;

impl rosidl_runtime_rs::Service for SmoothPath_GetResult {
    type Request = SmoothPath_GetResult_Request;
    type Response = SmoothPath_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__SmoothPath_GetResult() }
    }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__FollowPath_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__FollowPath_SendGoal
#[allow(missing_docs, non_camel_case_types)]
pub struct FollowPath_SendGoal;

impl rosidl_runtime_rs::Service for FollowPath_SendGoal {
    type Request = FollowPath_SendGoal_Request;
    type Response = FollowPath_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__FollowPath_SendGoal() }
    }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__FollowPath_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__FollowPath_GetResult
#[allow(missing_docs, non_camel_case_types)]
pub struct FollowPath_GetResult;

impl rosidl_runtime_rs::Service for FollowPath_GetResult {
    type Request = FollowPath_GetResult_Request;
    type Response = FollowPath_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__FollowPath_GetResult() }
    }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__NavigateToPose_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__NavigateToPose_SendGoal
#[allow(missing_docs, non_camel_case_types)]
pub struct NavigateToPose_SendGoal;

impl rosidl_runtime_rs::Service for NavigateToPose_SendGoal {
    type Request = NavigateToPose_SendGoal_Request;
    type Response = NavigateToPose_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__NavigateToPose_SendGoal() }
    }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__NavigateToPose_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__NavigateToPose_GetResult
#[allow(missing_docs, non_camel_case_types)]
pub struct NavigateToPose_GetResult;

impl rosidl_runtime_rs::Service for NavigateToPose_GetResult {
    type Request = NavigateToPose_GetResult_Request;
    type Response = NavigateToPose_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__NavigateToPose_GetResult() }
    }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__NavigateThroughPoses_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__NavigateThroughPoses_SendGoal
#[allow(missing_docs, non_camel_case_types)]
pub struct NavigateThroughPoses_SendGoal;

impl rosidl_runtime_rs::Service for NavigateThroughPoses_SendGoal {
    type Request = NavigateThroughPoses_SendGoal_Request;
    type Response = NavigateThroughPoses_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__NavigateThroughPoses_SendGoal() }
    }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__NavigateThroughPoses_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__NavigateThroughPoses_GetResult
#[allow(missing_docs, non_camel_case_types)]
pub struct NavigateThroughPoses_GetResult;

impl rosidl_runtime_rs::Service for NavigateThroughPoses_GetResult {
    type Request = NavigateThroughPoses_GetResult_Request;
    type Response = NavigateThroughPoses_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__NavigateThroughPoses_GetResult() }
    }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__Wait_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__Wait_SendGoal
#[allow(missing_docs, non_camel_case_types)]
pub struct Wait_SendGoal;

impl rosidl_runtime_rs::Service for Wait_SendGoal {
    type Request = Wait_SendGoal_Request;
    type Response = Wait_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__Wait_SendGoal() }
    }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__Wait_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__Wait_GetResult
#[allow(missing_docs, non_camel_case_types)]
pub struct Wait_GetResult;

impl rosidl_runtime_rs::Service for Wait_GetResult {
    type Request = Wait_GetResult_Request;
    type Response = Wait_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__Wait_GetResult() }
    }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__Spin_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__Spin_SendGoal
#[allow(missing_docs, non_camel_case_types)]
pub struct Spin_SendGoal;

impl rosidl_runtime_rs::Service for Spin_SendGoal {
    type Request = Spin_SendGoal_Request;
    type Response = Spin_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__Spin_SendGoal() }
    }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__Spin_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__Spin_GetResult
#[allow(missing_docs, non_camel_case_types)]
pub struct Spin_GetResult;

impl rosidl_runtime_rs::Service for Spin_GetResult {
    type Request = Spin_GetResult_Request;
    type Response = Spin_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__Spin_GetResult() }
    }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__DummyBehavior_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__DummyBehavior_SendGoal
#[allow(missing_docs, non_camel_case_types)]
pub struct DummyBehavior_SendGoal;

impl rosidl_runtime_rs::Service for DummyBehavior_SendGoal {
    type Request = DummyBehavior_SendGoal_Request;
    type Response = DummyBehavior_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__DummyBehavior_SendGoal() }
    }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__DummyBehavior_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__DummyBehavior_GetResult
#[allow(missing_docs, non_camel_case_types)]
pub struct DummyBehavior_GetResult;

impl rosidl_runtime_rs::Service for DummyBehavior_GetResult {
    type Request = DummyBehavior_GetResult_Request;
    type Response = DummyBehavior_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__DummyBehavior_GetResult() }
    }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__FollowWaypoints_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__FollowWaypoints_SendGoal
#[allow(missing_docs, non_camel_case_types)]
pub struct FollowWaypoints_SendGoal;

impl rosidl_runtime_rs::Service for FollowWaypoints_SendGoal {
    type Request = FollowWaypoints_SendGoal_Request;
    type Response = FollowWaypoints_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__FollowWaypoints_SendGoal() }
    }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__FollowWaypoints_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__FollowWaypoints_GetResult
#[allow(missing_docs, non_camel_case_types)]
pub struct FollowWaypoints_GetResult;

impl rosidl_runtime_rs::Service for FollowWaypoints_GetResult {
    type Request = FollowWaypoints_GetResult_Request;
    type Response = FollowWaypoints_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__FollowWaypoints_GetResult() }
    }
}


