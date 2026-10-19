
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to nav2_msgs__action__AssistedTeleop_Goal

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AssistedTeleop_Goal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub time_allowance: builtin_interfaces::msg::Duration,

}



impl Default for AssistedTeleop_Goal {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::AssistedTeleop_Goal::default())
  }
}

impl rosidl_runtime_rs::Message for AssistedTeleop_Goal {
  type RmwMsg = super::action::rmw::AssistedTeleop_Goal;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        time_allowance: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Owned(msg.time_allowance)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        time_allowance: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Borrowed(&msg.time_allowance)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      time_allowance: builtin_interfaces::msg::Duration::from_rmw_message(msg.time_allowance),
    }
  }
}


// Corresponds to nav2_msgs__action__AssistedTeleop_Result

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AssistedTeleop_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub total_elapsed_time: builtin_interfaces::msg::Duration,

}



impl Default for AssistedTeleop_Result {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::AssistedTeleop_Result::default())
  }
}

impl rosidl_runtime_rs::Message for AssistedTeleop_Result {
  type RmwMsg = super::action::rmw::AssistedTeleop_Result;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        total_elapsed_time: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Owned(msg.total_elapsed_time)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        total_elapsed_time: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Borrowed(&msg.total_elapsed_time)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      total_elapsed_time: builtin_interfaces::msg::Duration::from_rmw_message(msg.total_elapsed_time),
    }
  }
}


// Corresponds to nav2_msgs__action__AssistedTeleop_Feedback

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AssistedTeleop_Feedback {

    // This member is not documented.
    #[allow(missing_docs)]
    pub current_teleop_duration: builtin_interfaces::msg::Duration,

}



impl Default for AssistedTeleop_Feedback {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::AssistedTeleop_Feedback::default())
  }
}

impl rosidl_runtime_rs::Message for AssistedTeleop_Feedback {
  type RmwMsg = super::action::rmw::AssistedTeleop_Feedback;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        current_teleop_duration: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Owned(msg.current_teleop_duration)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        current_teleop_duration: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Borrowed(&msg.current_teleop_duration)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      current_teleop_duration: builtin_interfaces::msg::Duration::from_rmw_message(msg.current_teleop_duration),
    }
  }
}


// Corresponds to nav2_msgs__action__AssistedTeleop_FeedbackMessage

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AssistedTeleop_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::action::AssistedTeleop_Feedback,

}



impl Default for AssistedTeleop_FeedbackMessage {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::AssistedTeleop_FeedbackMessage::default())
  }
}

impl rosidl_runtime_rs::Message for AssistedTeleop_FeedbackMessage {
  type RmwMsg = super::action::rmw::AssistedTeleop_FeedbackMessage;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        feedback: super::action::AssistedTeleop_Feedback::into_rmw_message(std::borrow::Cow::Owned(msg.feedback)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        feedback: super::action::AssistedTeleop_Feedback::into_rmw_message(std::borrow::Cow::Borrowed(&msg.feedback)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      feedback: super::action::AssistedTeleop_Feedback::from_rmw_message(msg.feedback),
    }
  }
}


// Corresponds to nav2_msgs__action__BackUp_Goal

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BackUp_Goal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub target: geometry_msgs::msg::Point,


    // This member is not documented.
    #[allow(missing_docs)]
    pub speed: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub time_allowance: builtin_interfaces::msg::Duration,

}



impl Default for BackUp_Goal {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::BackUp_Goal::default())
  }
}

impl rosidl_runtime_rs::Message for BackUp_Goal {
  type RmwMsg = super::action::rmw::BackUp_Goal;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        target: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Owned(msg.target)).into_owned(),
        speed: msg.speed,
        time_allowance: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Owned(msg.time_allowance)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        target: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Borrowed(&msg.target)).into_owned(),
      speed: msg.speed,
        time_allowance: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Borrowed(&msg.time_allowance)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      target: geometry_msgs::msg::Point::from_rmw_message(msg.target),
      speed: msg.speed,
      time_allowance: builtin_interfaces::msg::Duration::from_rmw_message(msg.time_allowance),
    }
  }
}


// Corresponds to nav2_msgs__action__BackUp_Result

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BackUp_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub total_elapsed_time: builtin_interfaces::msg::Duration,

}



impl Default for BackUp_Result {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::BackUp_Result::default())
  }
}

impl rosidl_runtime_rs::Message for BackUp_Result {
  type RmwMsg = super::action::rmw::BackUp_Result;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        total_elapsed_time: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Owned(msg.total_elapsed_time)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        total_elapsed_time: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Borrowed(&msg.total_elapsed_time)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      total_elapsed_time: builtin_interfaces::msg::Duration::from_rmw_message(msg.total_elapsed_time),
    }
  }
}


// Corresponds to nav2_msgs__action__BackUp_Feedback

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BackUp_Feedback {

    // This member is not documented.
    #[allow(missing_docs)]
    pub distance_traveled: f32,

}



impl Default for BackUp_Feedback {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::BackUp_Feedback::default())
  }
}

impl rosidl_runtime_rs::Message for BackUp_Feedback {
  type RmwMsg = super::action::rmw::BackUp_Feedback;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        distance_traveled: msg.distance_traveled,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      distance_traveled: msg.distance_traveled,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      distance_traveled: msg.distance_traveled,
    }
  }
}


// Corresponds to nav2_msgs__action__BackUp_FeedbackMessage

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BackUp_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::action::BackUp_Feedback,

}



impl Default for BackUp_FeedbackMessage {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::BackUp_FeedbackMessage::default())
  }
}

impl rosidl_runtime_rs::Message for BackUp_FeedbackMessage {
  type RmwMsg = super::action::rmw::BackUp_FeedbackMessage;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        feedback: super::action::BackUp_Feedback::into_rmw_message(std::borrow::Cow::Owned(msg.feedback)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        feedback: super::action::BackUp_Feedback::into_rmw_message(std::borrow::Cow::Borrowed(&msg.feedback)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      feedback: super::action::BackUp_Feedback::from_rmw_message(msg.feedback),
    }
  }
}


// Corresponds to nav2_msgs__action__ComputePathToPose_Goal

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathToPose_Goal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: geometry_msgs::msg::PoseStamped,


    // This member is not documented.
    #[allow(missing_docs)]
    pub start: geometry_msgs::msg::PoseStamped,


    // This member is not documented.
    #[allow(missing_docs)]
    pub planner_id: std::string::String,

    /// If false, use current robot pose as path start, if true, use start above instead
    pub use_start: bool,

}



impl Default for ComputePathToPose_Goal {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::ComputePathToPose_Goal::default())
  }
}

impl rosidl_runtime_rs::Message for ComputePathToPose_Goal {
  type RmwMsg = super::action::rmw::ComputePathToPose_Goal;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Owned(msg.goal)).into_owned(),
        start: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Owned(msg.start)).into_owned(),
        planner_id: msg.planner_id.as_str().into(),
        use_start: msg.use_start,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal)).into_owned(),
        start: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Borrowed(&msg.start)).into_owned(),
        planner_id: msg.planner_id.as_str().into(),
      use_start: msg.use_start,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal: geometry_msgs::msg::PoseStamped::from_rmw_message(msg.goal),
      start: geometry_msgs::msg::PoseStamped::from_rmw_message(msg.start),
      planner_id: msg.planner_id.to_string(),
      use_start: msg.use_start,
    }
  }
}


// Corresponds to nav2_msgs__action__ComputePathToPose_Result

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathToPose_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub path: nav_msgs::msg::Path,


    // This member is not documented.
    #[allow(missing_docs)]
    pub planning_time: builtin_interfaces::msg::Duration,

}



impl Default for ComputePathToPose_Result {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::ComputePathToPose_Result::default())
  }
}

impl rosidl_runtime_rs::Message for ComputePathToPose_Result {
  type RmwMsg = super::action::rmw::ComputePathToPose_Result;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        path: nav_msgs::msg::Path::into_rmw_message(std::borrow::Cow::Owned(msg.path)).into_owned(),
        planning_time: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Owned(msg.planning_time)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        path: nav_msgs::msg::Path::into_rmw_message(std::borrow::Cow::Borrowed(&msg.path)).into_owned(),
        planning_time: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Borrowed(&msg.planning_time)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      path: nav_msgs::msg::Path::from_rmw_message(msg.path),
      planning_time: builtin_interfaces::msg::Duration::from_rmw_message(msg.planning_time),
    }
  }
}


// Corresponds to nav2_msgs__action__ComputePathToPose_Feedback

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathToPose_Feedback {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for ComputePathToPose_Feedback {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::ComputePathToPose_Feedback::default())
  }
}

impl rosidl_runtime_rs::Message for ComputePathToPose_Feedback {
  type RmwMsg = super::action::rmw::ComputePathToPose_Feedback;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
    }
  }
}


// Corresponds to nav2_msgs__action__ComputePathToPose_FeedbackMessage

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathToPose_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::action::ComputePathToPose_Feedback,

}



impl Default for ComputePathToPose_FeedbackMessage {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::ComputePathToPose_FeedbackMessage::default())
  }
}

impl rosidl_runtime_rs::Message for ComputePathToPose_FeedbackMessage {
  type RmwMsg = super::action::rmw::ComputePathToPose_FeedbackMessage;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        feedback: super::action::ComputePathToPose_Feedback::into_rmw_message(std::borrow::Cow::Owned(msg.feedback)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        feedback: super::action::ComputePathToPose_Feedback::into_rmw_message(std::borrow::Cow::Borrowed(&msg.feedback)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      feedback: super::action::ComputePathToPose_Feedback::from_rmw_message(msg.feedback),
    }
  }
}


// Corresponds to nav2_msgs__action__ComputePathThroughPoses_Goal

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathThroughPoses_Goal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goals: Vec<geometry_msgs::msg::PoseStamped>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub start: geometry_msgs::msg::PoseStamped,


    // This member is not documented.
    #[allow(missing_docs)]
    pub planner_id: std::string::String,

    /// If false, use current robot pose as path start, if true, use start above instead
    pub use_start: bool,

}



impl Default for ComputePathThroughPoses_Goal {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::ComputePathThroughPoses_Goal::default())
  }
}

impl rosidl_runtime_rs::Message for ComputePathThroughPoses_Goal {
  type RmwMsg = super::action::rmw::ComputePathThroughPoses_Goal;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goals: msg.goals
          .into_iter()
          .map(|elem| geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
        start: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Owned(msg.start)).into_owned(),
        planner_id: msg.planner_id.as_str().into(),
        use_start: msg.use_start,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goals: msg.goals
          .iter()
          .map(|elem| geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
        start: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Borrowed(&msg.start)).into_owned(),
        planner_id: msg.planner_id.as_str().into(),
      use_start: msg.use_start,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goals: msg.goals
          .into_iter()
          .map(geometry_msgs::msg::PoseStamped::from_rmw_message)
          .collect(),
      start: geometry_msgs::msg::PoseStamped::from_rmw_message(msg.start),
      planner_id: msg.planner_id.to_string(),
      use_start: msg.use_start,
    }
  }
}


// Corresponds to nav2_msgs__action__ComputePathThroughPoses_Result

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathThroughPoses_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub path: nav_msgs::msg::Path,


    // This member is not documented.
    #[allow(missing_docs)]
    pub planning_time: builtin_interfaces::msg::Duration,

}



impl Default for ComputePathThroughPoses_Result {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::ComputePathThroughPoses_Result::default())
  }
}

impl rosidl_runtime_rs::Message for ComputePathThroughPoses_Result {
  type RmwMsg = super::action::rmw::ComputePathThroughPoses_Result;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        path: nav_msgs::msg::Path::into_rmw_message(std::borrow::Cow::Owned(msg.path)).into_owned(),
        planning_time: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Owned(msg.planning_time)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        path: nav_msgs::msg::Path::into_rmw_message(std::borrow::Cow::Borrowed(&msg.path)).into_owned(),
        planning_time: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Borrowed(&msg.planning_time)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      path: nav_msgs::msg::Path::from_rmw_message(msg.path),
      planning_time: builtin_interfaces::msg::Duration::from_rmw_message(msg.planning_time),
    }
  }
}


// Corresponds to nav2_msgs__action__ComputePathThroughPoses_Feedback

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathThroughPoses_Feedback {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for ComputePathThroughPoses_Feedback {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::ComputePathThroughPoses_Feedback::default())
  }
}

impl rosidl_runtime_rs::Message for ComputePathThroughPoses_Feedback {
  type RmwMsg = super::action::rmw::ComputePathThroughPoses_Feedback;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
    }
  }
}


// Corresponds to nav2_msgs__action__ComputePathThroughPoses_FeedbackMessage

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathThroughPoses_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::action::ComputePathThroughPoses_Feedback,

}



impl Default for ComputePathThroughPoses_FeedbackMessage {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::ComputePathThroughPoses_FeedbackMessage::default())
  }
}

impl rosidl_runtime_rs::Message for ComputePathThroughPoses_FeedbackMessage {
  type RmwMsg = super::action::rmw::ComputePathThroughPoses_FeedbackMessage;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        feedback: super::action::ComputePathThroughPoses_Feedback::into_rmw_message(std::borrow::Cow::Owned(msg.feedback)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        feedback: super::action::ComputePathThroughPoses_Feedback::into_rmw_message(std::borrow::Cow::Borrowed(&msg.feedback)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      feedback: super::action::ComputePathThroughPoses_Feedback::from_rmw_message(msg.feedback),
    }
  }
}


// Corresponds to nav2_msgs__action__ComputeRoute_Goal

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputeRoute_Goal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub start_id: u16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub start: geometry_msgs::msg::PoseStamped,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: u16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: geometry_msgs::msg::PoseStamped,

    /// Whether to use the start field or find the start pose in TF
    pub use_start: bool,

    /// Whether to use the poses or the IDs fields for request
    pub use_poses: bool,

}



impl Default for ComputeRoute_Goal {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::ComputeRoute_Goal::default())
  }
}

impl rosidl_runtime_rs::Message for ComputeRoute_Goal {
  type RmwMsg = super::action::rmw::ComputeRoute_Goal;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        start_id: msg.start_id,
        start: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Owned(msg.start)).into_owned(),
        goal_id: msg.goal_id,
        goal: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Owned(msg.goal)).into_owned(),
        use_start: msg.use_start,
        use_poses: msg.use_poses,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      start_id: msg.start_id,
        start: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Borrowed(&msg.start)).into_owned(),
      goal_id: msg.goal_id,
        goal: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal)).into_owned(),
      use_start: msg.use_start,
      use_poses: msg.use_poses,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      start_id: msg.start_id,
      start: geometry_msgs::msg::PoseStamped::from_rmw_message(msg.start),
      goal_id: msg.goal_id,
      goal: geometry_msgs::msg::PoseStamped::from_rmw_message(msg.goal),
      use_start: msg.use_start,
      use_poses: msg.use_poses,
    }
  }
}


// Corresponds to nav2_msgs__action__ComputeRoute_Result

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputeRoute_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub planning_time: builtin_interfaces::msg::Duration,


    // This member is not documented.
    #[allow(missing_docs)]
    pub path: nav_msgs::msg::Path,


    // This member is not documented.
    #[allow(missing_docs)]
    pub route: super::msg::Route,

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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::ComputeRoute_Result::default())
  }
}

impl rosidl_runtime_rs::Message for ComputeRoute_Result {
  type RmwMsg = super::action::rmw::ComputeRoute_Result;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        planning_time: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Owned(msg.planning_time)).into_owned(),
        path: nav_msgs::msg::Path::into_rmw_message(std::borrow::Cow::Owned(msg.path)).into_owned(),
        route: super::msg::Route::into_rmw_message(std::borrow::Cow::Owned(msg.route)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        planning_time: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Borrowed(&msg.planning_time)).into_owned(),
        path: nav_msgs::msg::Path::into_rmw_message(std::borrow::Cow::Borrowed(&msg.path)).into_owned(),
        route: super::msg::Route::into_rmw_message(std::borrow::Cow::Borrowed(&msg.route)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      planning_time: builtin_interfaces::msg::Duration::from_rmw_message(msg.planning_time),
      path: nav_msgs::msg::Path::from_rmw_message(msg.path),
      route: super::msg::Route::from_rmw_message(msg.route),
    }
  }
}


// Corresponds to nav2_msgs__action__ComputeRoute_Feedback

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputeRoute_Feedback {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for ComputeRoute_Feedback {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::ComputeRoute_Feedback::default())
  }
}

impl rosidl_runtime_rs::Message for ComputeRoute_Feedback {
  type RmwMsg = super::action::rmw::ComputeRoute_Feedback;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
    }
  }
}


// Corresponds to nav2_msgs__action__ComputeRoute_FeedbackMessage

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputeRoute_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::action::ComputeRoute_Feedback,

}



impl Default for ComputeRoute_FeedbackMessage {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::ComputeRoute_FeedbackMessage::default())
  }
}

impl rosidl_runtime_rs::Message for ComputeRoute_FeedbackMessage {
  type RmwMsg = super::action::rmw::ComputeRoute_FeedbackMessage;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        feedback: super::action::ComputeRoute_Feedback::into_rmw_message(std::borrow::Cow::Owned(msg.feedback)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        feedback: super::action::ComputeRoute_Feedback::into_rmw_message(std::borrow::Cow::Borrowed(&msg.feedback)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      feedback: super::action::ComputeRoute_Feedback::from_rmw_message(msg.feedback),
    }
  }
}


// Corresponds to nav2_msgs__action__ComputeAndTrackRoute_Goal

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputeAndTrackRoute_Goal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub start_id: u16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub start: geometry_msgs::msg::PoseStamped,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: u16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: geometry_msgs::msg::PoseStamped,

    /// Whether to use the start field or find the start pose in TF
    pub use_start: bool,

    /// Whether to use the poses or the IDs fields for request
    pub use_poses: bool,

}



impl Default for ComputeAndTrackRoute_Goal {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::ComputeAndTrackRoute_Goal::default())
  }
}

impl rosidl_runtime_rs::Message for ComputeAndTrackRoute_Goal {
  type RmwMsg = super::action::rmw::ComputeAndTrackRoute_Goal;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        start_id: msg.start_id,
        start: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Owned(msg.start)).into_owned(),
        goal_id: msg.goal_id,
        goal: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Owned(msg.goal)).into_owned(),
        use_start: msg.use_start,
        use_poses: msg.use_poses,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      start_id: msg.start_id,
        start: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Borrowed(&msg.start)).into_owned(),
      goal_id: msg.goal_id,
        goal: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal)).into_owned(),
      use_start: msg.use_start,
      use_poses: msg.use_poses,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      start_id: msg.start_id,
      start: geometry_msgs::msg::PoseStamped::from_rmw_message(msg.start),
      goal_id: msg.goal_id,
      goal: geometry_msgs::msg::PoseStamped::from_rmw_message(msg.goal),
      use_start: msg.use_start,
      use_poses: msg.use_poses,
    }
  }
}


// Corresponds to nav2_msgs__action__ComputeAndTrackRoute_Result

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputeAndTrackRoute_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub execution_duration: builtin_interfaces::msg::Duration,

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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::ComputeAndTrackRoute_Result::default())
  }
}

impl rosidl_runtime_rs::Message for ComputeAndTrackRoute_Result {
  type RmwMsg = super::action::rmw::ComputeAndTrackRoute_Result;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        execution_duration: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Owned(msg.execution_duration)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        execution_duration: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Borrowed(&msg.execution_duration)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      execution_duration: builtin_interfaces::msg::Duration::from_rmw_message(msg.execution_duration),
    }
  }
}


// Corresponds to nav2_msgs__action__ComputeAndTrackRoute_Feedback

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    pub route: super::msg::Route,


    // This member is not documented.
    #[allow(missing_docs)]
    pub path: nav_msgs::msg::Path,


    // This member is not documented.
    #[allow(missing_docs)]
    pub operations_triggered: Vec<std::string::String>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub rerouted: bool,

}



impl Default for ComputeAndTrackRoute_Feedback {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::ComputeAndTrackRoute_Feedback::default())
  }
}

impl rosidl_runtime_rs::Message for ComputeAndTrackRoute_Feedback {
  type RmwMsg = super::action::rmw::ComputeAndTrackRoute_Feedback;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        last_node_id: msg.last_node_id,
        next_node_id: msg.next_node_id,
        current_edge_id: msg.current_edge_id,
        route: super::msg::Route::into_rmw_message(std::borrow::Cow::Owned(msg.route)).into_owned(),
        path: nav_msgs::msg::Path::into_rmw_message(std::borrow::Cow::Owned(msg.path)).into_owned(),
        operations_triggered: msg.operations_triggered
          .into_iter()
          .map(|elem| elem.as_str().into())
          .collect(),
        rerouted: msg.rerouted,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      last_node_id: msg.last_node_id,
      next_node_id: msg.next_node_id,
      current_edge_id: msg.current_edge_id,
        route: super::msg::Route::into_rmw_message(std::borrow::Cow::Borrowed(&msg.route)).into_owned(),
        path: nav_msgs::msg::Path::into_rmw_message(std::borrow::Cow::Borrowed(&msg.path)).into_owned(),
        operations_triggered: msg.operations_triggered
          .iter()
          .map(|elem| elem.as_str().into())
          .collect(),
      rerouted: msg.rerouted,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      last_node_id: msg.last_node_id,
      next_node_id: msg.next_node_id,
      current_edge_id: msg.current_edge_id,
      route: super::msg::Route::from_rmw_message(msg.route),
      path: nav_msgs::msg::Path::from_rmw_message(msg.path),
      operations_triggered: msg.operations_triggered
          .into_iter()
          .map(|elem| elem.to_string())
          .collect(),
      rerouted: msg.rerouted,
    }
  }
}


// Corresponds to nav2_msgs__action__ComputeAndTrackRoute_FeedbackMessage

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputeAndTrackRoute_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::action::ComputeAndTrackRoute_Feedback,

}



impl Default for ComputeAndTrackRoute_FeedbackMessage {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::ComputeAndTrackRoute_FeedbackMessage::default())
  }
}

impl rosidl_runtime_rs::Message for ComputeAndTrackRoute_FeedbackMessage {
  type RmwMsg = super::action::rmw::ComputeAndTrackRoute_FeedbackMessage;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        feedback: super::action::ComputeAndTrackRoute_Feedback::into_rmw_message(std::borrow::Cow::Owned(msg.feedback)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        feedback: super::action::ComputeAndTrackRoute_Feedback::into_rmw_message(std::borrow::Cow::Borrowed(&msg.feedback)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      feedback: super::action::ComputeAndTrackRoute_Feedback::from_rmw_message(msg.feedback),
    }
  }
}


// Corresponds to nav2_msgs__action__DriveOnHeading_Goal

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DriveOnHeading_Goal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub target: geometry_msgs::msg::Point,


    // This member is not documented.
    #[allow(missing_docs)]
    pub speed: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub time_allowance: builtin_interfaces::msg::Duration,

}



impl Default for DriveOnHeading_Goal {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::DriveOnHeading_Goal::default())
  }
}

impl rosidl_runtime_rs::Message for DriveOnHeading_Goal {
  type RmwMsg = super::action::rmw::DriveOnHeading_Goal;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        target: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Owned(msg.target)).into_owned(),
        speed: msg.speed,
        time_allowance: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Owned(msg.time_allowance)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        target: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Borrowed(&msg.target)).into_owned(),
      speed: msg.speed,
        time_allowance: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Borrowed(&msg.time_allowance)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      target: geometry_msgs::msg::Point::from_rmw_message(msg.target),
      speed: msg.speed,
      time_allowance: builtin_interfaces::msg::Duration::from_rmw_message(msg.time_allowance),
    }
  }
}


// Corresponds to nav2_msgs__action__DriveOnHeading_Result

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DriveOnHeading_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub total_elapsed_time: builtin_interfaces::msg::Duration,

}



impl Default for DriveOnHeading_Result {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::DriveOnHeading_Result::default())
  }
}

impl rosidl_runtime_rs::Message for DriveOnHeading_Result {
  type RmwMsg = super::action::rmw::DriveOnHeading_Result;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        total_elapsed_time: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Owned(msg.total_elapsed_time)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        total_elapsed_time: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Borrowed(&msg.total_elapsed_time)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      total_elapsed_time: builtin_interfaces::msg::Duration::from_rmw_message(msg.total_elapsed_time),
    }
  }
}


// Corresponds to nav2_msgs__action__DriveOnHeading_Feedback

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DriveOnHeading_Feedback {

    // This member is not documented.
    #[allow(missing_docs)]
    pub distance_traveled: f32,

}



impl Default for DriveOnHeading_Feedback {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::DriveOnHeading_Feedback::default())
  }
}

impl rosidl_runtime_rs::Message for DriveOnHeading_Feedback {
  type RmwMsg = super::action::rmw::DriveOnHeading_Feedback;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        distance_traveled: msg.distance_traveled,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      distance_traveled: msg.distance_traveled,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      distance_traveled: msg.distance_traveled,
    }
  }
}


// Corresponds to nav2_msgs__action__DriveOnHeading_FeedbackMessage

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DriveOnHeading_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::action::DriveOnHeading_Feedback,

}



impl Default for DriveOnHeading_FeedbackMessage {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::DriveOnHeading_FeedbackMessage::default())
  }
}

impl rosidl_runtime_rs::Message for DriveOnHeading_FeedbackMessage {
  type RmwMsg = super::action::rmw::DriveOnHeading_FeedbackMessage;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        feedback: super::action::DriveOnHeading_Feedback::into_rmw_message(std::borrow::Cow::Owned(msg.feedback)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        feedback: super::action::DriveOnHeading_Feedback::into_rmw_message(std::borrow::Cow::Borrowed(&msg.feedback)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      feedback: super::action::DriveOnHeading_Feedback::from_rmw_message(msg.feedback),
    }
  }
}


// Corresponds to nav2_msgs__action__SmoothPath_Goal

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SmoothPath_Goal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub path: nav_msgs::msg::Path,


    // This member is not documented.
    #[allow(missing_docs)]
    pub smoother_id: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub max_smoothing_duration: builtin_interfaces::msg::Duration,


    // This member is not documented.
    #[allow(missing_docs)]
    pub check_for_collisions: bool,

}



impl Default for SmoothPath_Goal {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::SmoothPath_Goal::default())
  }
}

impl rosidl_runtime_rs::Message for SmoothPath_Goal {
  type RmwMsg = super::action::rmw::SmoothPath_Goal;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        path: nav_msgs::msg::Path::into_rmw_message(std::borrow::Cow::Owned(msg.path)).into_owned(),
        smoother_id: msg.smoother_id.as_str().into(),
        max_smoothing_duration: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Owned(msg.max_smoothing_duration)).into_owned(),
        check_for_collisions: msg.check_for_collisions,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        path: nav_msgs::msg::Path::into_rmw_message(std::borrow::Cow::Borrowed(&msg.path)).into_owned(),
        smoother_id: msg.smoother_id.as_str().into(),
        max_smoothing_duration: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Borrowed(&msg.max_smoothing_duration)).into_owned(),
      check_for_collisions: msg.check_for_collisions,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      path: nav_msgs::msg::Path::from_rmw_message(msg.path),
      smoother_id: msg.smoother_id.to_string(),
      max_smoothing_duration: builtin_interfaces::msg::Duration::from_rmw_message(msg.max_smoothing_duration),
      check_for_collisions: msg.check_for_collisions,
    }
  }
}


// Corresponds to nav2_msgs__action__SmoothPath_Result

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SmoothPath_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub path: nav_msgs::msg::Path,


    // This member is not documented.
    #[allow(missing_docs)]
    pub smoothing_duration: builtin_interfaces::msg::Duration,


    // This member is not documented.
    #[allow(missing_docs)]
    pub was_completed: bool,

}



impl Default for SmoothPath_Result {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::SmoothPath_Result::default())
  }
}

impl rosidl_runtime_rs::Message for SmoothPath_Result {
  type RmwMsg = super::action::rmw::SmoothPath_Result;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        path: nav_msgs::msg::Path::into_rmw_message(std::borrow::Cow::Owned(msg.path)).into_owned(),
        smoothing_duration: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Owned(msg.smoothing_duration)).into_owned(),
        was_completed: msg.was_completed,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        path: nav_msgs::msg::Path::into_rmw_message(std::borrow::Cow::Borrowed(&msg.path)).into_owned(),
        smoothing_duration: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Borrowed(&msg.smoothing_duration)).into_owned(),
      was_completed: msg.was_completed,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      path: nav_msgs::msg::Path::from_rmw_message(msg.path),
      smoothing_duration: builtin_interfaces::msg::Duration::from_rmw_message(msg.smoothing_duration),
      was_completed: msg.was_completed,
    }
  }
}


// Corresponds to nav2_msgs__action__SmoothPath_Feedback

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SmoothPath_Feedback {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for SmoothPath_Feedback {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::SmoothPath_Feedback::default())
  }
}

impl rosidl_runtime_rs::Message for SmoothPath_Feedback {
  type RmwMsg = super::action::rmw::SmoothPath_Feedback;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
    }
  }
}


// Corresponds to nav2_msgs__action__SmoothPath_FeedbackMessage

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SmoothPath_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::action::SmoothPath_Feedback,

}



impl Default for SmoothPath_FeedbackMessage {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::SmoothPath_FeedbackMessage::default())
  }
}

impl rosidl_runtime_rs::Message for SmoothPath_FeedbackMessage {
  type RmwMsg = super::action::rmw::SmoothPath_FeedbackMessage;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        feedback: super::action::SmoothPath_Feedback::into_rmw_message(std::borrow::Cow::Owned(msg.feedback)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        feedback: super::action::SmoothPath_Feedback::into_rmw_message(std::borrow::Cow::Borrowed(&msg.feedback)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      feedback: super::action::SmoothPath_Feedback::from_rmw_message(msg.feedback),
    }
  }
}


// Corresponds to nav2_msgs__action__FollowPath_Goal

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowPath_Goal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub path: nav_msgs::msg::Path,


    // This member is not documented.
    #[allow(missing_docs)]
    pub controller_id: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_checker_id: std::string::String,

}



impl Default for FollowPath_Goal {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::FollowPath_Goal::default())
  }
}

impl rosidl_runtime_rs::Message for FollowPath_Goal {
  type RmwMsg = super::action::rmw::FollowPath_Goal;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        path: nav_msgs::msg::Path::into_rmw_message(std::borrow::Cow::Owned(msg.path)).into_owned(),
        controller_id: msg.controller_id.as_str().into(),
        goal_checker_id: msg.goal_checker_id.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        path: nav_msgs::msg::Path::into_rmw_message(std::borrow::Cow::Borrowed(&msg.path)).into_owned(),
        controller_id: msg.controller_id.as_str().into(),
        goal_checker_id: msg.goal_checker_id.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      path: nav_msgs::msg::Path::from_rmw_message(msg.path),
      controller_id: msg.controller_id.to_string(),
      goal_checker_id: msg.goal_checker_id.to_string(),
    }
  }
}


// Corresponds to nav2_msgs__action__FollowPath_Result

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowPath_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub result: std_msgs::msg::Empty,

}



impl Default for FollowPath_Result {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::FollowPath_Result::default())
  }
}

impl rosidl_runtime_rs::Message for FollowPath_Result {
  type RmwMsg = super::action::rmw::FollowPath_Result;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        result: std_msgs::msg::Empty::into_rmw_message(std::borrow::Cow::Owned(msg.result)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        result: std_msgs::msg::Empty::into_rmw_message(std::borrow::Cow::Borrowed(&msg.result)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      result: std_msgs::msg::Empty::from_rmw_message(msg.result),
    }
  }
}


// Corresponds to nav2_msgs__action__FollowPath_Feedback

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::FollowPath_Feedback::default())
  }
}

impl rosidl_runtime_rs::Message for FollowPath_Feedback {
  type RmwMsg = super::action::rmw::FollowPath_Feedback;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        distance_to_goal: msg.distance_to_goal,
        speed: msg.speed,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      distance_to_goal: msg.distance_to_goal,
      speed: msg.speed,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      distance_to_goal: msg.distance_to_goal,
      speed: msg.speed,
    }
  }
}


// Corresponds to nav2_msgs__action__FollowPath_FeedbackMessage

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowPath_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::action::FollowPath_Feedback,

}



impl Default for FollowPath_FeedbackMessage {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::FollowPath_FeedbackMessage::default())
  }
}

impl rosidl_runtime_rs::Message for FollowPath_FeedbackMessage {
  type RmwMsg = super::action::rmw::FollowPath_FeedbackMessage;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        feedback: super::action::FollowPath_Feedback::into_rmw_message(std::borrow::Cow::Owned(msg.feedback)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        feedback: super::action::FollowPath_Feedback::into_rmw_message(std::borrow::Cow::Borrowed(&msg.feedback)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      feedback: super::action::FollowPath_Feedback::from_rmw_message(msg.feedback),
    }
  }
}


// Corresponds to nav2_msgs__action__NavigateToPose_Goal

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateToPose_Goal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub pose: geometry_msgs::msg::PoseStamped,


    // This member is not documented.
    #[allow(missing_docs)]
    pub behavior_tree: std::string::String,

}



impl Default for NavigateToPose_Goal {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::NavigateToPose_Goal::default())
  }
}

impl rosidl_runtime_rs::Message for NavigateToPose_Goal {
  type RmwMsg = super::action::rmw::NavigateToPose_Goal;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        pose: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Owned(msg.pose)).into_owned(),
        behavior_tree: msg.behavior_tree.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        pose: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Borrowed(&msg.pose)).into_owned(),
        behavior_tree: msg.behavior_tree.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      pose: geometry_msgs::msg::PoseStamped::from_rmw_message(msg.pose),
      behavior_tree: msg.behavior_tree.to_string(),
    }
  }
}


// Corresponds to nav2_msgs__action__NavigateToPose_Result

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateToPose_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub result: std_msgs::msg::Empty,

}



impl Default for NavigateToPose_Result {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::NavigateToPose_Result::default())
  }
}

impl rosidl_runtime_rs::Message for NavigateToPose_Result {
  type RmwMsg = super::action::rmw::NavigateToPose_Result;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        result: std_msgs::msg::Empty::into_rmw_message(std::borrow::Cow::Owned(msg.result)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        result: std_msgs::msg::Empty::into_rmw_message(std::borrow::Cow::Borrowed(&msg.result)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      result: std_msgs::msg::Empty::from_rmw_message(msg.result),
    }
  }
}


// Corresponds to nav2_msgs__action__NavigateToPose_Feedback

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateToPose_Feedback {

    // This member is not documented.
    #[allow(missing_docs)]
    pub current_pose: geometry_msgs::msg::PoseStamped,


    // This member is not documented.
    #[allow(missing_docs)]
    pub navigation_time: builtin_interfaces::msg::Duration,


    // This member is not documented.
    #[allow(missing_docs)]
    pub estimated_time_remaining: builtin_interfaces::msg::Duration,


    // This member is not documented.
    #[allow(missing_docs)]
    pub number_of_recoveries: i16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub distance_remaining: f32,

}



impl Default for NavigateToPose_Feedback {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::NavigateToPose_Feedback::default())
  }
}

impl rosidl_runtime_rs::Message for NavigateToPose_Feedback {
  type RmwMsg = super::action::rmw::NavigateToPose_Feedback;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        current_pose: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Owned(msg.current_pose)).into_owned(),
        navigation_time: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Owned(msg.navigation_time)).into_owned(),
        estimated_time_remaining: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Owned(msg.estimated_time_remaining)).into_owned(),
        number_of_recoveries: msg.number_of_recoveries,
        distance_remaining: msg.distance_remaining,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        current_pose: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Borrowed(&msg.current_pose)).into_owned(),
        navigation_time: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Borrowed(&msg.navigation_time)).into_owned(),
        estimated_time_remaining: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Borrowed(&msg.estimated_time_remaining)).into_owned(),
      number_of_recoveries: msg.number_of_recoveries,
      distance_remaining: msg.distance_remaining,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      current_pose: geometry_msgs::msg::PoseStamped::from_rmw_message(msg.current_pose),
      navigation_time: builtin_interfaces::msg::Duration::from_rmw_message(msg.navigation_time),
      estimated_time_remaining: builtin_interfaces::msg::Duration::from_rmw_message(msg.estimated_time_remaining),
      number_of_recoveries: msg.number_of_recoveries,
      distance_remaining: msg.distance_remaining,
    }
  }
}


// Corresponds to nav2_msgs__action__NavigateToPose_FeedbackMessage

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateToPose_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::action::NavigateToPose_Feedback,

}



impl Default for NavigateToPose_FeedbackMessage {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::NavigateToPose_FeedbackMessage::default())
  }
}

impl rosidl_runtime_rs::Message for NavigateToPose_FeedbackMessage {
  type RmwMsg = super::action::rmw::NavigateToPose_FeedbackMessage;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        feedback: super::action::NavigateToPose_Feedback::into_rmw_message(std::borrow::Cow::Owned(msg.feedback)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        feedback: super::action::NavigateToPose_Feedback::into_rmw_message(std::borrow::Cow::Borrowed(&msg.feedback)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      feedback: super::action::NavigateToPose_Feedback::from_rmw_message(msg.feedback),
    }
  }
}


// Corresponds to nav2_msgs__action__NavigateThroughPoses_Goal

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateThroughPoses_Goal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub poses: Vec<geometry_msgs::msg::PoseStamped>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub behavior_tree: std::string::String,

}



impl Default for NavigateThroughPoses_Goal {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::NavigateThroughPoses_Goal::default())
  }
}

impl rosidl_runtime_rs::Message for NavigateThroughPoses_Goal {
  type RmwMsg = super::action::rmw::NavigateThroughPoses_Goal;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        poses: msg.poses
          .into_iter()
          .map(|elem| geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
        behavior_tree: msg.behavior_tree.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        poses: msg.poses
          .iter()
          .map(|elem| geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
        behavior_tree: msg.behavior_tree.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      poses: msg.poses
          .into_iter()
          .map(geometry_msgs::msg::PoseStamped::from_rmw_message)
          .collect(),
      behavior_tree: msg.behavior_tree.to_string(),
    }
  }
}


// Corresponds to nav2_msgs__action__NavigateThroughPoses_Result

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateThroughPoses_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub result: std_msgs::msg::Empty,

}



impl Default for NavigateThroughPoses_Result {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::NavigateThroughPoses_Result::default())
  }
}

impl rosidl_runtime_rs::Message for NavigateThroughPoses_Result {
  type RmwMsg = super::action::rmw::NavigateThroughPoses_Result;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        result: std_msgs::msg::Empty::into_rmw_message(std::borrow::Cow::Owned(msg.result)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        result: std_msgs::msg::Empty::into_rmw_message(std::borrow::Cow::Borrowed(&msg.result)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      result: std_msgs::msg::Empty::from_rmw_message(msg.result),
    }
  }
}


// Corresponds to nav2_msgs__action__NavigateThroughPoses_Feedback

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateThroughPoses_Feedback {

    // This member is not documented.
    #[allow(missing_docs)]
    pub current_pose: geometry_msgs::msg::PoseStamped,


    // This member is not documented.
    #[allow(missing_docs)]
    pub navigation_time: builtin_interfaces::msg::Duration,


    // This member is not documented.
    #[allow(missing_docs)]
    pub estimated_time_remaining: builtin_interfaces::msg::Duration,


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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::NavigateThroughPoses_Feedback::default())
  }
}

impl rosidl_runtime_rs::Message for NavigateThroughPoses_Feedback {
  type RmwMsg = super::action::rmw::NavigateThroughPoses_Feedback;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        current_pose: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Owned(msg.current_pose)).into_owned(),
        navigation_time: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Owned(msg.navigation_time)).into_owned(),
        estimated_time_remaining: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Owned(msg.estimated_time_remaining)).into_owned(),
        number_of_recoveries: msg.number_of_recoveries,
        distance_remaining: msg.distance_remaining,
        number_of_poses_remaining: msg.number_of_poses_remaining,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        current_pose: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Borrowed(&msg.current_pose)).into_owned(),
        navigation_time: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Borrowed(&msg.navigation_time)).into_owned(),
        estimated_time_remaining: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Borrowed(&msg.estimated_time_remaining)).into_owned(),
      number_of_recoveries: msg.number_of_recoveries,
      distance_remaining: msg.distance_remaining,
      number_of_poses_remaining: msg.number_of_poses_remaining,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      current_pose: geometry_msgs::msg::PoseStamped::from_rmw_message(msg.current_pose),
      navigation_time: builtin_interfaces::msg::Duration::from_rmw_message(msg.navigation_time),
      estimated_time_remaining: builtin_interfaces::msg::Duration::from_rmw_message(msg.estimated_time_remaining),
      number_of_recoveries: msg.number_of_recoveries,
      distance_remaining: msg.distance_remaining,
      number_of_poses_remaining: msg.number_of_poses_remaining,
    }
  }
}


// Corresponds to nav2_msgs__action__NavigateThroughPoses_FeedbackMessage

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateThroughPoses_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::action::NavigateThroughPoses_Feedback,

}



impl Default for NavigateThroughPoses_FeedbackMessage {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::NavigateThroughPoses_FeedbackMessage::default())
  }
}

impl rosidl_runtime_rs::Message for NavigateThroughPoses_FeedbackMessage {
  type RmwMsg = super::action::rmw::NavigateThroughPoses_FeedbackMessage;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        feedback: super::action::NavigateThroughPoses_Feedback::into_rmw_message(std::borrow::Cow::Owned(msg.feedback)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        feedback: super::action::NavigateThroughPoses_Feedback::into_rmw_message(std::borrow::Cow::Borrowed(&msg.feedback)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      feedback: super::action::NavigateThroughPoses_Feedback::from_rmw_message(msg.feedback),
    }
  }
}


// Corresponds to nav2_msgs__action__Wait_Goal

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Wait_Goal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub time: builtin_interfaces::msg::Duration,

}



impl Default for Wait_Goal {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::Wait_Goal::default())
  }
}

impl rosidl_runtime_rs::Message for Wait_Goal {
  type RmwMsg = super::action::rmw::Wait_Goal;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        time: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Owned(msg.time)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        time: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Borrowed(&msg.time)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      time: builtin_interfaces::msg::Duration::from_rmw_message(msg.time),
    }
  }
}


// Corresponds to nav2_msgs__action__Wait_Result

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Wait_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub total_elapsed_time: builtin_interfaces::msg::Duration,

}



impl Default for Wait_Result {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::Wait_Result::default())
  }
}

impl rosidl_runtime_rs::Message for Wait_Result {
  type RmwMsg = super::action::rmw::Wait_Result;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        total_elapsed_time: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Owned(msg.total_elapsed_time)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        total_elapsed_time: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Borrowed(&msg.total_elapsed_time)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      total_elapsed_time: builtin_interfaces::msg::Duration::from_rmw_message(msg.total_elapsed_time),
    }
  }
}


// Corresponds to nav2_msgs__action__Wait_Feedback

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Wait_Feedback {

    // This member is not documented.
    #[allow(missing_docs)]
    pub time_left: builtin_interfaces::msg::Duration,

}



impl Default for Wait_Feedback {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::Wait_Feedback::default())
  }
}

impl rosidl_runtime_rs::Message for Wait_Feedback {
  type RmwMsg = super::action::rmw::Wait_Feedback;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        time_left: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Owned(msg.time_left)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        time_left: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Borrowed(&msg.time_left)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      time_left: builtin_interfaces::msg::Duration::from_rmw_message(msg.time_left),
    }
  }
}


// Corresponds to nav2_msgs__action__Wait_FeedbackMessage

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Wait_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::action::Wait_Feedback,

}



impl Default for Wait_FeedbackMessage {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::Wait_FeedbackMessage::default())
  }
}

impl rosidl_runtime_rs::Message for Wait_FeedbackMessage {
  type RmwMsg = super::action::rmw::Wait_FeedbackMessage;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        feedback: super::action::Wait_Feedback::into_rmw_message(std::borrow::Cow::Owned(msg.feedback)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        feedback: super::action::Wait_Feedback::into_rmw_message(std::borrow::Cow::Borrowed(&msg.feedback)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      feedback: super::action::Wait_Feedback::from_rmw_message(msg.feedback),
    }
  }
}


// Corresponds to nav2_msgs__action__Spin_Goal

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Spin_Goal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub target_yaw: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub time_allowance: builtin_interfaces::msg::Duration,

}



impl Default for Spin_Goal {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::Spin_Goal::default())
  }
}

impl rosidl_runtime_rs::Message for Spin_Goal {
  type RmwMsg = super::action::rmw::Spin_Goal;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        target_yaw: msg.target_yaw,
        time_allowance: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Owned(msg.time_allowance)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      target_yaw: msg.target_yaw,
        time_allowance: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Borrowed(&msg.time_allowance)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      target_yaw: msg.target_yaw,
      time_allowance: builtin_interfaces::msg::Duration::from_rmw_message(msg.time_allowance),
    }
  }
}


// Corresponds to nav2_msgs__action__Spin_Result

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Spin_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub total_elapsed_time: builtin_interfaces::msg::Duration,

}



impl Default for Spin_Result {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::Spin_Result::default())
  }
}

impl rosidl_runtime_rs::Message for Spin_Result {
  type RmwMsg = super::action::rmw::Spin_Result;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        total_elapsed_time: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Owned(msg.total_elapsed_time)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        total_elapsed_time: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Borrowed(&msg.total_elapsed_time)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      total_elapsed_time: builtin_interfaces::msg::Duration::from_rmw_message(msg.total_elapsed_time),
    }
  }
}


// Corresponds to nav2_msgs__action__Spin_Feedback

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Spin_Feedback {

    // This member is not documented.
    #[allow(missing_docs)]
    pub angular_distance_traveled: f32,

}



impl Default for Spin_Feedback {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::Spin_Feedback::default())
  }
}

impl rosidl_runtime_rs::Message for Spin_Feedback {
  type RmwMsg = super::action::rmw::Spin_Feedback;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        angular_distance_traveled: msg.angular_distance_traveled,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      angular_distance_traveled: msg.angular_distance_traveled,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      angular_distance_traveled: msg.angular_distance_traveled,
    }
  }
}


// Corresponds to nav2_msgs__action__Spin_FeedbackMessage

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Spin_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::action::Spin_Feedback,

}



impl Default for Spin_FeedbackMessage {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::Spin_FeedbackMessage::default())
  }
}

impl rosidl_runtime_rs::Message for Spin_FeedbackMessage {
  type RmwMsg = super::action::rmw::Spin_FeedbackMessage;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        feedback: super::action::Spin_Feedback::into_rmw_message(std::borrow::Cow::Owned(msg.feedback)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        feedback: super::action::Spin_Feedback::into_rmw_message(std::borrow::Cow::Borrowed(&msg.feedback)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      feedback: super::action::Spin_Feedback::from_rmw_message(msg.feedback),
    }
  }
}


// Corresponds to nav2_msgs__action__DummyBehavior_Goal

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DummyBehavior_Goal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub command: std_msgs::msg::String,

}



impl Default for DummyBehavior_Goal {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::DummyBehavior_Goal::default())
  }
}

impl rosidl_runtime_rs::Message for DummyBehavior_Goal {
  type RmwMsg = super::action::rmw::DummyBehavior_Goal;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        command: std_msgs::msg::String::into_rmw_message(std::borrow::Cow::Owned(msg.command)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        command: std_msgs::msg::String::into_rmw_message(std::borrow::Cow::Borrowed(&msg.command)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      command: std_msgs::msg::String::from_rmw_message(msg.command),
    }
  }
}


// Corresponds to nav2_msgs__action__DummyBehavior_Result

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DummyBehavior_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub total_elapsed_time: builtin_interfaces::msg::Duration,

}



impl Default for DummyBehavior_Result {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::DummyBehavior_Result::default())
  }
}

impl rosidl_runtime_rs::Message for DummyBehavior_Result {
  type RmwMsg = super::action::rmw::DummyBehavior_Result;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        total_elapsed_time: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Owned(msg.total_elapsed_time)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        total_elapsed_time: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Borrowed(&msg.total_elapsed_time)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      total_elapsed_time: builtin_interfaces::msg::Duration::from_rmw_message(msg.total_elapsed_time),
    }
  }
}


// Corresponds to nav2_msgs__action__DummyBehavior_Feedback

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DummyBehavior_Feedback {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for DummyBehavior_Feedback {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::DummyBehavior_Feedback::default())
  }
}

impl rosidl_runtime_rs::Message for DummyBehavior_Feedback {
  type RmwMsg = super::action::rmw::DummyBehavior_Feedback;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
    }
  }
}


// Corresponds to nav2_msgs__action__DummyBehavior_FeedbackMessage

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DummyBehavior_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::action::DummyBehavior_Feedback,

}



impl Default for DummyBehavior_FeedbackMessage {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::DummyBehavior_FeedbackMessage::default())
  }
}

impl rosidl_runtime_rs::Message for DummyBehavior_FeedbackMessage {
  type RmwMsg = super::action::rmw::DummyBehavior_FeedbackMessage;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        feedback: super::action::DummyBehavior_Feedback::into_rmw_message(std::borrow::Cow::Owned(msg.feedback)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        feedback: super::action::DummyBehavior_Feedback::into_rmw_message(std::borrow::Cow::Borrowed(&msg.feedback)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      feedback: super::action::DummyBehavior_Feedback::from_rmw_message(msg.feedback),
    }
  }
}


// Corresponds to nav2_msgs__action__FollowWaypoints_Goal

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowWaypoints_Goal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub poses: Vec<geometry_msgs::msg::PoseStamped>,

}



impl Default for FollowWaypoints_Goal {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::FollowWaypoints_Goal::default())
  }
}

impl rosidl_runtime_rs::Message for FollowWaypoints_Goal {
  type RmwMsg = super::action::rmw::FollowWaypoints_Goal;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        poses: msg.poses
          .into_iter()
          .map(|elem| geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        poses: msg.poses
          .iter()
          .map(|elem| geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      poses: msg.poses
          .into_iter()
          .map(geometry_msgs::msg::PoseStamped::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to nav2_msgs__action__FollowWaypoints_Result

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowWaypoints_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub missed_waypoints: Vec<i32>,

}



impl Default for FollowWaypoints_Result {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::FollowWaypoints_Result::default())
  }
}

impl rosidl_runtime_rs::Message for FollowWaypoints_Result {
  type RmwMsg = super::action::rmw::FollowWaypoints_Result;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        missed_waypoints: msg.missed_waypoints.as_slice().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        missed_waypoints: msg.missed_waypoints.as_slice().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      missed_waypoints: msg.missed_waypoints.into(),
    }
  }
}


// Corresponds to nav2_msgs__action__FollowWaypoints_Feedback

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowWaypoints_Feedback {

    // This member is not documented.
    #[allow(missing_docs)]
    pub current_waypoint: u32,

}



impl Default for FollowWaypoints_Feedback {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::FollowWaypoints_Feedback::default())
  }
}

impl rosidl_runtime_rs::Message for FollowWaypoints_Feedback {
  type RmwMsg = super::action::rmw::FollowWaypoints_Feedback;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        current_waypoint: msg.current_waypoint,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      current_waypoint: msg.current_waypoint,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      current_waypoint: msg.current_waypoint,
    }
  }
}


// Corresponds to nav2_msgs__action__FollowWaypoints_FeedbackMessage

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowWaypoints_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::action::FollowWaypoints_Feedback,

}



impl Default for FollowWaypoints_FeedbackMessage {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::FollowWaypoints_FeedbackMessage::default())
  }
}

impl rosidl_runtime_rs::Message for FollowWaypoints_FeedbackMessage {
  type RmwMsg = super::action::rmw::FollowWaypoints_FeedbackMessage;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        feedback: super::action::FollowWaypoints_Feedback::into_rmw_message(std::borrow::Cow::Owned(msg.feedback)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        feedback: super::action::FollowWaypoints_Feedback::into_rmw_message(std::borrow::Cow::Borrowed(&msg.feedback)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      feedback: super::action::FollowWaypoints_Feedback::from_rmw_message(msg.feedback),
    }
  }
}






// Corresponds to nav2_msgs__action__AssistedTeleop_SendGoal_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AssistedTeleop_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::action::AssistedTeleop_Goal,

}



impl Default for AssistedTeleop_SendGoal_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::AssistedTeleop_SendGoal_Request::default())
  }
}

impl rosidl_runtime_rs::Message for AssistedTeleop_SendGoal_Request {
  type RmwMsg = super::action::rmw::AssistedTeleop_SendGoal_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        goal: super::action::AssistedTeleop_Goal::into_rmw_message(std::borrow::Cow::Owned(msg.goal)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        goal: super::action::AssistedTeleop_Goal::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      goal: super::action::AssistedTeleop_Goal::from_rmw_message(msg.goal),
    }
  }
}


// Corresponds to nav2_msgs__action__AssistedTeleop_SendGoal_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AssistedTeleop_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,

}



impl Default for AssistedTeleop_SendGoal_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::AssistedTeleop_SendGoal_Response::default())
  }
}

impl rosidl_runtime_rs::Message for AssistedTeleop_SendGoal_Response {
  type RmwMsg = super::action::rmw::AssistedTeleop_SendGoal_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      accepted: msg.accepted,
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
    }
  }
}


// Corresponds to nav2_msgs__action__AssistedTeleop_GetResult_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AssistedTeleop_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,

}



impl Default for AssistedTeleop_GetResult_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::AssistedTeleop_GetResult_Request::default())
  }
}

impl rosidl_runtime_rs::Message for AssistedTeleop_GetResult_Request {
  type RmwMsg = super::action::rmw::AssistedTeleop_GetResult_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
    }
  }
}


// Corresponds to nav2_msgs__action__AssistedTeleop_GetResult_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AssistedTeleop_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::action::AssistedTeleop_Result,

}



impl Default for AssistedTeleop_GetResult_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::AssistedTeleop_GetResult_Response::default())
  }
}

impl rosidl_runtime_rs::Message for AssistedTeleop_GetResult_Response {
  type RmwMsg = super::action::rmw::AssistedTeleop_GetResult_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: msg.status,
        result: super::action::AssistedTeleop_Result::into_rmw_message(std::borrow::Cow::Owned(msg.result)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      status: msg.status,
        result: super::action::AssistedTeleop_Result::into_rmw_message(std::borrow::Cow::Borrowed(&msg.result)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: msg.status,
      result: super::action::AssistedTeleop_Result::from_rmw_message(msg.result),
    }
  }
}


// Corresponds to nav2_msgs__action__BackUp_SendGoal_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BackUp_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::action::BackUp_Goal,

}



impl Default for BackUp_SendGoal_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::BackUp_SendGoal_Request::default())
  }
}

impl rosidl_runtime_rs::Message for BackUp_SendGoal_Request {
  type RmwMsg = super::action::rmw::BackUp_SendGoal_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        goal: super::action::BackUp_Goal::into_rmw_message(std::borrow::Cow::Owned(msg.goal)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        goal: super::action::BackUp_Goal::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      goal: super::action::BackUp_Goal::from_rmw_message(msg.goal),
    }
  }
}


// Corresponds to nav2_msgs__action__BackUp_SendGoal_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BackUp_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,

}



impl Default for BackUp_SendGoal_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::BackUp_SendGoal_Response::default())
  }
}

impl rosidl_runtime_rs::Message for BackUp_SendGoal_Response {
  type RmwMsg = super::action::rmw::BackUp_SendGoal_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      accepted: msg.accepted,
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
    }
  }
}


// Corresponds to nav2_msgs__action__BackUp_GetResult_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BackUp_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,

}



impl Default for BackUp_GetResult_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::BackUp_GetResult_Request::default())
  }
}

impl rosidl_runtime_rs::Message for BackUp_GetResult_Request {
  type RmwMsg = super::action::rmw::BackUp_GetResult_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
    }
  }
}


// Corresponds to nav2_msgs__action__BackUp_GetResult_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BackUp_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::action::BackUp_Result,

}



impl Default for BackUp_GetResult_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::BackUp_GetResult_Response::default())
  }
}

impl rosidl_runtime_rs::Message for BackUp_GetResult_Response {
  type RmwMsg = super::action::rmw::BackUp_GetResult_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: msg.status,
        result: super::action::BackUp_Result::into_rmw_message(std::borrow::Cow::Owned(msg.result)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      status: msg.status,
        result: super::action::BackUp_Result::into_rmw_message(std::borrow::Cow::Borrowed(&msg.result)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: msg.status,
      result: super::action::BackUp_Result::from_rmw_message(msg.result),
    }
  }
}


// Corresponds to nav2_msgs__action__ComputePathToPose_SendGoal_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathToPose_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::action::ComputePathToPose_Goal,

}



impl Default for ComputePathToPose_SendGoal_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::ComputePathToPose_SendGoal_Request::default())
  }
}

impl rosidl_runtime_rs::Message for ComputePathToPose_SendGoal_Request {
  type RmwMsg = super::action::rmw::ComputePathToPose_SendGoal_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        goal: super::action::ComputePathToPose_Goal::into_rmw_message(std::borrow::Cow::Owned(msg.goal)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        goal: super::action::ComputePathToPose_Goal::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      goal: super::action::ComputePathToPose_Goal::from_rmw_message(msg.goal),
    }
  }
}


// Corresponds to nav2_msgs__action__ComputePathToPose_SendGoal_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathToPose_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,

}



impl Default for ComputePathToPose_SendGoal_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::ComputePathToPose_SendGoal_Response::default())
  }
}

impl rosidl_runtime_rs::Message for ComputePathToPose_SendGoal_Response {
  type RmwMsg = super::action::rmw::ComputePathToPose_SendGoal_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      accepted: msg.accepted,
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
    }
  }
}


// Corresponds to nav2_msgs__action__ComputePathToPose_GetResult_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathToPose_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,

}



impl Default for ComputePathToPose_GetResult_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::ComputePathToPose_GetResult_Request::default())
  }
}

impl rosidl_runtime_rs::Message for ComputePathToPose_GetResult_Request {
  type RmwMsg = super::action::rmw::ComputePathToPose_GetResult_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
    }
  }
}


// Corresponds to nav2_msgs__action__ComputePathToPose_GetResult_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathToPose_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::action::ComputePathToPose_Result,

}



impl Default for ComputePathToPose_GetResult_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::ComputePathToPose_GetResult_Response::default())
  }
}

impl rosidl_runtime_rs::Message for ComputePathToPose_GetResult_Response {
  type RmwMsg = super::action::rmw::ComputePathToPose_GetResult_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: msg.status,
        result: super::action::ComputePathToPose_Result::into_rmw_message(std::borrow::Cow::Owned(msg.result)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      status: msg.status,
        result: super::action::ComputePathToPose_Result::into_rmw_message(std::borrow::Cow::Borrowed(&msg.result)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: msg.status,
      result: super::action::ComputePathToPose_Result::from_rmw_message(msg.result),
    }
  }
}


// Corresponds to nav2_msgs__action__ComputePathThroughPoses_SendGoal_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathThroughPoses_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::action::ComputePathThroughPoses_Goal,

}



impl Default for ComputePathThroughPoses_SendGoal_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::ComputePathThroughPoses_SendGoal_Request::default())
  }
}

impl rosidl_runtime_rs::Message for ComputePathThroughPoses_SendGoal_Request {
  type RmwMsg = super::action::rmw::ComputePathThroughPoses_SendGoal_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        goal: super::action::ComputePathThroughPoses_Goal::into_rmw_message(std::borrow::Cow::Owned(msg.goal)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        goal: super::action::ComputePathThroughPoses_Goal::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      goal: super::action::ComputePathThroughPoses_Goal::from_rmw_message(msg.goal),
    }
  }
}


// Corresponds to nav2_msgs__action__ComputePathThroughPoses_SendGoal_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathThroughPoses_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,

}



impl Default for ComputePathThroughPoses_SendGoal_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::ComputePathThroughPoses_SendGoal_Response::default())
  }
}

impl rosidl_runtime_rs::Message for ComputePathThroughPoses_SendGoal_Response {
  type RmwMsg = super::action::rmw::ComputePathThroughPoses_SendGoal_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      accepted: msg.accepted,
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
    }
  }
}


// Corresponds to nav2_msgs__action__ComputePathThroughPoses_GetResult_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathThroughPoses_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,

}



impl Default for ComputePathThroughPoses_GetResult_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::ComputePathThroughPoses_GetResult_Request::default())
  }
}

impl rosidl_runtime_rs::Message for ComputePathThroughPoses_GetResult_Request {
  type RmwMsg = super::action::rmw::ComputePathThroughPoses_GetResult_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
    }
  }
}


// Corresponds to nav2_msgs__action__ComputePathThroughPoses_GetResult_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputePathThroughPoses_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::action::ComputePathThroughPoses_Result,

}



impl Default for ComputePathThroughPoses_GetResult_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::ComputePathThroughPoses_GetResult_Response::default())
  }
}

impl rosidl_runtime_rs::Message for ComputePathThroughPoses_GetResult_Response {
  type RmwMsg = super::action::rmw::ComputePathThroughPoses_GetResult_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: msg.status,
        result: super::action::ComputePathThroughPoses_Result::into_rmw_message(std::borrow::Cow::Owned(msg.result)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      status: msg.status,
        result: super::action::ComputePathThroughPoses_Result::into_rmw_message(std::borrow::Cow::Borrowed(&msg.result)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: msg.status,
      result: super::action::ComputePathThroughPoses_Result::from_rmw_message(msg.result),
    }
  }
}


// Corresponds to nav2_msgs__action__ComputeRoute_SendGoal_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputeRoute_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::action::ComputeRoute_Goal,

}



impl Default for ComputeRoute_SendGoal_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::ComputeRoute_SendGoal_Request::default())
  }
}

impl rosidl_runtime_rs::Message for ComputeRoute_SendGoal_Request {
  type RmwMsg = super::action::rmw::ComputeRoute_SendGoal_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        goal: super::action::ComputeRoute_Goal::into_rmw_message(std::borrow::Cow::Owned(msg.goal)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        goal: super::action::ComputeRoute_Goal::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      goal: super::action::ComputeRoute_Goal::from_rmw_message(msg.goal),
    }
  }
}


// Corresponds to nav2_msgs__action__ComputeRoute_SendGoal_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputeRoute_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,

}



impl Default for ComputeRoute_SendGoal_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::ComputeRoute_SendGoal_Response::default())
  }
}

impl rosidl_runtime_rs::Message for ComputeRoute_SendGoal_Response {
  type RmwMsg = super::action::rmw::ComputeRoute_SendGoal_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      accepted: msg.accepted,
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
    }
  }
}


// Corresponds to nav2_msgs__action__ComputeRoute_GetResult_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputeRoute_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,

}



impl Default for ComputeRoute_GetResult_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::ComputeRoute_GetResult_Request::default())
  }
}

impl rosidl_runtime_rs::Message for ComputeRoute_GetResult_Request {
  type RmwMsg = super::action::rmw::ComputeRoute_GetResult_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
    }
  }
}


// Corresponds to nav2_msgs__action__ComputeRoute_GetResult_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputeRoute_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::action::ComputeRoute_Result,

}



impl Default for ComputeRoute_GetResult_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::ComputeRoute_GetResult_Response::default())
  }
}

impl rosidl_runtime_rs::Message for ComputeRoute_GetResult_Response {
  type RmwMsg = super::action::rmw::ComputeRoute_GetResult_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: msg.status,
        result: super::action::ComputeRoute_Result::into_rmw_message(std::borrow::Cow::Owned(msg.result)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      status: msg.status,
        result: super::action::ComputeRoute_Result::into_rmw_message(std::borrow::Cow::Borrowed(&msg.result)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: msg.status,
      result: super::action::ComputeRoute_Result::from_rmw_message(msg.result),
    }
  }
}


// Corresponds to nav2_msgs__action__ComputeAndTrackRoute_SendGoal_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputeAndTrackRoute_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::action::ComputeAndTrackRoute_Goal,

}



impl Default for ComputeAndTrackRoute_SendGoal_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::ComputeAndTrackRoute_SendGoal_Request::default())
  }
}

impl rosidl_runtime_rs::Message for ComputeAndTrackRoute_SendGoal_Request {
  type RmwMsg = super::action::rmw::ComputeAndTrackRoute_SendGoal_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        goal: super::action::ComputeAndTrackRoute_Goal::into_rmw_message(std::borrow::Cow::Owned(msg.goal)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        goal: super::action::ComputeAndTrackRoute_Goal::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      goal: super::action::ComputeAndTrackRoute_Goal::from_rmw_message(msg.goal),
    }
  }
}


// Corresponds to nav2_msgs__action__ComputeAndTrackRoute_SendGoal_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputeAndTrackRoute_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,

}



impl Default for ComputeAndTrackRoute_SendGoal_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::ComputeAndTrackRoute_SendGoal_Response::default())
  }
}

impl rosidl_runtime_rs::Message for ComputeAndTrackRoute_SendGoal_Response {
  type RmwMsg = super::action::rmw::ComputeAndTrackRoute_SendGoal_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      accepted: msg.accepted,
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
    }
  }
}


// Corresponds to nav2_msgs__action__ComputeAndTrackRoute_GetResult_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputeAndTrackRoute_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,

}



impl Default for ComputeAndTrackRoute_GetResult_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::ComputeAndTrackRoute_GetResult_Request::default())
  }
}

impl rosidl_runtime_rs::Message for ComputeAndTrackRoute_GetResult_Request {
  type RmwMsg = super::action::rmw::ComputeAndTrackRoute_GetResult_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
    }
  }
}


// Corresponds to nav2_msgs__action__ComputeAndTrackRoute_GetResult_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComputeAndTrackRoute_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::action::ComputeAndTrackRoute_Result,

}



impl Default for ComputeAndTrackRoute_GetResult_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::ComputeAndTrackRoute_GetResult_Response::default())
  }
}

impl rosidl_runtime_rs::Message for ComputeAndTrackRoute_GetResult_Response {
  type RmwMsg = super::action::rmw::ComputeAndTrackRoute_GetResult_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: msg.status,
        result: super::action::ComputeAndTrackRoute_Result::into_rmw_message(std::borrow::Cow::Owned(msg.result)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      status: msg.status,
        result: super::action::ComputeAndTrackRoute_Result::into_rmw_message(std::borrow::Cow::Borrowed(&msg.result)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: msg.status,
      result: super::action::ComputeAndTrackRoute_Result::from_rmw_message(msg.result),
    }
  }
}


// Corresponds to nav2_msgs__action__DriveOnHeading_SendGoal_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DriveOnHeading_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::action::DriveOnHeading_Goal,

}



impl Default for DriveOnHeading_SendGoal_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::DriveOnHeading_SendGoal_Request::default())
  }
}

impl rosidl_runtime_rs::Message for DriveOnHeading_SendGoal_Request {
  type RmwMsg = super::action::rmw::DriveOnHeading_SendGoal_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        goal: super::action::DriveOnHeading_Goal::into_rmw_message(std::borrow::Cow::Owned(msg.goal)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        goal: super::action::DriveOnHeading_Goal::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      goal: super::action::DriveOnHeading_Goal::from_rmw_message(msg.goal),
    }
  }
}


// Corresponds to nav2_msgs__action__DriveOnHeading_SendGoal_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DriveOnHeading_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,

}



impl Default for DriveOnHeading_SendGoal_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::DriveOnHeading_SendGoal_Response::default())
  }
}

impl rosidl_runtime_rs::Message for DriveOnHeading_SendGoal_Response {
  type RmwMsg = super::action::rmw::DriveOnHeading_SendGoal_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      accepted: msg.accepted,
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
    }
  }
}


// Corresponds to nav2_msgs__action__DriveOnHeading_GetResult_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DriveOnHeading_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,

}



impl Default for DriveOnHeading_GetResult_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::DriveOnHeading_GetResult_Request::default())
  }
}

impl rosidl_runtime_rs::Message for DriveOnHeading_GetResult_Request {
  type RmwMsg = super::action::rmw::DriveOnHeading_GetResult_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
    }
  }
}


// Corresponds to nav2_msgs__action__DriveOnHeading_GetResult_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DriveOnHeading_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::action::DriveOnHeading_Result,

}



impl Default for DriveOnHeading_GetResult_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::DriveOnHeading_GetResult_Response::default())
  }
}

impl rosidl_runtime_rs::Message for DriveOnHeading_GetResult_Response {
  type RmwMsg = super::action::rmw::DriveOnHeading_GetResult_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: msg.status,
        result: super::action::DriveOnHeading_Result::into_rmw_message(std::borrow::Cow::Owned(msg.result)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      status: msg.status,
        result: super::action::DriveOnHeading_Result::into_rmw_message(std::borrow::Cow::Borrowed(&msg.result)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: msg.status,
      result: super::action::DriveOnHeading_Result::from_rmw_message(msg.result),
    }
  }
}


// Corresponds to nav2_msgs__action__SmoothPath_SendGoal_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SmoothPath_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::action::SmoothPath_Goal,

}



impl Default for SmoothPath_SendGoal_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::SmoothPath_SendGoal_Request::default())
  }
}

impl rosidl_runtime_rs::Message for SmoothPath_SendGoal_Request {
  type RmwMsg = super::action::rmw::SmoothPath_SendGoal_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        goal: super::action::SmoothPath_Goal::into_rmw_message(std::borrow::Cow::Owned(msg.goal)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        goal: super::action::SmoothPath_Goal::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      goal: super::action::SmoothPath_Goal::from_rmw_message(msg.goal),
    }
  }
}


// Corresponds to nav2_msgs__action__SmoothPath_SendGoal_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SmoothPath_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,

}



impl Default for SmoothPath_SendGoal_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::SmoothPath_SendGoal_Response::default())
  }
}

impl rosidl_runtime_rs::Message for SmoothPath_SendGoal_Response {
  type RmwMsg = super::action::rmw::SmoothPath_SendGoal_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      accepted: msg.accepted,
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
    }
  }
}


// Corresponds to nav2_msgs__action__SmoothPath_GetResult_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SmoothPath_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,

}



impl Default for SmoothPath_GetResult_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::SmoothPath_GetResult_Request::default())
  }
}

impl rosidl_runtime_rs::Message for SmoothPath_GetResult_Request {
  type RmwMsg = super::action::rmw::SmoothPath_GetResult_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
    }
  }
}


// Corresponds to nav2_msgs__action__SmoothPath_GetResult_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SmoothPath_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::action::SmoothPath_Result,

}



impl Default for SmoothPath_GetResult_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::SmoothPath_GetResult_Response::default())
  }
}

impl rosidl_runtime_rs::Message for SmoothPath_GetResult_Response {
  type RmwMsg = super::action::rmw::SmoothPath_GetResult_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: msg.status,
        result: super::action::SmoothPath_Result::into_rmw_message(std::borrow::Cow::Owned(msg.result)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      status: msg.status,
        result: super::action::SmoothPath_Result::into_rmw_message(std::borrow::Cow::Borrowed(&msg.result)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: msg.status,
      result: super::action::SmoothPath_Result::from_rmw_message(msg.result),
    }
  }
}


// Corresponds to nav2_msgs__action__FollowPath_SendGoal_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowPath_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::action::FollowPath_Goal,

}



impl Default for FollowPath_SendGoal_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::FollowPath_SendGoal_Request::default())
  }
}

impl rosidl_runtime_rs::Message for FollowPath_SendGoal_Request {
  type RmwMsg = super::action::rmw::FollowPath_SendGoal_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        goal: super::action::FollowPath_Goal::into_rmw_message(std::borrow::Cow::Owned(msg.goal)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        goal: super::action::FollowPath_Goal::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      goal: super::action::FollowPath_Goal::from_rmw_message(msg.goal),
    }
  }
}


// Corresponds to nav2_msgs__action__FollowPath_SendGoal_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowPath_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,

}



impl Default for FollowPath_SendGoal_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::FollowPath_SendGoal_Response::default())
  }
}

impl rosidl_runtime_rs::Message for FollowPath_SendGoal_Response {
  type RmwMsg = super::action::rmw::FollowPath_SendGoal_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      accepted: msg.accepted,
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
    }
  }
}


// Corresponds to nav2_msgs__action__FollowPath_GetResult_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowPath_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,

}



impl Default for FollowPath_GetResult_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::FollowPath_GetResult_Request::default())
  }
}

impl rosidl_runtime_rs::Message for FollowPath_GetResult_Request {
  type RmwMsg = super::action::rmw::FollowPath_GetResult_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
    }
  }
}


// Corresponds to nav2_msgs__action__FollowPath_GetResult_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowPath_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::action::FollowPath_Result,

}



impl Default for FollowPath_GetResult_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::FollowPath_GetResult_Response::default())
  }
}

impl rosidl_runtime_rs::Message for FollowPath_GetResult_Response {
  type RmwMsg = super::action::rmw::FollowPath_GetResult_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: msg.status,
        result: super::action::FollowPath_Result::into_rmw_message(std::borrow::Cow::Owned(msg.result)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      status: msg.status,
        result: super::action::FollowPath_Result::into_rmw_message(std::borrow::Cow::Borrowed(&msg.result)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: msg.status,
      result: super::action::FollowPath_Result::from_rmw_message(msg.result),
    }
  }
}


// Corresponds to nav2_msgs__action__NavigateToPose_SendGoal_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateToPose_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::action::NavigateToPose_Goal,

}



impl Default for NavigateToPose_SendGoal_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::NavigateToPose_SendGoal_Request::default())
  }
}

impl rosidl_runtime_rs::Message for NavigateToPose_SendGoal_Request {
  type RmwMsg = super::action::rmw::NavigateToPose_SendGoal_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        goal: super::action::NavigateToPose_Goal::into_rmw_message(std::borrow::Cow::Owned(msg.goal)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        goal: super::action::NavigateToPose_Goal::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      goal: super::action::NavigateToPose_Goal::from_rmw_message(msg.goal),
    }
  }
}


// Corresponds to nav2_msgs__action__NavigateToPose_SendGoal_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateToPose_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,

}



impl Default for NavigateToPose_SendGoal_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::NavigateToPose_SendGoal_Response::default())
  }
}

impl rosidl_runtime_rs::Message for NavigateToPose_SendGoal_Response {
  type RmwMsg = super::action::rmw::NavigateToPose_SendGoal_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      accepted: msg.accepted,
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
    }
  }
}


// Corresponds to nav2_msgs__action__NavigateToPose_GetResult_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateToPose_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,

}



impl Default for NavigateToPose_GetResult_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::NavigateToPose_GetResult_Request::default())
  }
}

impl rosidl_runtime_rs::Message for NavigateToPose_GetResult_Request {
  type RmwMsg = super::action::rmw::NavigateToPose_GetResult_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
    }
  }
}


// Corresponds to nav2_msgs__action__NavigateToPose_GetResult_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateToPose_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::action::NavigateToPose_Result,

}



impl Default for NavigateToPose_GetResult_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::NavigateToPose_GetResult_Response::default())
  }
}

impl rosidl_runtime_rs::Message for NavigateToPose_GetResult_Response {
  type RmwMsg = super::action::rmw::NavigateToPose_GetResult_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: msg.status,
        result: super::action::NavigateToPose_Result::into_rmw_message(std::borrow::Cow::Owned(msg.result)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      status: msg.status,
        result: super::action::NavigateToPose_Result::into_rmw_message(std::borrow::Cow::Borrowed(&msg.result)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: msg.status,
      result: super::action::NavigateToPose_Result::from_rmw_message(msg.result),
    }
  }
}


// Corresponds to nav2_msgs__action__NavigateThroughPoses_SendGoal_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateThroughPoses_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::action::NavigateThroughPoses_Goal,

}



impl Default for NavigateThroughPoses_SendGoal_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::NavigateThroughPoses_SendGoal_Request::default())
  }
}

impl rosidl_runtime_rs::Message for NavigateThroughPoses_SendGoal_Request {
  type RmwMsg = super::action::rmw::NavigateThroughPoses_SendGoal_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        goal: super::action::NavigateThroughPoses_Goal::into_rmw_message(std::borrow::Cow::Owned(msg.goal)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        goal: super::action::NavigateThroughPoses_Goal::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      goal: super::action::NavigateThroughPoses_Goal::from_rmw_message(msg.goal),
    }
  }
}


// Corresponds to nav2_msgs__action__NavigateThroughPoses_SendGoal_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateThroughPoses_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,

}



impl Default for NavigateThroughPoses_SendGoal_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::NavigateThroughPoses_SendGoal_Response::default())
  }
}

impl rosidl_runtime_rs::Message for NavigateThroughPoses_SendGoal_Response {
  type RmwMsg = super::action::rmw::NavigateThroughPoses_SendGoal_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      accepted: msg.accepted,
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
    }
  }
}


// Corresponds to nav2_msgs__action__NavigateThroughPoses_GetResult_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateThroughPoses_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,

}



impl Default for NavigateThroughPoses_GetResult_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::NavigateThroughPoses_GetResult_Request::default())
  }
}

impl rosidl_runtime_rs::Message for NavigateThroughPoses_GetResult_Request {
  type RmwMsg = super::action::rmw::NavigateThroughPoses_GetResult_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
    }
  }
}


// Corresponds to nav2_msgs__action__NavigateThroughPoses_GetResult_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavigateThroughPoses_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::action::NavigateThroughPoses_Result,

}



impl Default for NavigateThroughPoses_GetResult_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::NavigateThroughPoses_GetResult_Response::default())
  }
}

impl rosidl_runtime_rs::Message for NavigateThroughPoses_GetResult_Response {
  type RmwMsg = super::action::rmw::NavigateThroughPoses_GetResult_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: msg.status,
        result: super::action::NavigateThroughPoses_Result::into_rmw_message(std::borrow::Cow::Owned(msg.result)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      status: msg.status,
        result: super::action::NavigateThroughPoses_Result::into_rmw_message(std::borrow::Cow::Borrowed(&msg.result)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: msg.status,
      result: super::action::NavigateThroughPoses_Result::from_rmw_message(msg.result),
    }
  }
}


// Corresponds to nav2_msgs__action__Wait_SendGoal_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Wait_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::action::Wait_Goal,

}



impl Default for Wait_SendGoal_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::Wait_SendGoal_Request::default())
  }
}

impl rosidl_runtime_rs::Message for Wait_SendGoal_Request {
  type RmwMsg = super::action::rmw::Wait_SendGoal_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        goal: super::action::Wait_Goal::into_rmw_message(std::borrow::Cow::Owned(msg.goal)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        goal: super::action::Wait_Goal::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      goal: super::action::Wait_Goal::from_rmw_message(msg.goal),
    }
  }
}


// Corresponds to nav2_msgs__action__Wait_SendGoal_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Wait_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,

}



impl Default for Wait_SendGoal_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::Wait_SendGoal_Response::default())
  }
}

impl rosidl_runtime_rs::Message for Wait_SendGoal_Response {
  type RmwMsg = super::action::rmw::Wait_SendGoal_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      accepted: msg.accepted,
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
    }
  }
}


// Corresponds to nav2_msgs__action__Wait_GetResult_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Wait_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,

}



impl Default for Wait_GetResult_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::Wait_GetResult_Request::default())
  }
}

impl rosidl_runtime_rs::Message for Wait_GetResult_Request {
  type RmwMsg = super::action::rmw::Wait_GetResult_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
    }
  }
}


// Corresponds to nav2_msgs__action__Wait_GetResult_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Wait_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::action::Wait_Result,

}



impl Default for Wait_GetResult_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::Wait_GetResult_Response::default())
  }
}

impl rosidl_runtime_rs::Message for Wait_GetResult_Response {
  type RmwMsg = super::action::rmw::Wait_GetResult_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: msg.status,
        result: super::action::Wait_Result::into_rmw_message(std::borrow::Cow::Owned(msg.result)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      status: msg.status,
        result: super::action::Wait_Result::into_rmw_message(std::borrow::Cow::Borrowed(&msg.result)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: msg.status,
      result: super::action::Wait_Result::from_rmw_message(msg.result),
    }
  }
}


// Corresponds to nav2_msgs__action__Spin_SendGoal_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Spin_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::action::Spin_Goal,

}



impl Default for Spin_SendGoal_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::Spin_SendGoal_Request::default())
  }
}

impl rosidl_runtime_rs::Message for Spin_SendGoal_Request {
  type RmwMsg = super::action::rmw::Spin_SendGoal_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        goal: super::action::Spin_Goal::into_rmw_message(std::borrow::Cow::Owned(msg.goal)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        goal: super::action::Spin_Goal::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      goal: super::action::Spin_Goal::from_rmw_message(msg.goal),
    }
  }
}


// Corresponds to nav2_msgs__action__Spin_SendGoal_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Spin_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,

}



impl Default for Spin_SendGoal_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::Spin_SendGoal_Response::default())
  }
}

impl rosidl_runtime_rs::Message for Spin_SendGoal_Response {
  type RmwMsg = super::action::rmw::Spin_SendGoal_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      accepted: msg.accepted,
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
    }
  }
}


// Corresponds to nav2_msgs__action__Spin_GetResult_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Spin_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,

}



impl Default for Spin_GetResult_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::Spin_GetResult_Request::default())
  }
}

impl rosidl_runtime_rs::Message for Spin_GetResult_Request {
  type RmwMsg = super::action::rmw::Spin_GetResult_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
    }
  }
}


// Corresponds to nav2_msgs__action__Spin_GetResult_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Spin_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::action::Spin_Result,

}



impl Default for Spin_GetResult_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::Spin_GetResult_Response::default())
  }
}

impl rosidl_runtime_rs::Message for Spin_GetResult_Response {
  type RmwMsg = super::action::rmw::Spin_GetResult_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: msg.status,
        result: super::action::Spin_Result::into_rmw_message(std::borrow::Cow::Owned(msg.result)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      status: msg.status,
        result: super::action::Spin_Result::into_rmw_message(std::borrow::Cow::Borrowed(&msg.result)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: msg.status,
      result: super::action::Spin_Result::from_rmw_message(msg.result),
    }
  }
}


// Corresponds to nav2_msgs__action__DummyBehavior_SendGoal_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DummyBehavior_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::action::DummyBehavior_Goal,

}



impl Default for DummyBehavior_SendGoal_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::DummyBehavior_SendGoal_Request::default())
  }
}

impl rosidl_runtime_rs::Message for DummyBehavior_SendGoal_Request {
  type RmwMsg = super::action::rmw::DummyBehavior_SendGoal_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        goal: super::action::DummyBehavior_Goal::into_rmw_message(std::borrow::Cow::Owned(msg.goal)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        goal: super::action::DummyBehavior_Goal::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      goal: super::action::DummyBehavior_Goal::from_rmw_message(msg.goal),
    }
  }
}


// Corresponds to nav2_msgs__action__DummyBehavior_SendGoal_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DummyBehavior_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,

}



impl Default for DummyBehavior_SendGoal_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::DummyBehavior_SendGoal_Response::default())
  }
}

impl rosidl_runtime_rs::Message for DummyBehavior_SendGoal_Response {
  type RmwMsg = super::action::rmw::DummyBehavior_SendGoal_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      accepted: msg.accepted,
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
    }
  }
}


// Corresponds to nav2_msgs__action__DummyBehavior_GetResult_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DummyBehavior_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,

}



impl Default for DummyBehavior_GetResult_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::DummyBehavior_GetResult_Request::default())
  }
}

impl rosidl_runtime_rs::Message for DummyBehavior_GetResult_Request {
  type RmwMsg = super::action::rmw::DummyBehavior_GetResult_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
    }
  }
}


// Corresponds to nav2_msgs__action__DummyBehavior_GetResult_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DummyBehavior_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::action::DummyBehavior_Result,

}



impl Default for DummyBehavior_GetResult_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::DummyBehavior_GetResult_Response::default())
  }
}

impl rosidl_runtime_rs::Message for DummyBehavior_GetResult_Response {
  type RmwMsg = super::action::rmw::DummyBehavior_GetResult_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: msg.status,
        result: super::action::DummyBehavior_Result::into_rmw_message(std::borrow::Cow::Owned(msg.result)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      status: msg.status,
        result: super::action::DummyBehavior_Result::into_rmw_message(std::borrow::Cow::Borrowed(&msg.result)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: msg.status,
      result: super::action::DummyBehavior_Result::from_rmw_message(msg.result),
    }
  }
}


// Corresponds to nav2_msgs__action__FollowWaypoints_SendGoal_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowWaypoints_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::action::FollowWaypoints_Goal,

}



impl Default for FollowWaypoints_SendGoal_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::FollowWaypoints_SendGoal_Request::default())
  }
}

impl rosidl_runtime_rs::Message for FollowWaypoints_SendGoal_Request {
  type RmwMsg = super::action::rmw::FollowWaypoints_SendGoal_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        goal: super::action::FollowWaypoints_Goal::into_rmw_message(std::borrow::Cow::Owned(msg.goal)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        goal: super::action::FollowWaypoints_Goal::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      goal: super::action::FollowWaypoints_Goal::from_rmw_message(msg.goal),
    }
  }
}


// Corresponds to nav2_msgs__action__FollowWaypoints_SendGoal_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowWaypoints_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,

}



impl Default for FollowWaypoints_SendGoal_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::FollowWaypoints_SendGoal_Response::default())
  }
}

impl rosidl_runtime_rs::Message for FollowWaypoints_SendGoal_Response {
  type RmwMsg = super::action::rmw::FollowWaypoints_SendGoal_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      accepted: msg.accepted,
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
    }
  }
}


// Corresponds to nav2_msgs__action__FollowWaypoints_GetResult_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowWaypoints_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,

}



impl Default for FollowWaypoints_GetResult_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::FollowWaypoints_GetResult_Request::default())
  }
}

impl rosidl_runtime_rs::Message for FollowWaypoints_GetResult_Request {
  type RmwMsg = super::action::rmw::FollowWaypoints_GetResult_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
    }
  }
}


// Corresponds to nav2_msgs__action__FollowWaypoints_GetResult_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FollowWaypoints_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::action::FollowWaypoints_Result,

}



impl Default for FollowWaypoints_GetResult_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::FollowWaypoints_GetResult_Response::default())
  }
}

impl rosidl_runtime_rs::Message for FollowWaypoints_GetResult_Response {
  type RmwMsg = super::action::rmw::FollowWaypoints_GetResult_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: msg.status,
        result: super::action::FollowWaypoints_Result::into_rmw_message(std::borrow::Cow::Owned(msg.result)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      status: msg.status,
        result: super::action::FollowWaypoints_Result::into_rmw_message(std::borrow::Cow::Borrowed(&msg.result)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: msg.status,
      result: super::action::FollowWaypoints_Result::from_rmw_message(msg.result),
    }
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






#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__AssistedTeleop() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__AssistedTeleop
#[allow(missing_docs, non_camel_case_types)]
pub struct AssistedTeleop;

impl rosidl_runtime_rs::Action for AssistedTeleop {
  // --- Associated types for client library users ---
  /// The goal message defined in the action definition.
  type Goal = AssistedTeleop_Goal;

  /// The result message defined in the action definition.
  type Result = AssistedTeleop_Result;

  /// The feedback message defined in the action definition.
  type Feedback = AssistedTeleop_Feedback;

  // --- Associated types for client library implementation ---
  /// The feedback message with generic fields which wraps the feedback message.
  type FeedbackMessage = super::action::AssistedTeleop_FeedbackMessage;

  /// The send_goal service using a wrapped version of the goal message as a request.
  type SendGoalService = super::action::AssistedTeleop_SendGoal;

  /// The generic service to cancel a goal.
  type CancelGoalService = action_msgs::srv::rmw::CancelGoal;

  /// The get_result service using a wrapped version of the result message as a response.
  type GetResultService = super::action::AssistedTeleop_GetResult;

  // --- Methods for client library implementation ---
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__AssistedTeleop() }
  }

  fn create_goal_request(
    goal_id: &[u8; 16],
    goal: super::action::rmw::AssistedTeleop_Goal,
  ) -> super::action::rmw::AssistedTeleop_SendGoal_Request {
   super::action::rmw::AssistedTeleop_SendGoal_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
      goal,
    }
  }

  fn split_goal_request(
    request: super::action::rmw::AssistedTeleop_SendGoal_Request,
  ) -> (
    [u8; 16],
   super::action::rmw::AssistedTeleop_Goal,
  ) {
    (request.goal_id.uuid, request.goal)
  }

  fn create_goal_response(
    accepted: bool,
    stamp: (i32, u32),
  ) -> super::action::rmw::AssistedTeleop_SendGoal_Response {
   super::action::rmw::AssistedTeleop_SendGoal_Response {
      accepted,
      stamp: builtin_interfaces::msg::rmw::Time {
        sec: stamp.0,
        nanosec: stamp.1,
      },
    }
  }

  fn get_goal_response_accepted(
    response: &super::action::rmw::AssistedTeleop_SendGoal_Response,
  ) -> bool {
    response.accepted
  }

  fn get_goal_response_stamp(
    response: &super::action::rmw::AssistedTeleop_SendGoal_Response,
  ) -> (i32, u32) {
    (response.stamp.sec, response.stamp.nanosec)
  }

  fn create_feedback_message(
    goal_id: &[u8; 16],
    feedback: super::action::rmw::AssistedTeleop_Feedback,
  ) -> super::action::rmw::AssistedTeleop_FeedbackMessage {
    let mut message = super::action::rmw::AssistedTeleop_FeedbackMessage::default();
    message.goal_id.uuid = *goal_id;
    message.feedback = feedback;
    message
  }

  fn split_feedback_message(
    feedback: super::action::rmw::AssistedTeleop_FeedbackMessage,
  ) -> (
    [u8; 16],
   super::action::rmw::AssistedTeleop_Feedback,
  ) {
    (feedback.goal_id.uuid, feedback.feedback)
  }

  fn create_result_request(
    goal_id: &[u8; 16],
  ) -> super::action::rmw::AssistedTeleop_GetResult_Request {
   super::action::rmw::AssistedTeleop_GetResult_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
    }
  }

  fn get_result_request_uuid(
    request: &super::action::rmw::AssistedTeleop_GetResult_Request,
  ) -> &[u8; 16] {
    &request.goal_id.uuid
  }

  fn create_result_response(
    status: i8,
    result: super::action::rmw::AssistedTeleop_Result,
  ) -> super::action::rmw::AssistedTeleop_GetResult_Response {
   super::action::rmw::AssistedTeleop_GetResult_Response {
      status,
      result,
    }
  }

  fn split_result_response(
    response: super::action::rmw::AssistedTeleop_GetResult_Response
  ) -> (
    i8,
   super::action::rmw::AssistedTeleop_Result,
  ) {
    (response.status, response.result)
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__BackUp() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__BackUp
#[allow(missing_docs, non_camel_case_types)]
pub struct BackUp;

impl rosidl_runtime_rs::Action for BackUp {
  // --- Associated types for client library users ---
  /// The goal message defined in the action definition.
  type Goal = BackUp_Goal;

  /// The result message defined in the action definition.
  type Result = BackUp_Result;

  /// The feedback message defined in the action definition.
  type Feedback = BackUp_Feedback;

  // --- Associated types for client library implementation ---
  /// The feedback message with generic fields which wraps the feedback message.
  type FeedbackMessage = super::action::BackUp_FeedbackMessage;

  /// The send_goal service using a wrapped version of the goal message as a request.
  type SendGoalService = super::action::BackUp_SendGoal;

  /// The generic service to cancel a goal.
  type CancelGoalService = action_msgs::srv::rmw::CancelGoal;

  /// The get_result service using a wrapped version of the result message as a response.
  type GetResultService = super::action::BackUp_GetResult;

  // --- Methods for client library implementation ---
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__BackUp() }
  }

  fn create_goal_request(
    goal_id: &[u8; 16],
    goal: super::action::rmw::BackUp_Goal,
  ) -> super::action::rmw::BackUp_SendGoal_Request {
   super::action::rmw::BackUp_SendGoal_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
      goal,
    }
  }

  fn split_goal_request(
    request: super::action::rmw::BackUp_SendGoal_Request,
  ) -> (
    [u8; 16],
   super::action::rmw::BackUp_Goal,
  ) {
    (request.goal_id.uuid, request.goal)
  }

  fn create_goal_response(
    accepted: bool,
    stamp: (i32, u32),
  ) -> super::action::rmw::BackUp_SendGoal_Response {
   super::action::rmw::BackUp_SendGoal_Response {
      accepted,
      stamp: builtin_interfaces::msg::rmw::Time {
        sec: stamp.0,
        nanosec: stamp.1,
      },
    }
  }

  fn get_goal_response_accepted(
    response: &super::action::rmw::BackUp_SendGoal_Response,
  ) -> bool {
    response.accepted
  }

  fn get_goal_response_stamp(
    response: &super::action::rmw::BackUp_SendGoal_Response,
  ) -> (i32, u32) {
    (response.stamp.sec, response.stamp.nanosec)
  }

  fn create_feedback_message(
    goal_id: &[u8; 16],
    feedback: super::action::rmw::BackUp_Feedback,
  ) -> super::action::rmw::BackUp_FeedbackMessage {
    let mut message = super::action::rmw::BackUp_FeedbackMessage::default();
    message.goal_id.uuid = *goal_id;
    message.feedback = feedback;
    message
  }

  fn split_feedback_message(
    feedback: super::action::rmw::BackUp_FeedbackMessage,
  ) -> (
    [u8; 16],
   super::action::rmw::BackUp_Feedback,
  ) {
    (feedback.goal_id.uuid, feedback.feedback)
  }

  fn create_result_request(
    goal_id: &[u8; 16],
  ) -> super::action::rmw::BackUp_GetResult_Request {
   super::action::rmw::BackUp_GetResult_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
    }
  }

  fn get_result_request_uuid(
    request: &super::action::rmw::BackUp_GetResult_Request,
  ) -> &[u8; 16] {
    &request.goal_id.uuid
  }

  fn create_result_response(
    status: i8,
    result: super::action::rmw::BackUp_Result,
  ) -> super::action::rmw::BackUp_GetResult_Response {
   super::action::rmw::BackUp_GetResult_Response {
      status,
      result,
    }
  }

  fn split_result_response(
    response: super::action::rmw::BackUp_GetResult_Response
  ) -> (
    i8,
   super::action::rmw::BackUp_Result,
  ) {
    (response.status, response.result)
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__ComputePathToPose() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__ComputePathToPose
#[allow(missing_docs, non_camel_case_types)]
pub struct ComputePathToPose;

impl rosidl_runtime_rs::Action for ComputePathToPose {
  // --- Associated types for client library users ---
  /// The goal message defined in the action definition.
  type Goal = ComputePathToPose_Goal;

  /// The result message defined in the action definition.
  type Result = ComputePathToPose_Result;

  /// The feedback message defined in the action definition.
  type Feedback = ComputePathToPose_Feedback;

  // --- Associated types for client library implementation ---
  /// The feedback message with generic fields which wraps the feedback message.
  type FeedbackMessage = super::action::ComputePathToPose_FeedbackMessage;

  /// The send_goal service using a wrapped version of the goal message as a request.
  type SendGoalService = super::action::ComputePathToPose_SendGoal;

  /// The generic service to cancel a goal.
  type CancelGoalService = action_msgs::srv::rmw::CancelGoal;

  /// The get_result service using a wrapped version of the result message as a response.
  type GetResultService = super::action::ComputePathToPose_GetResult;

  // --- Methods for client library implementation ---
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__ComputePathToPose() }
  }

  fn create_goal_request(
    goal_id: &[u8; 16],
    goal: super::action::rmw::ComputePathToPose_Goal,
  ) -> super::action::rmw::ComputePathToPose_SendGoal_Request {
   super::action::rmw::ComputePathToPose_SendGoal_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
      goal,
    }
  }

  fn split_goal_request(
    request: super::action::rmw::ComputePathToPose_SendGoal_Request,
  ) -> (
    [u8; 16],
   super::action::rmw::ComputePathToPose_Goal,
  ) {
    (request.goal_id.uuid, request.goal)
  }

  fn create_goal_response(
    accepted: bool,
    stamp: (i32, u32),
  ) -> super::action::rmw::ComputePathToPose_SendGoal_Response {
   super::action::rmw::ComputePathToPose_SendGoal_Response {
      accepted,
      stamp: builtin_interfaces::msg::rmw::Time {
        sec: stamp.0,
        nanosec: stamp.1,
      },
    }
  }

  fn get_goal_response_accepted(
    response: &super::action::rmw::ComputePathToPose_SendGoal_Response,
  ) -> bool {
    response.accepted
  }

  fn get_goal_response_stamp(
    response: &super::action::rmw::ComputePathToPose_SendGoal_Response,
  ) -> (i32, u32) {
    (response.stamp.sec, response.stamp.nanosec)
  }

  fn create_feedback_message(
    goal_id: &[u8; 16],
    feedback: super::action::rmw::ComputePathToPose_Feedback,
  ) -> super::action::rmw::ComputePathToPose_FeedbackMessage {
    let mut message = super::action::rmw::ComputePathToPose_FeedbackMessage::default();
    message.goal_id.uuid = *goal_id;
    message.feedback = feedback;
    message
  }

  fn split_feedback_message(
    feedback: super::action::rmw::ComputePathToPose_FeedbackMessage,
  ) -> (
    [u8; 16],
   super::action::rmw::ComputePathToPose_Feedback,
  ) {
    (feedback.goal_id.uuid, feedback.feedback)
  }

  fn create_result_request(
    goal_id: &[u8; 16],
  ) -> super::action::rmw::ComputePathToPose_GetResult_Request {
   super::action::rmw::ComputePathToPose_GetResult_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
    }
  }

  fn get_result_request_uuid(
    request: &super::action::rmw::ComputePathToPose_GetResult_Request,
  ) -> &[u8; 16] {
    &request.goal_id.uuid
  }

  fn create_result_response(
    status: i8,
    result: super::action::rmw::ComputePathToPose_Result,
  ) -> super::action::rmw::ComputePathToPose_GetResult_Response {
   super::action::rmw::ComputePathToPose_GetResult_Response {
      status,
      result,
    }
  }

  fn split_result_response(
    response: super::action::rmw::ComputePathToPose_GetResult_Response
  ) -> (
    i8,
   super::action::rmw::ComputePathToPose_Result,
  ) {
    (response.status, response.result)
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__ComputePathThroughPoses() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__ComputePathThroughPoses
#[allow(missing_docs, non_camel_case_types)]
pub struct ComputePathThroughPoses;

impl rosidl_runtime_rs::Action for ComputePathThroughPoses {
  // --- Associated types for client library users ---
  /// The goal message defined in the action definition.
  type Goal = ComputePathThroughPoses_Goal;

  /// The result message defined in the action definition.
  type Result = ComputePathThroughPoses_Result;

  /// The feedback message defined in the action definition.
  type Feedback = ComputePathThroughPoses_Feedback;

  // --- Associated types for client library implementation ---
  /// The feedback message with generic fields which wraps the feedback message.
  type FeedbackMessage = super::action::ComputePathThroughPoses_FeedbackMessage;

  /// The send_goal service using a wrapped version of the goal message as a request.
  type SendGoalService = super::action::ComputePathThroughPoses_SendGoal;

  /// The generic service to cancel a goal.
  type CancelGoalService = action_msgs::srv::rmw::CancelGoal;

  /// The get_result service using a wrapped version of the result message as a response.
  type GetResultService = super::action::ComputePathThroughPoses_GetResult;

  // --- Methods for client library implementation ---
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__ComputePathThroughPoses() }
  }

  fn create_goal_request(
    goal_id: &[u8; 16],
    goal: super::action::rmw::ComputePathThroughPoses_Goal,
  ) -> super::action::rmw::ComputePathThroughPoses_SendGoal_Request {
   super::action::rmw::ComputePathThroughPoses_SendGoal_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
      goal,
    }
  }

  fn split_goal_request(
    request: super::action::rmw::ComputePathThroughPoses_SendGoal_Request,
  ) -> (
    [u8; 16],
   super::action::rmw::ComputePathThroughPoses_Goal,
  ) {
    (request.goal_id.uuid, request.goal)
  }

  fn create_goal_response(
    accepted: bool,
    stamp: (i32, u32),
  ) -> super::action::rmw::ComputePathThroughPoses_SendGoal_Response {
   super::action::rmw::ComputePathThroughPoses_SendGoal_Response {
      accepted,
      stamp: builtin_interfaces::msg::rmw::Time {
        sec: stamp.0,
        nanosec: stamp.1,
      },
    }
  }

  fn get_goal_response_accepted(
    response: &super::action::rmw::ComputePathThroughPoses_SendGoal_Response,
  ) -> bool {
    response.accepted
  }

  fn get_goal_response_stamp(
    response: &super::action::rmw::ComputePathThroughPoses_SendGoal_Response,
  ) -> (i32, u32) {
    (response.stamp.sec, response.stamp.nanosec)
  }

  fn create_feedback_message(
    goal_id: &[u8; 16],
    feedback: super::action::rmw::ComputePathThroughPoses_Feedback,
  ) -> super::action::rmw::ComputePathThroughPoses_FeedbackMessage {
    let mut message = super::action::rmw::ComputePathThroughPoses_FeedbackMessage::default();
    message.goal_id.uuid = *goal_id;
    message.feedback = feedback;
    message
  }

  fn split_feedback_message(
    feedback: super::action::rmw::ComputePathThroughPoses_FeedbackMessage,
  ) -> (
    [u8; 16],
   super::action::rmw::ComputePathThroughPoses_Feedback,
  ) {
    (feedback.goal_id.uuid, feedback.feedback)
  }

  fn create_result_request(
    goal_id: &[u8; 16],
  ) -> super::action::rmw::ComputePathThroughPoses_GetResult_Request {
   super::action::rmw::ComputePathThroughPoses_GetResult_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
    }
  }

  fn get_result_request_uuid(
    request: &super::action::rmw::ComputePathThroughPoses_GetResult_Request,
  ) -> &[u8; 16] {
    &request.goal_id.uuid
  }

  fn create_result_response(
    status: i8,
    result: super::action::rmw::ComputePathThroughPoses_Result,
  ) -> super::action::rmw::ComputePathThroughPoses_GetResult_Response {
   super::action::rmw::ComputePathThroughPoses_GetResult_Response {
      status,
      result,
    }
  }

  fn split_result_response(
    response: super::action::rmw::ComputePathThroughPoses_GetResult_Response
  ) -> (
    i8,
   super::action::rmw::ComputePathThroughPoses_Result,
  ) {
    (response.status, response.result)
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__ComputeRoute() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__ComputeRoute
#[allow(missing_docs, non_camel_case_types)]
pub struct ComputeRoute;

impl rosidl_runtime_rs::Action for ComputeRoute {
  // --- Associated types for client library users ---
  /// The goal message defined in the action definition.
  type Goal = ComputeRoute_Goal;

  /// The result message defined in the action definition.
  type Result = ComputeRoute_Result;

  /// The feedback message defined in the action definition.
  type Feedback = ComputeRoute_Feedback;

  // --- Associated types for client library implementation ---
  /// The feedback message with generic fields which wraps the feedback message.
  type FeedbackMessage = super::action::ComputeRoute_FeedbackMessage;

  /// The send_goal service using a wrapped version of the goal message as a request.
  type SendGoalService = super::action::ComputeRoute_SendGoal;

  /// The generic service to cancel a goal.
  type CancelGoalService = action_msgs::srv::rmw::CancelGoal;

  /// The get_result service using a wrapped version of the result message as a response.
  type GetResultService = super::action::ComputeRoute_GetResult;

  // --- Methods for client library implementation ---
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__ComputeRoute() }
  }

  fn create_goal_request(
    goal_id: &[u8; 16],
    goal: super::action::rmw::ComputeRoute_Goal,
  ) -> super::action::rmw::ComputeRoute_SendGoal_Request {
   super::action::rmw::ComputeRoute_SendGoal_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
      goal,
    }
  }

  fn split_goal_request(
    request: super::action::rmw::ComputeRoute_SendGoal_Request,
  ) -> (
    [u8; 16],
   super::action::rmw::ComputeRoute_Goal,
  ) {
    (request.goal_id.uuid, request.goal)
  }

  fn create_goal_response(
    accepted: bool,
    stamp: (i32, u32),
  ) -> super::action::rmw::ComputeRoute_SendGoal_Response {
   super::action::rmw::ComputeRoute_SendGoal_Response {
      accepted,
      stamp: builtin_interfaces::msg::rmw::Time {
        sec: stamp.0,
        nanosec: stamp.1,
      },
    }
  }

  fn get_goal_response_accepted(
    response: &super::action::rmw::ComputeRoute_SendGoal_Response,
  ) -> bool {
    response.accepted
  }

  fn get_goal_response_stamp(
    response: &super::action::rmw::ComputeRoute_SendGoal_Response,
  ) -> (i32, u32) {
    (response.stamp.sec, response.stamp.nanosec)
  }

  fn create_feedback_message(
    goal_id: &[u8; 16],
    feedback: super::action::rmw::ComputeRoute_Feedback,
  ) -> super::action::rmw::ComputeRoute_FeedbackMessage {
    let mut message = super::action::rmw::ComputeRoute_FeedbackMessage::default();
    message.goal_id.uuid = *goal_id;
    message.feedback = feedback;
    message
  }

  fn split_feedback_message(
    feedback: super::action::rmw::ComputeRoute_FeedbackMessage,
  ) -> (
    [u8; 16],
   super::action::rmw::ComputeRoute_Feedback,
  ) {
    (feedback.goal_id.uuid, feedback.feedback)
  }

  fn create_result_request(
    goal_id: &[u8; 16],
  ) -> super::action::rmw::ComputeRoute_GetResult_Request {
   super::action::rmw::ComputeRoute_GetResult_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
    }
  }

  fn get_result_request_uuid(
    request: &super::action::rmw::ComputeRoute_GetResult_Request,
  ) -> &[u8; 16] {
    &request.goal_id.uuid
  }

  fn create_result_response(
    status: i8,
    result: super::action::rmw::ComputeRoute_Result,
  ) -> super::action::rmw::ComputeRoute_GetResult_Response {
   super::action::rmw::ComputeRoute_GetResult_Response {
      status,
      result,
    }
  }

  fn split_result_response(
    response: super::action::rmw::ComputeRoute_GetResult_Response
  ) -> (
    i8,
   super::action::rmw::ComputeRoute_Result,
  ) {
    (response.status, response.result)
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__ComputeAndTrackRoute() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__ComputeAndTrackRoute
#[allow(missing_docs, non_camel_case_types)]
pub struct ComputeAndTrackRoute;

impl rosidl_runtime_rs::Action for ComputeAndTrackRoute {
  // --- Associated types for client library users ---
  /// The goal message defined in the action definition.
  type Goal = ComputeAndTrackRoute_Goal;

  /// The result message defined in the action definition.
  type Result = ComputeAndTrackRoute_Result;

  /// The feedback message defined in the action definition.
  type Feedback = ComputeAndTrackRoute_Feedback;

  // --- Associated types for client library implementation ---
  /// The feedback message with generic fields which wraps the feedback message.
  type FeedbackMessage = super::action::ComputeAndTrackRoute_FeedbackMessage;

  /// The send_goal service using a wrapped version of the goal message as a request.
  type SendGoalService = super::action::ComputeAndTrackRoute_SendGoal;

  /// The generic service to cancel a goal.
  type CancelGoalService = action_msgs::srv::rmw::CancelGoal;

  /// The get_result service using a wrapped version of the result message as a response.
  type GetResultService = super::action::ComputeAndTrackRoute_GetResult;

  // --- Methods for client library implementation ---
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__ComputeAndTrackRoute() }
  }

  fn create_goal_request(
    goal_id: &[u8; 16],
    goal: super::action::rmw::ComputeAndTrackRoute_Goal,
  ) -> super::action::rmw::ComputeAndTrackRoute_SendGoal_Request {
   super::action::rmw::ComputeAndTrackRoute_SendGoal_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
      goal,
    }
  }

  fn split_goal_request(
    request: super::action::rmw::ComputeAndTrackRoute_SendGoal_Request,
  ) -> (
    [u8; 16],
   super::action::rmw::ComputeAndTrackRoute_Goal,
  ) {
    (request.goal_id.uuid, request.goal)
  }

  fn create_goal_response(
    accepted: bool,
    stamp: (i32, u32),
  ) -> super::action::rmw::ComputeAndTrackRoute_SendGoal_Response {
   super::action::rmw::ComputeAndTrackRoute_SendGoal_Response {
      accepted,
      stamp: builtin_interfaces::msg::rmw::Time {
        sec: stamp.0,
        nanosec: stamp.1,
      },
    }
  }

  fn get_goal_response_accepted(
    response: &super::action::rmw::ComputeAndTrackRoute_SendGoal_Response,
  ) -> bool {
    response.accepted
  }

  fn get_goal_response_stamp(
    response: &super::action::rmw::ComputeAndTrackRoute_SendGoal_Response,
  ) -> (i32, u32) {
    (response.stamp.sec, response.stamp.nanosec)
  }

  fn create_feedback_message(
    goal_id: &[u8; 16],
    feedback: super::action::rmw::ComputeAndTrackRoute_Feedback,
  ) -> super::action::rmw::ComputeAndTrackRoute_FeedbackMessage {
    let mut message = super::action::rmw::ComputeAndTrackRoute_FeedbackMessage::default();
    message.goal_id.uuid = *goal_id;
    message.feedback = feedback;
    message
  }

  fn split_feedback_message(
    feedback: super::action::rmw::ComputeAndTrackRoute_FeedbackMessage,
  ) -> (
    [u8; 16],
   super::action::rmw::ComputeAndTrackRoute_Feedback,
  ) {
    (feedback.goal_id.uuid, feedback.feedback)
  }

  fn create_result_request(
    goal_id: &[u8; 16],
  ) -> super::action::rmw::ComputeAndTrackRoute_GetResult_Request {
   super::action::rmw::ComputeAndTrackRoute_GetResult_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
    }
  }

  fn get_result_request_uuid(
    request: &super::action::rmw::ComputeAndTrackRoute_GetResult_Request,
  ) -> &[u8; 16] {
    &request.goal_id.uuid
  }

  fn create_result_response(
    status: i8,
    result: super::action::rmw::ComputeAndTrackRoute_Result,
  ) -> super::action::rmw::ComputeAndTrackRoute_GetResult_Response {
   super::action::rmw::ComputeAndTrackRoute_GetResult_Response {
      status,
      result,
    }
  }

  fn split_result_response(
    response: super::action::rmw::ComputeAndTrackRoute_GetResult_Response
  ) -> (
    i8,
   super::action::rmw::ComputeAndTrackRoute_Result,
  ) {
    (response.status, response.result)
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__DriveOnHeading() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__DriveOnHeading
#[allow(missing_docs, non_camel_case_types)]
pub struct DriveOnHeading;

impl rosidl_runtime_rs::Action for DriveOnHeading {
  // --- Associated types for client library users ---
  /// The goal message defined in the action definition.
  type Goal = DriveOnHeading_Goal;

  /// The result message defined in the action definition.
  type Result = DriveOnHeading_Result;

  /// The feedback message defined in the action definition.
  type Feedback = DriveOnHeading_Feedback;

  // --- Associated types for client library implementation ---
  /// The feedback message with generic fields which wraps the feedback message.
  type FeedbackMessage = super::action::DriveOnHeading_FeedbackMessage;

  /// The send_goal service using a wrapped version of the goal message as a request.
  type SendGoalService = super::action::DriveOnHeading_SendGoal;

  /// The generic service to cancel a goal.
  type CancelGoalService = action_msgs::srv::rmw::CancelGoal;

  /// The get_result service using a wrapped version of the result message as a response.
  type GetResultService = super::action::DriveOnHeading_GetResult;

  // --- Methods for client library implementation ---
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__DriveOnHeading() }
  }

  fn create_goal_request(
    goal_id: &[u8; 16],
    goal: super::action::rmw::DriveOnHeading_Goal,
  ) -> super::action::rmw::DriveOnHeading_SendGoal_Request {
   super::action::rmw::DriveOnHeading_SendGoal_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
      goal,
    }
  }

  fn split_goal_request(
    request: super::action::rmw::DriveOnHeading_SendGoal_Request,
  ) -> (
    [u8; 16],
   super::action::rmw::DriveOnHeading_Goal,
  ) {
    (request.goal_id.uuid, request.goal)
  }

  fn create_goal_response(
    accepted: bool,
    stamp: (i32, u32),
  ) -> super::action::rmw::DriveOnHeading_SendGoal_Response {
   super::action::rmw::DriveOnHeading_SendGoal_Response {
      accepted,
      stamp: builtin_interfaces::msg::rmw::Time {
        sec: stamp.0,
        nanosec: stamp.1,
      },
    }
  }

  fn get_goal_response_accepted(
    response: &super::action::rmw::DriveOnHeading_SendGoal_Response,
  ) -> bool {
    response.accepted
  }

  fn get_goal_response_stamp(
    response: &super::action::rmw::DriveOnHeading_SendGoal_Response,
  ) -> (i32, u32) {
    (response.stamp.sec, response.stamp.nanosec)
  }

  fn create_feedback_message(
    goal_id: &[u8; 16],
    feedback: super::action::rmw::DriveOnHeading_Feedback,
  ) -> super::action::rmw::DriveOnHeading_FeedbackMessage {
    let mut message = super::action::rmw::DriveOnHeading_FeedbackMessage::default();
    message.goal_id.uuid = *goal_id;
    message.feedback = feedback;
    message
  }

  fn split_feedback_message(
    feedback: super::action::rmw::DriveOnHeading_FeedbackMessage,
  ) -> (
    [u8; 16],
   super::action::rmw::DriveOnHeading_Feedback,
  ) {
    (feedback.goal_id.uuid, feedback.feedback)
  }

  fn create_result_request(
    goal_id: &[u8; 16],
  ) -> super::action::rmw::DriveOnHeading_GetResult_Request {
   super::action::rmw::DriveOnHeading_GetResult_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
    }
  }

  fn get_result_request_uuid(
    request: &super::action::rmw::DriveOnHeading_GetResult_Request,
  ) -> &[u8; 16] {
    &request.goal_id.uuid
  }

  fn create_result_response(
    status: i8,
    result: super::action::rmw::DriveOnHeading_Result,
  ) -> super::action::rmw::DriveOnHeading_GetResult_Response {
   super::action::rmw::DriveOnHeading_GetResult_Response {
      status,
      result,
    }
  }

  fn split_result_response(
    response: super::action::rmw::DriveOnHeading_GetResult_Response
  ) -> (
    i8,
   super::action::rmw::DriveOnHeading_Result,
  ) {
    (response.status, response.result)
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__SmoothPath() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__SmoothPath
#[allow(missing_docs, non_camel_case_types)]
pub struct SmoothPath;

impl rosidl_runtime_rs::Action for SmoothPath {
  // --- Associated types for client library users ---
  /// The goal message defined in the action definition.
  type Goal = SmoothPath_Goal;

  /// The result message defined in the action definition.
  type Result = SmoothPath_Result;

  /// The feedback message defined in the action definition.
  type Feedback = SmoothPath_Feedback;

  // --- Associated types for client library implementation ---
  /// The feedback message with generic fields which wraps the feedback message.
  type FeedbackMessage = super::action::SmoothPath_FeedbackMessage;

  /// The send_goal service using a wrapped version of the goal message as a request.
  type SendGoalService = super::action::SmoothPath_SendGoal;

  /// The generic service to cancel a goal.
  type CancelGoalService = action_msgs::srv::rmw::CancelGoal;

  /// The get_result service using a wrapped version of the result message as a response.
  type GetResultService = super::action::SmoothPath_GetResult;

  // --- Methods for client library implementation ---
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__SmoothPath() }
  }

  fn create_goal_request(
    goal_id: &[u8; 16],
    goal: super::action::rmw::SmoothPath_Goal,
  ) -> super::action::rmw::SmoothPath_SendGoal_Request {
   super::action::rmw::SmoothPath_SendGoal_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
      goal,
    }
  }

  fn split_goal_request(
    request: super::action::rmw::SmoothPath_SendGoal_Request,
  ) -> (
    [u8; 16],
   super::action::rmw::SmoothPath_Goal,
  ) {
    (request.goal_id.uuid, request.goal)
  }

  fn create_goal_response(
    accepted: bool,
    stamp: (i32, u32),
  ) -> super::action::rmw::SmoothPath_SendGoal_Response {
   super::action::rmw::SmoothPath_SendGoal_Response {
      accepted,
      stamp: builtin_interfaces::msg::rmw::Time {
        sec: stamp.0,
        nanosec: stamp.1,
      },
    }
  }

  fn get_goal_response_accepted(
    response: &super::action::rmw::SmoothPath_SendGoal_Response,
  ) -> bool {
    response.accepted
  }

  fn get_goal_response_stamp(
    response: &super::action::rmw::SmoothPath_SendGoal_Response,
  ) -> (i32, u32) {
    (response.stamp.sec, response.stamp.nanosec)
  }

  fn create_feedback_message(
    goal_id: &[u8; 16],
    feedback: super::action::rmw::SmoothPath_Feedback,
  ) -> super::action::rmw::SmoothPath_FeedbackMessage {
    let mut message = super::action::rmw::SmoothPath_FeedbackMessage::default();
    message.goal_id.uuid = *goal_id;
    message.feedback = feedback;
    message
  }

  fn split_feedback_message(
    feedback: super::action::rmw::SmoothPath_FeedbackMessage,
  ) -> (
    [u8; 16],
   super::action::rmw::SmoothPath_Feedback,
  ) {
    (feedback.goal_id.uuid, feedback.feedback)
  }

  fn create_result_request(
    goal_id: &[u8; 16],
  ) -> super::action::rmw::SmoothPath_GetResult_Request {
   super::action::rmw::SmoothPath_GetResult_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
    }
  }

  fn get_result_request_uuid(
    request: &super::action::rmw::SmoothPath_GetResult_Request,
  ) -> &[u8; 16] {
    &request.goal_id.uuid
  }

  fn create_result_response(
    status: i8,
    result: super::action::rmw::SmoothPath_Result,
  ) -> super::action::rmw::SmoothPath_GetResult_Response {
   super::action::rmw::SmoothPath_GetResult_Response {
      status,
      result,
    }
  }

  fn split_result_response(
    response: super::action::rmw::SmoothPath_GetResult_Response
  ) -> (
    i8,
   super::action::rmw::SmoothPath_Result,
  ) {
    (response.status, response.result)
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__FollowPath() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__FollowPath
#[allow(missing_docs, non_camel_case_types)]
pub struct FollowPath;

impl rosidl_runtime_rs::Action for FollowPath {
  // --- Associated types for client library users ---
  /// The goal message defined in the action definition.
  type Goal = FollowPath_Goal;

  /// The result message defined in the action definition.
  type Result = FollowPath_Result;

  /// The feedback message defined in the action definition.
  type Feedback = FollowPath_Feedback;

  // --- Associated types for client library implementation ---
  /// The feedback message with generic fields which wraps the feedback message.
  type FeedbackMessage = super::action::FollowPath_FeedbackMessage;

  /// The send_goal service using a wrapped version of the goal message as a request.
  type SendGoalService = super::action::FollowPath_SendGoal;

  /// The generic service to cancel a goal.
  type CancelGoalService = action_msgs::srv::rmw::CancelGoal;

  /// The get_result service using a wrapped version of the result message as a response.
  type GetResultService = super::action::FollowPath_GetResult;

  // --- Methods for client library implementation ---
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__FollowPath() }
  }

  fn create_goal_request(
    goal_id: &[u8; 16],
    goal: super::action::rmw::FollowPath_Goal,
  ) -> super::action::rmw::FollowPath_SendGoal_Request {
   super::action::rmw::FollowPath_SendGoal_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
      goal,
    }
  }

  fn split_goal_request(
    request: super::action::rmw::FollowPath_SendGoal_Request,
  ) -> (
    [u8; 16],
   super::action::rmw::FollowPath_Goal,
  ) {
    (request.goal_id.uuid, request.goal)
  }

  fn create_goal_response(
    accepted: bool,
    stamp: (i32, u32),
  ) -> super::action::rmw::FollowPath_SendGoal_Response {
   super::action::rmw::FollowPath_SendGoal_Response {
      accepted,
      stamp: builtin_interfaces::msg::rmw::Time {
        sec: stamp.0,
        nanosec: stamp.1,
      },
    }
  }

  fn get_goal_response_accepted(
    response: &super::action::rmw::FollowPath_SendGoal_Response,
  ) -> bool {
    response.accepted
  }

  fn get_goal_response_stamp(
    response: &super::action::rmw::FollowPath_SendGoal_Response,
  ) -> (i32, u32) {
    (response.stamp.sec, response.stamp.nanosec)
  }

  fn create_feedback_message(
    goal_id: &[u8; 16],
    feedback: super::action::rmw::FollowPath_Feedback,
  ) -> super::action::rmw::FollowPath_FeedbackMessage {
    let mut message = super::action::rmw::FollowPath_FeedbackMessage::default();
    message.goal_id.uuid = *goal_id;
    message.feedback = feedback;
    message
  }

  fn split_feedback_message(
    feedback: super::action::rmw::FollowPath_FeedbackMessage,
  ) -> (
    [u8; 16],
   super::action::rmw::FollowPath_Feedback,
  ) {
    (feedback.goal_id.uuid, feedback.feedback)
  }

  fn create_result_request(
    goal_id: &[u8; 16],
  ) -> super::action::rmw::FollowPath_GetResult_Request {
   super::action::rmw::FollowPath_GetResult_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
    }
  }

  fn get_result_request_uuid(
    request: &super::action::rmw::FollowPath_GetResult_Request,
  ) -> &[u8; 16] {
    &request.goal_id.uuid
  }

  fn create_result_response(
    status: i8,
    result: super::action::rmw::FollowPath_Result,
  ) -> super::action::rmw::FollowPath_GetResult_Response {
   super::action::rmw::FollowPath_GetResult_Response {
      status,
      result,
    }
  }

  fn split_result_response(
    response: super::action::rmw::FollowPath_GetResult_Response
  ) -> (
    i8,
   super::action::rmw::FollowPath_Result,
  ) {
    (response.status, response.result)
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__NavigateToPose() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__NavigateToPose
#[allow(missing_docs, non_camel_case_types)]
pub struct NavigateToPose;

impl rosidl_runtime_rs::Action for NavigateToPose {
  // --- Associated types for client library users ---
  /// The goal message defined in the action definition.
  type Goal = NavigateToPose_Goal;

  /// The result message defined in the action definition.
  type Result = NavigateToPose_Result;

  /// The feedback message defined in the action definition.
  type Feedback = NavigateToPose_Feedback;

  // --- Associated types for client library implementation ---
  /// The feedback message with generic fields which wraps the feedback message.
  type FeedbackMessage = super::action::NavigateToPose_FeedbackMessage;

  /// The send_goal service using a wrapped version of the goal message as a request.
  type SendGoalService = super::action::NavigateToPose_SendGoal;

  /// The generic service to cancel a goal.
  type CancelGoalService = action_msgs::srv::rmw::CancelGoal;

  /// The get_result service using a wrapped version of the result message as a response.
  type GetResultService = super::action::NavigateToPose_GetResult;

  // --- Methods for client library implementation ---
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__NavigateToPose() }
  }

  fn create_goal_request(
    goal_id: &[u8; 16],
    goal: super::action::rmw::NavigateToPose_Goal,
  ) -> super::action::rmw::NavigateToPose_SendGoal_Request {
   super::action::rmw::NavigateToPose_SendGoal_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
      goal,
    }
  }

  fn split_goal_request(
    request: super::action::rmw::NavigateToPose_SendGoal_Request,
  ) -> (
    [u8; 16],
   super::action::rmw::NavigateToPose_Goal,
  ) {
    (request.goal_id.uuid, request.goal)
  }

  fn create_goal_response(
    accepted: bool,
    stamp: (i32, u32),
  ) -> super::action::rmw::NavigateToPose_SendGoal_Response {
   super::action::rmw::NavigateToPose_SendGoal_Response {
      accepted,
      stamp: builtin_interfaces::msg::rmw::Time {
        sec: stamp.0,
        nanosec: stamp.1,
      },
    }
  }

  fn get_goal_response_accepted(
    response: &super::action::rmw::NavigateToPose_SendGoal_Response,
  ) -> bool {
    response.accepted
  }

  fn get_goal_response_stamp(
    response: &super::action::rmw::NavigateToPose_SendGoal_Response,
  ) -> (i32, u32) {
    (response.stamp.sec, response.stamp.nanosec)
  }

  fn create_feedback_message(
    goal_id: &[u8; 16],
    feedback: super::action::rmw::NavigateToPose_Feedback,
  ) -> super::action::rmw::NavigateToPose_FeedbackMessage {
    let mut message = super::action::rmw::NavigateToPose_FeedbackMessage::default();
    message.goal_id.uuid = *goal_id;
    message.feedback = feedback;
    message
  }

  fn split_feedback_message(
    feedback: super::action::rmw::NavigateToPose_FeedbackMessage,
  ) -> (
    [u8; 16],
   super::action::rmw::NavigateToPose_Feedback,
  ) {
    (feedback.goal_id.uuid, feedback.feedback)
  }

  fn create_result_request(
    goal_id: &[u8; 16],
  ) -> super::action::rmw::NavigateToPose_GetResult_Request {
   super::action::rmw::NavigateToPose_GetResult_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
    }
  }

  fn get_result_request_uuid(
    request: &super::action::rmw::NavigateToPose_GetResult_Request,
  ) -> &[u8; 16] {
    &request.goal_id.uuid
  }

  fn create_result_response(
    status: i8,
    result: super::action::rmw::NavigateToPose_Result,
  ) -> super::action::rmw::NavigateToPose_GetResult_Response {
   super::action::rmw::NavigateToPose_GetResult_Response {
      status,
      result,
    }
  }

  fn split_result_response(
    response: super::action::rmw::NavigateToPose_GetResult_Response
  ) -> (
    i8,
   super::action::rmw::NavigateToPose_Result,
  ) {
    (response.status, response.result)
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__NavigateThroughPoses() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__NavigateThroughPoses
#[allow(missing_docs, non_camel_case_types)]
pub struct NavigateThroughPoses;

impl rosidl_runtime_rs::Action for NavigateThroughPoses {
  // --- Associated types for client library users ---
  /// The goal message defined in the action definition.
  type Goal = NavigateThroughPoses_Goal;

  /// The result message defined in the action definition.
  type Result = NavigateThroughPoses_Result;

  /// The feedback message defined in the action definition.
  type Feedback = NavigateThroughPoses_Feedback;

  // --- Associated types for client library implementation ---
  /// The feedback message with generic fields which wraps the feedback message.
  type FeedbackMessage = super::action::NavigateThroughPoses_FeedbackMessage;

  /// The send_goal service using a wrapped version of the goal message as a request.
  type SendGoalService = super::action::NavigateThroughPoses_SendGoal;

  /// The generic service to cancel a goal.
  type CancelGoalService = action_msgs::srv::rmw::CancelGoal;

  /// The get_result service using a wrapped version of the result message as a response.
  type GetResultService = super::action::NavigateThroughPoses_GetResult;

  // --- Methods for client library implementation ---
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__NavigateThroughPoses() }
  }

  fn create_goal_request(
    goal_id: &[u8; 16],
    goal: super::action::rmw::NavigateThroughPoses_Goal,
  ) -> super::action::rmw::NavigateThroughPoses_SendGoal_Request {
   super::action::rmw::NavigateThroughPoses_SendGoal_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
      goal,
    }
  }

  fn split_goal_request(
    request: super::action::rmw::NavigateThroughPoses_SendGoal_Request,
  ) -> (
    [u8; 16],
   super::action::rmw::NavigateThroughPoses_Goal,
  ) {
    (request.goal_id.uuid, request.goal)
  }

  fn create_goal_response(
    accepted: bool,
    stamp: (i32, u32),
  ) -> super::action::rmw::NavigateThroughPoses_SendGoal_Response {
   super::action::rmw::NavigateThroughPoses_SendGoal_Response {
      accepted,
      stamp: builtin_interfaces::msg::rmw::Time {
        sec: stamp.0,
        nanosec: stamp.1,
      },
    }
  }

  fn get_goal_response_accepted(
    response: &super::action::rmw::NavigateThroughPoses_SendGoal_Response,
  ) -> bool {
    response.accepted
  }

  fn get_goal_response_stamp(
    response: &super::action::rmw::NavigateThroughPoses_SendGoal_Response,
  ) -> (i32, u32) {
    (response.stamp.sec, response.stamp.nanosec)
  }

  fn create_feedback_message(
    goal_id: &[u8; 16],
    feedback: super::action::rmw::NavigateThroughPoses_Feedback,
  ) -> super::action::rmw::NavigateThroughPoses_FeedbackMessage {
    let mut message = super::action::rmw::NavigateThroughPoses_FeedbackMessage::default();
    message.goal_id.uuid = *goal_id;
    message.feedback = feedback;
    message
  }

  fn split_feedback_message(
    feedback: super::action::rmw::NavigateThroughPoses_FeedbackMessage,
  ) -> (
    [u8; 16],
   super::action::rmw::NavigateThroughPoses_Feedback,
  ) {
    (feedback.goal_id.uuid, feedback.feedback)
  }

  fn create_result_request(
    goal_id: &[u8; 16],
  ) -> super::action::rmw::NavigateThroughPoses_GetResult_Request {
   super::action::rmw::NavigateThroughPoses_GetResult_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
    }
  }

  fn get_result_request_uuid(
    request: &super::action::rmw::NavigateThroughPoses_GetResult_Request,
  ) -> &[u8; 16] {
    &request.goal_id.uuid
  }

  fn create_result_response(
    status: i8,
    result: super::action::rmw::NavigateThroughPoses_Result,
  ) -> super::action::rmw::NavigateThroughPoses_GetResult_Response {
   super::action::rmw::NavigateThroughPoses_GetResult_Response {
      status,
      result,
    }
  }

  fn split_result_response(
    response: super::action::rmw::NavigateThroughPoses_GetResult_Response
  ) -> (
    i8,
   super::action::rmw::NavigateThroughPoses_Result,
  ) {
    (response.status, response.result)
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__Wait() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__Wait
#[allow(missing_docs, non_camel_case_types)]
pub struct Wait;

impl rosidl_runtime_rs::Action for Wait {
  // --- Associated types for client library users ---
  /// The goal message defined in the action definition.
  type Goal = Wait_Goal;

  /// The result message defined in the action definition.
  type Result = Wait_Result;

  /// The feedback message defined in the action definition.
  type Feedback = Wait_Feedback;

  // --- Associated types for client library implementation ---
  /// The feedback message with generic fields which wraps the feedback message.
  type FeedbackMessage = super::action::Wait_FeedbackMessage;

  /// The send_goal service using a wrapped version of the goal message as a request.
  type SendGoalService = super::action::Wait_SendGoal;

  /// The generic service to cancel a goal.
  type CancelGoalService = action_msgs::srv::rmw::CancelGoal;

  /// The get_result service using a wrapped version of the result message as a response.
  type GetResultService = super::action::Wait_GetResult;

  // --- Methods for client library implementation ---
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__Wait() }
  }

  fn create_goal_request(
    goal_id: &[u8; 16],
    goal: super::action::rmw::Wait_Goal,
  ) -> super::action::rmw::Wait_SendGoal_Request {
   super::action::rmw::Wait_SendGoal_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
      goal,
    }
  }

  fn split_goal_request(
    request: super::action::rmw::Wait_SendGoal_Request,
  ) -> (
    [u8; 16],
   super::action::rmw::Wait_Goal,
  ) {
    (request.goal_id.uuid, request.goal)
  }

  fn create_goal_response(
    accepted: bool,
    stamp: (i32, u32),
  ) -> super::action::rmw::Wait_SendGoal_Response {
   super::action::rmw::Wait_SendGoal_Response {
      accepted,
      stamp: builtin_interfaces::msg::rmw::Time {
        sec: stamp.0,
        nanosec: stamp.1,
      },
    }
  }

  fn get_goal_response_accepted(
    response: &super::action::rmw::Wait_SendGoal_Response,
  ) -> bool {
    response.accepted
  }

  fn get_goal_response_stamp(
    response: &super::action::rmw::Wait_SendGoal_Response,
  ) -> (i32, u32) {
    (response.stamp.sec, response.stamp.nanosec)
  }

  fn create_feedback_message(
    goal_id: &[u8; 16],
    feedback: super::action::rmw::Wait_Feedback,
  ) -> super::action::rmw::Wait_FeedbackMessage {
    let mut message = super::action::rmw::Wait_FeedbackMessage::default();
    message.goal_id.uuid = *goal_id;
    message.feedback = feedback;
    message
  }

  fn split_feedback_message(
    feedback: super::action::rmw::Wait_FeedbackMessage,
  ) -> (
    [u8; 16],
   super::action::rmw::Wait_Feedback,
  ) {
    (feedback.goal_id.uuid, feedback.feedback)
  }

  fn create_result_request(
    goal_id: &[u8; 16],
  ) -> super::action::rmw::Wait_GetResult_Request {
   super::action::rmw::Wait_GetResult_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
    }
  }

  fn get_result_request_uuid(
    request: &super::action::rmw::Wait_GetResult_Request,
  ) -> &[u8; 16] {
    &request.goal_id.uuid
  }

  fn create_result_response(
    status: i8,
    result: super::action::rmw::Wait_Result,
  ) -> super::action::rmw::Wait_GetResult_Response {
   super::action::rmw::Wait_GetResult_Response {
      status,
      result,
    }
  }

  fn split_result_response(
    response: super::action::rmw::Wait_GetResult_Response
  ) -> (
    i8,
   super::action::rmw::Wait_Result,
  ) {
    (response.status, response.result)
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__Spin() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__Spin
#[allow(missing_docs, non_camel_case_types)]
pub struct Spin;

impl rosidl_runtime_rs::Action for Spin {
  // --- Associated types for client library users ---
  /// The goal message defined in the action definition.
  type Goal = Spin_Goal;

  /// The result message defined in the action definition.
  type Result = Spin_Result;

  /// The feedback message defined in the action definition.
  type Feedback = Spin_Feedback;

  // --- Associated types for client library implementation ---
  /// The feedback message with generic fields which wraps the feedback message.
  type FeedbackMessage = super::action::Spin_FeedbackMessage;

  /// The send_goal service using a wrapped version of the goal message as a request.
  type SendGoalService = super::action::Spin_SendGoal;

  /// The generic service to cancel a goal.
  type CancelGoalService = action_msgs::srv::rmw::CancelGoal;

  /// The get_result service using a wrapped version of the result message as a response.
  type GetResultService = super::action::Spin_GetResult;

  // --- Methods for client library implementation ---
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__Spin() }
  }

  fn create_goal_request(
    goal_id: &[u8; 16],
    goal: super::action::rmw::Spin_Goal,
  ) -> super::action::rmw::Spin_SendGoal_Request {
   super::action::rmw::Spin_SendGoal_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
      goal,
    }
  }

  fn split_goal_request(
    request: super::action::rmw::Spin_SendGoal_Request,
  ) -> (
    [u8; 16],
   super::action::rmw::Spin_Goal,
  ) {
    (request.goal_id.uuid, request.goal)
  }

  fn create_goal_response(
    accepted: bool,
    stamp: (i32, u32),
  ) -> super::action::rmw::Spin_SendGoal_Response {
   super::action::rmw::Spin_SendGoal_Response {
      accepted,
      stamp: builtin_interfaces::msg::rmw::Time {
        sec: stamp.0,
        nanosec: stamp.1,
      },
    }
  }

  fn get_goal_response_accepted(
    response: &super::action::rmw::Spin_SendGoal_Response,
  ) -> bool {
    response.accepted
  }

  fn get_goal_response_stamp(
    response: &super::action::rmw::Spin_SendGoal_Response,
  ) -> (i32, u32) {
    (response.stamp.sec, response.stamp.nanosec)
  }

  fn create_feedback_message(
    goal_id: &[u8; 16],
    feedback: super::action::rmw::Spin_Feedback,
  ) -> super::action::rmw::Spin_FeedbackMessage {
    let mut message = super::action::rmw::Spin_FeedbackMessage::default();
    message.goal_id.uuid = *goal_id;
    message.feedback = feedback;
    message
  }

  fn split_feedback_message(
    feedback: super::action::rmw::Spin_FeedbackMessage,
  ) -> (
    [u8; 16],
   super::action::rmw::Spin_Feedback,
  ) {
    (feedback.goal_id.uuid, feedback.feedback)
  }

  fn create_result_request(
    goal_id: &[u8; 16],
  ) -> super::action::rmw::Spin_GetResult_Request {
   super::action::rmw::Spin_GetResult_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
    }
  }

  fn get_result_request_uuid(
    request: &super::action::rmw::Spin_GetResult_Request,
  ) -> &[u8; 16] {
    &request.goal_id.uuid
  }

  fn create_result_response(
    status: i8,
    result: super::action::rmw::Spin_Result,
  ) -> super::action::rmw::Spin_GetResult_Response {
   super::action::rmw::Spin_GetResult_Response {
      status,
      result,
    }
  }

  fn split_result_response(
    response: super::action::rmw::Spin_GetResult_Response
  ) -> (
    i8,
   super::action::rmw::Spin_Result,
  ) {
    (response.status, response.result)
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__DummyBehavior() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__DummyBehavior
#[allow(missing_docs, non_camel_case_types)]
pub struct DummyBehavior;

impl rosidl_runtime_rs::Action for DummyBehavior {
  // --- Associated types for client library users ---
  /// The goal message defined in the action definition.
  type Goal = DummyBehavior_Goal;

  /// The result message defined in the action definition.
  type Result = DummyBehavior_Result;

  /// The feedback message defined in the action definition.
  type Feedback = DummyBehavior_Feedback;

  // --- Associated types for client library implementation ---
  /// The feedback message with generic fields which wraps the feedback message.
  type FeedbackMessage = super::action::DummyBehavior_FeedbackMessage;

  /// The send_goal service using a wrapped version of the goal message as a request.
  type SendGoalService = super::action::DummyBehavior_SendGoal;

  /// The generic service to cancel a goal.
  type CancelGoalService = action_msgs::srv::rmw::CancelGoal;

  /// The get_result service using a wrapped version of the result message as a response.
  type GetResultService = super::action::DummyBehavior_GetResult;

  // --- Methods for client library implementation ---
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__DummyBehavior() }
  }

  fn create_goal_request(
    goal_id: &[u8; 16],
    goal: super::action::rmw::DummyBehavior_Goal,
  ) -> super::action::rmw::DummyBehavior_SendGoal_Request {
   super::action::rmw::DummyBehavior_SendGoal_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
      goal,
    }
  }

  fn split_goal_request(
    request: super::action::rmw::DummyBehavior_SendGoal_Request,
  ) -> (
    [u8; 16],
   super::action::rmw::DummyBehavior_Goal,
  ) {
    (request.goal_id.uuid, request.goal)
  }

  fn create_goal_response(
    accepted: bool,
    stamp: (i32, u32),
  ) -> super::action::rmw::DummyBehavior_SendGoal_Response {
   super::action::rmw::DummyBehavior_SendGoal_Response {
      accepted,
      stamp: builtin_interfaces::msg::rmw::Time {
        sec: stamp.0,
        nanosec: stamp.1,
      },
    }
  }

  fn get_goal_response_accepted(
    response: &super::action::rmw::DummyBehavior_SendGoal_Response,
  ) -> bool {
    response.accepted
  }

  fn get_goal_response_stamp(
    response: &super::action::rmw::DummyBehavior_SendGoal_Response,
  ) -> (i32, u32) {
    (response.stamp.sec, response.stamp.nanosec)
  }

  fn create_feedback_message(
    goal_id: &[u8; 16],
    feedback: super::action::rmw::DummyBehavior_Feedback,
  ) -> super::action::rmw::DummyBehavior_FeedbackMessage {
    let mut message = super::action::rmw::DummyBehavior_FeedbackMessage::default();
    message.goal_id.uuid = *goal_id;
    message.feedback = feedback;
    message
  }

  fn split_feedback_message(
    feedback: super::action::rmw::DummyBehavior_FeedbackMessage,
  ) -> (
    [u8; 16],
   super::action::rmw::DummyBehavior_Feedback,
  ) {
    (feedback.goal_id.uuid, feedback.feedback)
  }

  fn create_result_request(
    goal_id: &[u8; 16],
  ) -> super::action::rmw::DummyBehavior_GetResult_Request {
   super::action::rmw::DummyBehavior_GetResult_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
    }
  }

  fn get_result_request_uuid(
    request: &super::action::rmw::DummyBehavior_GetResult_Request,
  ) -> &[u8; 16] {
    &request.goal_id.uuid
  }

  fn create_result_response(
    status: i8,
    result: super::action::rmw::DummyBehavior_Result,
  ) -> super::action::rmw::DummyBehavior_GetResult_Response {
   super::action::rmw::DummyBehavior_GetResult_Response {
      status,
      result,
    }
  }

  fn split_result_response(
    response: super::action::rmw::DummyBehavior_GetResult_Response
  ) -> (
    i8,
   super::action::rmw::DummyBehavior_Result,
  ) {
    (response.status, response.result)
  }
}




#[link(name = "nav2_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__FollowWaypoints() -> *const std::ffi::c_void;
}

// Corresponds to nav2_msgs__action__FollowWaypoints
#[allow(missing_docs, non_camel_case_types)]
pub struct FollowWaypoints;

impl rosidl_runtime_rs::Action for FollowWaypoints {
  // --- Associated types for client library users ---
  /// The goal message defined in the action definition.
  type Goal = FollowWaypoints_Goal;

  /// The result message defined in the action definition.
  type Result = FollowWaypoints_Result;

  /// The feedback message defined in the action definition.
  type Feedback = FollowWaypoints_Feedback;

  // --- Associated types for client library implementation ---
  /// The feedback message with generic fields which wraps the feedback message.
  type FeedbackMessage = super::action::FollowWaypoints_FeedbackMessage;

  /// The send_goal service using a wrapped version of the goal message as a request.
  type SendGoalService = super::action::FollowWaypoints_SendGoal;

  /// The generic service to cancel a goal.
  type CancelGoalService = action_msgs::srv::rmw::CancelGoal;

  /// The get_result service using a wrapped version of the result message as a response.
  type GetResultService = super::action::FollowWaypoints_GetResult;

  // --- Methods for client library implementation ---
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__FollowWaypoints() }
  }

  fn create_goal_request(
    goal_id: &[u8; 16],
    goal: super::action::rmw::FollowWaypoints_Goal,
  ) -> super::action::rmw::FollowWaypoints_SendGoal_Request {
   super::action::rmw::FollowWaypoints_SendGoal_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
      goal,
    }
  }

  fn split_goal_request(
    request: super::action::rmw::FollowWaypoints_SendGoal_Request,
  ) -> (
    [u8; 16],
   super::action::rmw::FollowWaypoints_Goal,
  ) {
    (request.goal_id.uuid, request.goal)
  }

  fn create_goal_response(
    accepted: bool,
    stamp: (i32, u32),
  ) -> super::action::rmw::FollowWaypoints_SendGoal_Response {
   super::action::rmw::FollowWaypoints_SendGoal_Response {
      accepted,
      stamp: builtin_interfaces::msg::rmw::Time {
        sec: stamp.0,
        nanosec: stamp.1,
      },
    }
  }

  fn get_goal_response_accepted(
    response: &super::action::rmw::FollowWaypoints_SendGoal_Response,
  ) -> bool {
    response.accepted
  }

  fn get_goal_response_stamp(
    response: &super::action::rmw::FollowWaypoints_SendGoal_Response,
  ) -> (i32, u32) {
    (response.stamp.sec, response.stamp.nanosec)
  }

  fn create_feedback_message(
    goal_id: &[u8; 16],
    feedback: super::action::rmw::FollowWaypoints_Feedback,
  ) -> super::action::rmw::FollowWaypoints_FeedbackMessage {
    let mut message = super::action::rmw::FollowWaypoints_FeedbackMessage::default();
    message.goal_id.uuid = *goal_id;
    message.feedback = feedback;
    message
  }

  fn split_feedback_message(
    feedback: super::action::rmw::FollowWaypoints_FeedbackMessage,
  ) -> (
    [u8; 16],
   super::action::rmw::FollowWaypoints_Feedback,
  ) {
    (feedback.goal_id.uuid, feedback.feedback)
  }

  fn create_result_request(
    goal_id: &[u8; 16],
  ) -> super::action::rmw::FollowWaypoints_GetResult_Request {
   super::action::rmw::FollowWaypoints_GetResult_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
    }
  }

  fn get_result_request_uuid(
    request: &super::action::rmw::FollowWaypoints_GetResult_Request,
  ) -> &[u8; 16] {
    &request.goal_id.uuid
  }

  fn create_result_response(
    status: i8,
    result: super::action::rmw::FollowWaypoints_Result,
  ) -> super::action::rmw::FollowWaypoints_GetResult_Response {
   super::action::rmw::FollowWaypoints_GetResult_Response {
      status,
      result,
    }
  }

  fn split_result_response(
    response: super::action::rmw::FollowWaypoints_GetResult_Response
  ) -> (
    i8,
   super::action::rmw::FollowWaypoints_Result,
  ) {
    (response.status, response.result)
  }
}


