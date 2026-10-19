#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};




// Corresponds to nav2_msgs__srv__GetCostmap_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetCostmap_Request {
    /// Specifications for the requested costmap
    pub specs: super::msg::CostmapMetaData,

}



impl Default for GetCostmap_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::GetCostmap_Request::default())
  }
}

impl rosidl_runtime_rs::Message for GetCostmap_Request {
  type RmwMsg = super::srv::rmw::GetCostmap_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        specs: super::msg::CostmapMetaData::into_rmw_message(std::borrow::Cow::Owned(msg.specs)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        specs: super::msg::CostmapMetaData::into_rmw_message(std::borrow::Cow::Borrowed(&msg.specs)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      specs: super::msg::CostmapMetaData::from_rmw_message(msg.specs),
    }
  }
}


// Corresponds to nav2_msgs__srv__GetCostmap_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetCostmap_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub map: super::msg::Costmap,

}



impl Default for GetCostmap_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::GetCostmap_Response::default())
  }
}

impl rosidl_runtime_rs::Message for GetCostmap_Response {
  type RmwMsg = super::srv::rmw::GetCostmap_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        map: super::msg::Costmap::into_rmw_message(std::borrow::Cow::Owned(msg.map)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        map: super::msg::Costmap::into_rmw_message(std::borrow::Cow::Borrowed(&msg.map)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      map: super::msg::Costmap::from_rmw_message(msg.map),
    }
  }
}


// Corresponds to nav2_msgs__srv__IsPathValid_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct IsPathValid_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub path: nav_msgs::msg::Path,

}



impl Default for IsPathValid_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::IsPathValid_Request::default())
  }
}

impl rosidl_runtime_rs::Message for IsPathValid_Request {
  type RmwMsg = super::srv::rmw::IsPathValid_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        path: nav_msgs::msg::Path::into_rmw_message(std::borrow::Cow::Owned(msg.path)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        path: nav_msgs::msg::Path::into_rmw_message(std::borrow::Cow::Borrowed(&msg.path)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      path: nav_msgs::msg::Path::from_rmw_message(msg.path),
    }
  }
}


// Corresponds to nav2_msgs__srv__IsPathValid_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct IsPathValid_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub is_valid: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub invalid_pose_indices: Vec<i32>,

}



impl Default for IsPathValid_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::IsPathValid_Response::default())
  }
}

impl rosidl_runtime_rs::Message for IsPathValid_Response {
  type RmwMsg = super::srv::rmw::IsPathValid_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        is_valid: msg.is_valid,
        invalid_pose_indices: msg.invalid_pose_indices.as_slice().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      is_valid: msg.is_valid,
        invalid_pose_indices: msg.invalid_pose_indices.as_slice().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      is_valid: msg.is_valid,
      invalid_pose_indices: msg.invalid_pose_indices.into(),
    }
  }
}


// Corresponds to nav2_msgs__srv__ClearCostmapExceptRegion_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ClearCostmapExceptRegion_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub reset_distance: f32,

}



impl Default for ClearCostmapExceptRegion_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::ClearCostmapExceptRegion_Request::default())
  }
}

impl rosidl_runtime_rs::Message for ClearCostmapExceptRegion_Request {
  type RmwMsg = super::srv::rmw::ClearCostmapExceptRegion_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        reset_distance: msg.reset_distance,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      reset_distance: msg.reset_distance,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      reset_distance: msg.reset_distance,
    }
  }
}


// Corresponds to nav2_msgs__srv__ClearCostmapExceptRegion_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ClearCostmapExceptRegion_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub response: std_msgs::msg::Empty,

}



impl Default for ClearCostmapExceptRegion_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::ClearCostmapExceptRegion_Response::default())
  }
}

impl rosidl_runtime_rs::Message for ClearCostmapExceptRegion_Response {
  type RmwMsg = super::srv::rmw::ClearCostmapExceptRegion_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        response: std_msgs::msg::Empty::into_rmw_message(std::borrow::Cow::Owned(msg.response)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        response: std_msgs::msg::Empty::into_rmw_message(std::borrow::Cow::Borrowed(&msg.response)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      response: std_msgs::msg::Empty::from_rmw_message(msg.response),
    }
  }
}


// Corresponds to nav2_msgs__srv__ClearCostmapAroundRobot_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ClearCostmapAroundRobot_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub reset_distance: f32,

}



impl Default for ClearCostmapAroundRobot_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::ClearCostmapAroundRobot_Request::default())
  }
}

impl rosidl_runtime_rs::Message for ClearCostmapAroundRobot_Request {
  type RmwMsg = super::srv::rmw::ClearCostmapAroundRobot_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        reset_distance: msg.reset_distance,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      reset_distance: msg.reset_distance,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      reset_distance: msg.reset_distance,
    }
  }
}


// Corresponds to nav2_msgs__srv__ClearCostmapAroundRobot_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ClearCostmapAroundRobot_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub response: std_msgs::msg::Empty,

}



impl Default for ClearCostmapAroundRobot_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::ClearCostmapAroundRobot_Response::default())
  }
}

impl rosidl_runtime_rs::Message for ClearCostmapAroundRobot_Response {
  type RmwMsg = super::srv::rmw::ClearCostmapAroundRobot_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        response: std_msgs::msg::Empty::into_rmw_message(std::borrow::Cow::Owned(msg.response)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        response: std_msgs::msg::Empty::into_rmw_message(std::borrow::Cow::Borrowed(&msg.response)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      response: std_msgs::msg::Empty::from_rmw_message(msg.response),
    }
  }
}


// Corresponds to nav2_msgs__srv__ClearEntireCostmap_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ClearEntireCostmap_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub request: std_msgs::msg::Empty,

}



impl Default for ClearEntireCostmap_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::ClearEntireCostmap_Request::default())
  }
}

impl rosidl_runtime_rs::Message for ClearEntireCostmap_Request {
  type RmwMsg = super::srv::rmw::ClearEntireCostmap_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        request: std_msgs::msg::Empty::into_rmw_message(std::borrow::Cow::Owned(msg.request)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        request: std_msgs::msg::Empty::into_rmw_message(std::borrow::Cow::Borrowed(&msg.request)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      request: std_msgs::msg::Empty::from_rmw_message(msg.request),
    }
  }
}


// Corresponds to nav2_msgs__srv__ClearEntireCostmap_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ClearEntireCostmap_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub response: std_msgs::msg::Empty,

}



impl Default for ClearEntireCostmap_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::ClearEntireCostmap_Response::default())
  }
}

impl rosidl_runtime_rs::Message for ClearEntireCostmap_Response {
  type RmwMsg = super::srv::rmw::ClearEntireCostmap_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        response: std_msgs::msg::Empty::into_rmw_message(std::borrow::Cow::Owned(msg.response)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        response: std_msgs::msg::Empty::into_rmw_message(std::borrow::Cow::Borrowed(&msg.response)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      response: std_msgs::msg::Empty::from_rmw_message(msg.response),
    }
  }
}


// Corresponds to nav2_msgs__srv__DynamicEdges_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DynamicEdges_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub closed_edges: Vec<u16>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub opened_edges: Vec<u16>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub adjust_edges: Vec<super::msg::EdgeCost>,

}



impl Default for DynamicEdges_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::DynamicEdges_Request::default())
  }
}

impl rosidl_runtime_rs::Message for DynamicEdges_Request {
  type RmwMsg = super::srv::rmw::DynamicEdges_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        closed_edges: msg.closed_edges.as_slice().into(),
        opened_edges: msg.opened_edges.as_slice().into(),
        adjust_edges: msg.adjust_edges
          .into_iter()
          .map(|elem| super::msg::EdgeCost::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        closed_edges: msg.closed_edges.as_slice().into(),
        opened_edges: msg.opened_edges.as_slice().into(),
        adjust_edges: msg.adjust_edges
          .iter()
          .map(|elem| super::msg::EdgeCost::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      closed_edges: msg.closed_edges.into(),
      opened_edges: msg.opened_edges.into(),
      adjust_edges: msg.adjust_edges
          .into_iter()
          .map(super::msg::EdgeCost::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to nav2_msgs__srv__DynamicEdges_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DynamicEdges_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,

}



impl Default for DynamicEdges_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::DynamicEdges_Response::default())
  }
}

impl rosidl_runtime_rs::Message for DynamicEdges_Response {
  type RmwMsg = super::srv::rmw::DynamicEdges_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        success: msg.success,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      success: msg.success,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      success: msg.success,
    }
  }
}


// Corresponds to nav2_msgs__srv__ManageLifecycleNodes_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::ManageLifecycleNodes_Request::default())
  }
}

impl rosidl_runtime_rs::Message for ManageLifecycleNodes_Request {
  type RmwMsg = super::srv::rmw::ManageLifecycleNodes_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        command: msg.command,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      command: msg.command,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      command: msg.command,
    }
  }
}


// Corresponds to nav2_msgs__srv__ManageLifecycleNodes_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ManageLifecycleNodes_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,

}



impl Default for ManageLifecycleNodes_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::ManageLifecycleNodes_Response::default())
  }
}

impl rosidl_runtime_rs::Message for ManageLifecycleNodes_Response {
  type RmwMsg = super::srv::rmw::ManageLifecycleNodes_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        success: msg.success,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      success: msg.success,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      success: msg.success,
    }
  }
}


// Corresponds to nav2_msgs__srv__LoadMap_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct LoadMap_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub map_url: std::string::String,

}



impl Default for LoadMap_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::LoadMap_Request::default())
  }
}

impl rosidl_runtime_rs::Message for LoadMap_Request {
  type RmwMsg = super::srv::rmw::LoadMap_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        map_url: msg.map_url.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        map_url: msg.map_url.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      map_url: msg.map_url.to_string(),
    }
  }
}


// Corresponds to nav2_msgs__srv__LoadMap_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct LoadMap_Response {
    /// Returned map is only valid if result equals RESULT_SUCCESS
    pub map: nav_msgs::msg::OccupancyGrid,


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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::LoadMap_Response::default())
  }
}

impl rosidl_runtime_rs::Message for LoadMap_Response {
  type RmwMsg = super::srv::rmw::LoadMap_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        map: nav_msgs::msg::OccupancyGrid::into_rmw_message(std::borrow::Cow::Owned(msg.map)).into_owned(),
        result: msg.result,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        map: nav_msgs::msg::OccupancyGrid::into_rmw_message(std::borrow::Cow::Borrowed(&msg.map)).into_owned(),
      result: msg.result,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      map: nav_msgs::msg::OccupancyGrid::from_rmw_message(msg.map),
      result: msg.result,
    }
  }
}


// Corresponds to nav2_msgs__srv__SaveMap_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SaveMap_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub map_topic: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub map_url: std::string::String,

    /// Constants for image_format. Supported formats: pgm, png, bmp
    pub image_format: std::string::String,

    /// Map modes: trinary, scale or raw
    pub map_mode: std::string::String,

    /// Thresholds. Values in range of
    pub free_thresh: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub occupied_thresh: f32,

}



impl Default for SaveMap_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SaveMap_Request::default())
  }
}

impl rosidl_runtime_rs::Message for SaveMap_Request {
  type RmwMsg = super::srv::rmw::SaveMap_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        map_topic: msg.map_topic.as_str().into(),
        map_url: msg.map_url.as_str().into(),
        image_format: msg.image_format.as_str().into(),
        map_mode: msg.map_mode.as_str().into(),
        free_thresh: msg.free_thresh,
        occupied_thresh: msg.occupied_thresh,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        map_topic: msg.map_topic.as_str().into(),
        map_url: msg.map_url.as_str().into(),
        image_format: msg.image_format.as_str().into(),
        map_mode: msg.map_mode.as_str().into(),
      free_thresh: msg.free_thresh,
      occupied_thresh: msg.occupied_thresh,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      map_topic: msg.map_topic.to_string(),
      map_url: msg.map_url.to_string(),
      image_format: msg.image_format.to_string(),
      map_mode: msg.map_mode.to_string(),
      free_thresh: msg.free_thresh,
      occupied_thresh: msg.occupied_thresh,
    }
  }
}


// Corresponds to nav2_msgs__srv__SaveMap_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SaveMap_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub result: bool,

}



impl Default for SaveMap_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SaveMap_Response::default())
  }
}

impl rosidl_runtime_rs::Message for SaveMap_Response {
  type RmwMsg = super::srv::rmw::SaveMap_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        result: msg.result,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      result: msg.result,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      result: msg.result,
    }
  }
}


// Corresponds to nav2_msgs__srv__SetInitialPose_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetInitialPose_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub pose: geometry_msgs::msg::PoseWithCovarianceStamped,

}



impl Default for SetInitialPose_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SetInitialPose_Request::default())
  }
}

impl rosidl_runtime_rs::Message for SetInitialPose_Request {
  type RmwMsg = super::srv::rmw::SetInitialPose_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        pose: geometry_msgs::msg::PoseWithCovarianceStamped::into_rmw_message(std::borrow::Cow::Owned(msg.pose)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        pose: geometry_msgs::msg::PoseWithCovarianceStamped::into_rmw_message(std::borrow::Cow::Borrowed(&msg.pose)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      pose: geometry_msgs::msg::PoseWithCovarianceStamped::from_rmw_message(msg.pose),
    }
  }
}


// Corresponds to nav2_msgs__srv__SetInitialPose_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetInitialPose_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for SetInitialPose_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SetInitialPose_Response::default())
  }
}

impl rosidl_runtime_rs::Message for SetInitialPose_Response {
  type RmwMsg = super::srv::rmw::SetInitialPose_Response;

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


// Corresponds to nav2_msgs__srv__SetRouteGraph_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetRouteGraph_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub graph_filepath: std::string::String,

}



impl Default for SetRouteGraph_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SetRouteGraph_Request::default())
  }
}

impl rosidl_runtime_rs::Message for SetRouteGraph_Request {
  type RmwMsg = super::srv::rmw::SetRouteGraph_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        graph_filepath: msg.graph_filepath.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        graph_filepath: msg.graph_filepath.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      graph_filepath: msg.graph_filepath.to_string(),
    }
  }
}


// Corresponds to nav2_msgs__srv__SetRouteGraph_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetRouteGraph_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,

}



impl Default for SetRouteGraph_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SetRouteGraph_Response::default())
  }
}

impl rosidl_runtime_rs::Message for SetRouteGraph_Response {
  type RmwMsg = super::srv::rmw::SetRouteGraph_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        success: msg.success,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      success: msg.success,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      success: msg.success,
    }
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


