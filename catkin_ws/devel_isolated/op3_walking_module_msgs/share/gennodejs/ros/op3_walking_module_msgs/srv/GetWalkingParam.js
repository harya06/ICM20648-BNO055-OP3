// Auto-generated. Do not edit!

// (in-package op3_walking_module_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

let WalkingParam = require('../msg/WalkingParam.js');

//-----------------------------------------------------------

class GetWalkingParamRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.get_param = null;
    }
    else {
      if (initObj.hasOwnProperty('get_param')) {
        this.get_param = initObj.get_param
      }
      else {
        this.get_param = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetWalkingParamRequest
    // Serialize message field [get_param]
    bufferOffset = _serializer.bool(obj.get_param, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetWalkingParamRequest
    let len;
    let data = new GetWalkingParamRequest(null);
    // Deserialize message field [get_param]
    data.get_param = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'op3_walking_module_msgs/GetWalkingParamRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd964a0520d7b14bf821ec75afc464385';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool            get_param
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetWalkingParamRequest(null);
    if (msg.get_param !== undefined) {
      resolved.get_param = msg.get_param;
    }
    else {
      resolved.get_param = false
    }

    return resolved;
    }
};

class GetWalkingParamResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.parameters = null;
    }
    else {
      if (initObj.hasOwnProperty('parameters')) {
        this.parameters = initObj.parameters
      }
      else {
        this.parameters = new WalkingParam();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetWalkingParamResponse
    // Serialize message field [parameters]
    bufferOffset = WalkingParam.serialize(obj.parameters, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetWalkingParamResponse
    let len;
    let data = new GetWalkingParamResponse(null);
    // Deserialize message field [parameters]
    data.parameters = WalkingParam.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 102;
  }

  static datatype() {
    // Returns string type for a service object
    return 'op3_walking_module_msgs/GetWalkingParamResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '351c78d980e15976e0c1bcf480ef041c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    WalkingParam    parameters
    
    ================================================================================
    MSG: op3_walking_module_msgs/WalkingParam
    ####### walking init pose #######
    float32 init_x_offset
    float32 init_y_offset
    float32 init_z_offset
    float32 init_roll_offset
    float32 init_pitch_offset
    float32 init_yaw_offset
    
    ####### time parameter #####
    float32 period_time
    float32 dsp_ratio
    float32 step_fb_ratio
    
    ########## walking parameter ########
    float32 x_move_amplitude
    float32 y_move_amplitude
    float32 z_move_amplitude
    float32 angle_move_amplitude
    bool move_aim_on
    
    ########## balance parameter ##########
    bool balance_enable
    float32 balance_hip_roll_gain
    float32 balance_knee_gain
    float32 balance_ankle_roll_gain
    float32 balance_ankle_pitch_gain
    float32 y_swap_amplitude
    float32 z_swap_amplitude
    float32 arm_swing_gain
    float32 pelvis_offset
    float32 hip_pitch_offset
    
    ########## gain parameter ##########
    int32 p_gain
    int32 i_gain
    int32 d_gain
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetWalkingParamResponse(null);
    if (msg.parameters !== undefined) {
      resolved.parameters = WalkingParam.Resolve(msg.parameters)
    }
    else {
      resolved.parameters = new WalkingParam()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetWalkingParamRequest,
  Response: GetWalkingParamResponse,
  md5sum() { return 'fa49c3d94c3b9543346b4158f6b0a3fd'; },
  datatype() { return 'op3_walking_module_msgs/GetWalkingParam'; }
};
