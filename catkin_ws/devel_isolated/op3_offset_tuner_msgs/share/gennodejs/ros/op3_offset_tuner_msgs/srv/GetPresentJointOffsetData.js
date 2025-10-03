// Auto-generated. Do not edit!

// (in-package op3_offset_tuner_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

let JointOffsetPositionData = require('../msg/JointOffsetPositionData.js');

//-----------------------------------------------------------

class GetPresentJointOffsetDataRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetPresentJointOffsetDataRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetPresentJointOffsetDataRequest
    let len;
    let data = new GetPresentJointOffsetDataRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'op3_offset_tuner_msgs/GetPresentJointOffsetDataRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetPresentJointOffsetDataRequest(null);
    return resolved;
    }
};

class GetPresentJointOffsetDataResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.present_data_array = null;
    }
    else {
      if (initObj.hasOwnProperty('present_data_array')) {
        this.present_data_array = initObj.present_data_array
      }
      else {
        this.present_data_array = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetPresentJointOffsetDataResponse
    // Serialize message field [present_data_array]
    // Serialize the length for message field [present_data_array]
    bufferOffset = _serializer.uint32(obj.present_data_array.length, buffer, bufferOffset);
    obj.present_data_array.forEach((val) => {
      bufferOffset = JointOffsetPositionData.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetPresentJointOffsetDataResponse
    let len;
    let data = new GetPresentJointOffsetDataResponse(null);
    // Deserialize message field [present_data_array]
    // Deserialize array length for message field [present_data_array]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.present_data_array = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.present_data_array[i] = JointOffsetPositionData.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.present_data_array.forEach((val) => {
      length += JointOffsetPositionData.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'op3_offset_tuner_msgs/GetPresentJointOffsetDataResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fe3cab7abbd49468a5d502064853404e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    JointOffsetPositionData[] present_data_array
    
    ================================================================================
    MSG: op3_offset_tuner_msgs/JointOffsetPositionData
    string  joint_name
    float64 goal_value
    float64 offset_value
    float64 present_value
    int32   p_gain
    int32   i_gain
    int32   d_gain
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetPresentJointOffsetDataResponse(null);
    if (msg.present_data_array !== undefined) {
      resolved.present_data_array = new Array(msg.present_data_array.length);
      for (let i = 0; i < resolved.present_data_array.length; ++i) {
        resolved.present_data_array[i] = JointOffsetPositionData.Resolve(msg.present_data_array[i]);
      }
    }
    else {
      resolved.present_data_array = []
    }

    return resolved;
    }
};

module.exports = {
  Request: GetPresentJointOffsetDataRequest,
  Response: GetPresentJointOffsetDataResponse,
  md5sum() { return 'fe3cab7abbd49468a5d502064853404e'; },
  datatype() { return 'op3_offset_tuner_msgs/GetPresentJointOffsetData'; }
};
