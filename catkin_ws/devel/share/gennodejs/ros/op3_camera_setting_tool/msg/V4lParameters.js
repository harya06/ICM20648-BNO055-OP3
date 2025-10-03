// Auto-generated. Do not edit!

// (in-package op3_camera_setting_tool.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let V4lParameter = require('./V4lParameter.js');

//-----------------------------------------------------------

class V4lParameters {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.video_parameter = null;
    }
    else {
      if (initObj.hasOwnProperty('video_parameter')) {
        this.video_parameter = initObj.video_parameter
      }
      else {
        this.video_parameter = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type V4lParameters
    // Serialize message field [video_parameter]
    // Serialize the length for message field [video_parameter]
    bufferOffset = _serializer.uint32(obj.video_parameter.length, buffer, bufferOffset);
    obj.video_parameter.forEach((val) => {
      bufferOffset = V4lParameter.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type V4lParameters
    let len;
    let data = new V4lParameters(null);
    // Deserialize message field [video_parameter]
    // Deserialize array length for message field [video_parameter]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.video_parameter = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.video_parameter[i] = V4lParameter.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.video_parameter.forEach((val) => {
      length += V4lParameter.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'op3_camera_setting_tool/V4lParameters';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'becf7b1c441fba0b43309667fa8477b4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    V4lParameter[]    video_parameter
    ================================================================================
    MSG: op3_camera_setting_tool/V4lParameter
    string  name
    int32   value
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new V4lParameters(null);
    if (msg.video_parameter !== undefined) {
      resolved.video_parameter = new Array(msg.video_parameter.length);
      for (let i = 0; i < resolved.video_parameter.length; ++i) {
        resolved.video_parameter[i] = V4lParameter.Resolve(msg.video_parameter[i]);
      }
    }
    else {
      resolved.video_parameter = []
    }

    return resolved;
    }
};

module.exports = V4lParameters;
