// Auto-generated. Do not edit!

// (in-package op3_online_walking_module_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class PreviewRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.control_cycle = null;
      this.lipm_height = null;
    }
    else {
      if (initObj.hasOwnProperty('control_cycle')) {
        this.control_cycle = initObj.control_cycle
      }
      else {
        this.control_cycle = 0.0;
      }
      if (initObj.hasOwnProperty('lipm_height')) {
        this.lipm_height = initObj.lipm_height
      }
      else {
        this.lipm_height = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PreviewRequest
    // Serialize message field [control_cycle]
    bufferOffset = _serializer.float64(obj.control_cycle, buffer, bufferOffset);
    // Serialize message field [lipm_height]
    bufferOffset = _serializer.float64(obj.lipm_height, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PreviewRequest
    let len;
    let data = new PreviewRequest(null);
    // Deserialize message field [control_cycle]
    data.control_cycle = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [lipm_height]
    data.lipm_height = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'op3_online_walking_module_msgs/PreviewRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5dd8c29c2c1ccae01b0503604675335f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 control_cycle
    float64 lipm_height
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PreviewRequest(null);
    if (msg.control_cycle !== undefined) {
      resolved.control_cycle = msg.control_cycle;
    }
    else {
      resolved.control_cycle = 0.0
    }

    if (msg.lipm_height !== undefined) {
      resolved.lipm_height = msg.lipm_height;
    }
    else {
      resolved.lipm_height = 0.0
    }

    return resolved;
    }
};

module.exports = PreviewRequest;
