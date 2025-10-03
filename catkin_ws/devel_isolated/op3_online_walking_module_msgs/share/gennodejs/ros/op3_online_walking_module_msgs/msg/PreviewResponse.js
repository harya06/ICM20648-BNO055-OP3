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

class PreviewResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.K_row = null;
      this.K_col = null;
      this.K = null;
      this.P_row = null;
      this.P_col = null;
      this.P = null;
    }
    else {
      if (initObj.hasOwnProperty('K_row')) {
        this.K_row = initObj.K_row
      }
      else {
        this.K_row = 0;
      }
      if (initObj.hasOwnProperty('K_col')) {
        this.K_col = initObj.K_col
      }
      else {
        this.K_col = 0;
      }
      if (initObj.hasOwnProperty('K')) {
        this.K = initObj.K
      }
      else {
        this.K = [];
      }
      if (initObj.hasOwnProperty('P_row')) {
        this.P_row = initObj.P_row
      }
      else {
        this.P_row = 0;
      }
      if (initObj.hasOwnProperty('P_col')) {
        this.P_col = initObj.P_col
      }
      else {
        this.P_col = 0;
      }
      if (initObj.hasOwnProperty('P')) {
        this.P = initObj.P
      }
      else {
        this.P = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PreviewResponse
    // Serialize message field [K_row]
    bufferOffset = _serializer.int32(obj.K_row, buffer, bufferOffset);
    // Serialize message field [K_col]
    bufferOffset = _serializer.int32(obj.K_col, buffer, bufferOffset);
    // Serialize message field [K]
    bufferOffset = _arraySerializer.float64(obj.K, buffer, bufferOffset, null);
    // Serialize message field [P_row]
    bufferOffset = _serializer.int32(obj.P_row, buffer, bufferOffset);
    // Serialize message field [P_col]
    bufferOffset = _serializer.int32(obj.P_col, buffer, bufferOffset);
    // Serialize message field [P]
    bufferOffset = _arraySerializer.float64(obj.P, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PreviewResponse
    let len;
    let data = new PreviewResponse(null);
    // Deserialize message field [K_row]
    data.K_row = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [K_col]
    data.K_col = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [K]
    data.K = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [P_row]
    data.P_row = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [P_col]
    data.P_col = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [P]
    data.P = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.K.length;
    length += 8 * object.P.length;
    return length + 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'op3_online_walking_module_msgs/PreviewResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '485c4c72e8d24c7f7a770f8a88709eb6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 K_row
    int32 K_col
    float64[] K
    int32 P_row
    int32 P_col
    float64[] P
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PreviewResponse(null);
    if (msg.K_row !== undefined) {
      resolved.K_row = msg.K_row;
    }
    else {
      resolved.K_row = 0
    }

    if (msg.K_col !== undefined) {
      resolved.K_col = msg.K_col;
    }
    else {
      resolved.K_col = 0
    }

    if (msg.K !== undefined) {
      resolved.K = msg.K;
    }
    else {
      resolved.K = []
    }

    if (msg.P_row !== undefined) {
      resolved.P_row = msg.P_row;
    }
    else {
      resolved.P_row = 0
    }

    if (msg.P_col !== undefined) {
      resolved.P_col = msg.P_col;
    }
    else {
      resolved.P_col = 0
    }

    if (msg.P !== undefined) {
      resolved.P = msg.P;
    }
    else {
      resolved.P = []
    }

    return resolved;
    }
};

module.exports = PreviewResponse;
