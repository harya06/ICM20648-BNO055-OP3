// Auto-generated. Do not edit!

// (in-package utils.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class GetVisionDevRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetVisionDevRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetVisionDevRequest
    let len;
    let data = new GetVisionDevRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'utils/GetVisionDevRequest';
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
    const resolved = new GetVisionDevRequest(null);
    return resolved;
    }
};

class GetVisionDevResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.dev = null;
    }
    else {
      if (initObj.hasOwnProperty('dev')) {
        this.dev = initObj.dev
      }
      else {
        this.dev = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetVisionDevResponse
    // Serialize message field [dev]
    bufferOffset = _serializer.int64(obj.dev, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetVisionDevResponse
    let len;
    let data = new GetVisionDevResponse(null);
    // Deserialize message field [dev]
    data.dev = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'utils/GetVisionDevResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '28d20ec34b723ab37e9c2029519bbce4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 dev
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetVisionDevResponse(null);
    if (msg.dev !== undefined) {
      resolved.dev = msg.dev;
    }
    else {
      resolved.dev = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: GetVisionDevRequest,
  Response: GetVisionDevResponse,
  md5sum() { return '28d20ec34b723ab37e9c2029519bbce4'; },
  datatype() { return 'utils/GetVisionDev'; }
};
