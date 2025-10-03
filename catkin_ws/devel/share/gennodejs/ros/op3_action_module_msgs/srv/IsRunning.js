// Auto-generated. Do not edit!

// (in-package op3_action_module_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class IsRunningRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type IsRunningRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type IsRunningRequest
    let len;
    let data = new IsRunningRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'op3_action_module_msgs/IsRunningRequest';
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
    const resolved = new IsRunningRequest(null);
    return resolved;
    }
};

class IsRunningResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.is_running = null;
    }
    else {
      if (initObj.hasOwnProperty('is_running')) {
        this.is_running = initObj.is_running
      }
      else {
        this.is_running = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type IsRunningResponse
    // Serialize message field [is_running]
    bufferOffset = _serializer.bool(obj.is_running, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type IsRunningResponse
    let len;
    let data = new IsRunningResponse(null);
    // Deserialize message field [is_running]
    data.is_running = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'op3_action_module_msgs/IsRunningResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ae3468a1af93d845e943210e7cef5a54';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool is_running
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new IsRunningResponse(null);
    if (msg.is_running !== undefined) {
      resolved.is_running = msg.is_running;
    }
    else {
      resolved.is_running = false
    }

    return resolved;
    }
};

module.exports = {
  Request: IsRunningRequest,
  Response: IsRunningResponse,
  md5sum() { return 'ae3468a1af93d845e943210e7cef5a54'; },
  datatype() { return 'op3_action_module_msgs/IsRunning'; }
};
