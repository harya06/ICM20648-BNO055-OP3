// Auto-generated. Do not edit!

// (in-package op3_online_walking_module_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class FootStepArray {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.moving_foot = null;
      this.data = null;
    }
    else {
      if (initObj.hasOwnProperty('moving_foot')) {
        this.moving_foot = initObj.moving_foot
      }
      else {
        this.moving_foot = [];
      }
      if (initObj.hasOwnProperty('data')) {
        this.data = initObj.data
      }
      else {
        this.data = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FootStepArray
    // Serialize message field [moving_foot]
    bufferOffset = _arraySerializer.int32(obj.moving_foot, buffer, bufferOffset, null);
    // Serialize message field [data]
    // Serialize the length for message field [data]
    bufferOffset = _serializer.uint32(obj.data.length, buffer, bufferOffset);
    obj.data.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Pose2D.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FootStepArray
    let len;
    let data = new FootStepArray(null);
    // Deserialize message field [moving_foot]
    data.moving_foot = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [data]
    // Deserialize array length for message field [data]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.data = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.data[i] = geometry_msgs.msg.Pose2D.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.moving_foot.length;
    length += 24 * object.data.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'op3_online_walking_module_msgs/FootStepArray';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd024e75ed0a26f72c71cd63c2720fb63';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    int32 LEFT_FOOT  = 0 # Left foot constant
    int32 RIGHT_FOOT = 1 # Right foot constant
    
    int32[]                moving_foot
    geometry_msgs/Pose2D[] data
    
    ================================================================================
    MSG: geometry_msgs/Pose2D
    # This expresses a position and orientation on a 2D manifold.
    
    float64 x
    float64 y
    float64 theta
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FootStepArray(null);
    if (msg.moving_foot !== undefined) {
      resolved.moving_foot = msg.moving_foot;
    }
    else {
      resolved.moving_foot = []
    }

    if (msg.data !== undefined) {
      resolved.data = new Array(msg.data.length);
      for (let i = 0; i < resolved.data.length; ++i) {
        resolved.data[i] = geometry_msgs.msg.Pose2D.Resolve(msg.data[i]);
      }
    }
    else {
      resolved.data = []
    }

    return resolved;
    }
};

// Constants for message
FootStepArray.Constants = {
  LEFT_FOOT: 0,
  RIGHT_FOOT: 1,
}

module.exports = FootStepArray;
