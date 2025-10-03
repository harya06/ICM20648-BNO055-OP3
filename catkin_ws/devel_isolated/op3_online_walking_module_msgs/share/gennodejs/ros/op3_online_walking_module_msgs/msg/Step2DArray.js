// Auto-generated. Do not edit!

// (in-package op3_online_walking_module_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Step2D = require('./Step2D.js');

//-----------------------------------------------------------

class Step2DArray {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.step_time = null;
      this.footsteps_2d = null;
    }
    else {
      if (initObj.hasOwnProperty('step_time')) {
        this.step_time = initObj.step_time
      }
      else {
        this.step_time = 0.0;
      }
      if (initObj.hasOwnProperty('footsteps_2d')) {
        this.footsteps_2d = initObj.footsteps_2d
      }
      else {
        this.footsteps_2d = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Step2DArray
    // Serialize message field [step_time]
    bufferOffset = _serializer.float64(obj.step_time, buffer, bufferOffset);
    // Serialize message field [footsteps_2d]
    // Serialize the length for message field [footsteps_2d]
    bufferOffset = _serializer.uint32(obj.footsteps_2d.length, buffer, bufferOffset);
    obj.footsteps_2d.forEach((val) => {
      bufferOffset = Step2D.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Step2DArray
    let len;
    let data = new Step2DArray(null);
    // Deserialize message field [step_time]
    data.step_time = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [footsteps_2d]
    // Deserialize array length for message field [footsteps_2d]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.footsteps_2d = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.footsteps_2d[i] = Step2D.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 25 * object.footsteps_2d.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'op3_online_walking_module_msgs/Step2DArray';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '598423594f1c61377b299ae8d55d0f04';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 step_time
    Step2D[] footsteps_2d
    
    ================================================================================
    MSG: op3_online_walking_module_msgs/Step2D
    #2D StepData
    
    geometry_msgs/Pose2D step2d   # step pose as relative offset to last leg
    
    
    # which leg to be used (left/right/no, see below)
    uint8 moving_foot   
    
    uint8 LEFT_FOOT_SWING  = 1 # Left foot constant
    uint8 RIGHT_FOOT_SWING = 2 # Right foot constant
    uint8 STANDING         = 3 # Standing constant
    
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
    const resolved = new Step2DArray(null);
    if (msg.step_time !== undefined) {
      resolved.step_time = msg.step_time;
    }
    else {
      resolved.step_time = 0.0
    }

    if (msg.footsteps_2d !== undefined) {
      resolved.footsteps_2d = new Array(msg.footsteps_2d.length);
      for (let i = 0; i < resolved.footsteps_2d.length; ++i) {
        resolved.footsteps_2d[i] = Step2D.Resolve(msg.footsteps_2d[i]);
      }
    }
    else {
      resolved.footsteps_2d = []
    }

    return resolved;
    }
};

module.exports = Step2DArray;
