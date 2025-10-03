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

class Step2D {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.step2d = null;
      this.moving_foot = null;
    }
    else {
      if (initObj.hasOwnProperty('step2d')) {
        this.step2d = initObj.step2d
      }
      else {
        this.step2d = new geometry_msgs.msg.Pose2D();
      }
      if (initObj.hasOwnProperty('moving_foot')) {
        this.moving_foot = initObj.moving_foot
      }
      else {
        this.moving_foot = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Step2D
    // Serialize message field [step2d]
    bufferOffset = geometry_msgs.msg.Pose2D.serialize(obj.step2d, buffer, bufferOffset);
    // Serialize message field [moving_foot]
    bufferOffset = _serializer.uint8(obj.moving_foot, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Step2D
    let len;
    let data = new Step2D(null);
    // Deserialize message field [step2d]
    data.step2d = geometry_msgs.msg.Pose2D.deserialize(buffer, bufferOffset);
    // Deserialize message field [moving_foot]
    data.moving_foot = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 25;
  }

  static datatype() {
    // Returns string type for a message object
    return 'op3_online_walking_module_msgs/Step2D';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8b716dffcd181458918724c59549dd00';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new Step2D(null);
    if (msg.step2d !== undefined) {
      resolved.step2d = geometry_msgs.msg.Pose2D.Resolve(msg.step2d)
    }
    else {
      resolved.step2d = new geometry_msgs.msg.Pose2D()
    }

    if (msg.moving_foot !== undefined) {
      resolved.moving_foot = msg.moving_foot;
    }
    else {
      resolved.moving_foot = 0
    }

    return resolved;
    }
};

// Constants for message
Step2D.Constants = {
  LEFT_FOOT_SWING: 1,
  RIGHT_FOOT_SWING: 2,
  STANDING: 3,
}

module.exports = Step2D;
