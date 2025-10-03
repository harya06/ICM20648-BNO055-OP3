// Auto-generated. Do not edit!

// (in-package op3_online_walking_module_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

let KinematicsPose = require('../msg/KinematicsPose.js');

//-----------------------------------------------------------

class GetKinematicsPoseRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.name = null;
    }
    else {
      if (initObj.hasOwnProperty('name')) {
        this.name = initObj.name
      }
      else {
        this.name = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetKinematicsPoseRequest
    // Serialize message field [name]
    bufferOffset = _serializer.string(obj.name, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetKinematicsPoseRequest
    let len;
    let data = new GetKinematicsPoseRequest(null);
    // Deserialize message field [name]
    data.name = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.name.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'op3_online_walking_module_msgs/GetKinematicsPoseRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c1f3d28f1b044c871e6eff2e9fc3c667';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string    name
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetKinematicsPoseRequest(null);
    if (msg.name !== undefined) {
      resolved.name = msg.name;
    }
    else {
      resolved.name = ''
    }

    return resolved;
    }
};

class GetKinematicsPoseResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pose = null;
    }
    else {
      if (initObj.hasOwnProperty('pose')) {
        this.pose = initObj.pose
      }
      else {
        this.pose = new KinematicsPose();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetKinematicsPoseResponse
    // Serialize message field [pose]
    bufferOffset = KinematicsPose.serialize(obj.pose, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetKinematicsPoseResponse
    let len;
    let data = new GetKinematicsPoseResponse(null);
    // Deserialize message field [pose]
    data.pose = KinematicsPose.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += KinematicsPose.getMessageSize(object.pose);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'op3_online_walking_module_msgs/GetKinematicsPoseResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c75b426633d0bb0b22c3a048b9eac9ee';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    op3_online_walking_module_msgs/KinematicsPose pose
    
    
    ================================================================================
    MSG: op3_online_walking_module_msgs/KinematicsPose
    string  name
    float64  mov_time
    geometry_msgs/Pose pose
    
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetKinematicsPoseResponse(null);
    if (msg.pose !== undefined) {
      resolved.pose = KinematicsPose.Resolve(msg.pose)
    }
    else {
      resolved.pose = new KinematicsPose()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetKinematicsPoseRequest,
  Response: GetKinematicsPoseResponse,
  md5sum() { return 'aad71eecbad83fbcb051be59f9fbb914'; },
  datatype() { return 'op3_online_walking_module_msgs/GetKinematicsPose'; }
};
