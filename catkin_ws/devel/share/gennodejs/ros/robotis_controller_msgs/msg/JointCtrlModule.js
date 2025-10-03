// Auto-generated. Do not edit!

// (in-package robotis_controller_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class JointCtrlModule {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.joint_name = null;
      this.module_name = null;
    }
    else {
      if (initObj.hasOwnProperty('joint_name')) {
        this.joint_name = initObj.joint_name
      }
      else {
        this.joint_name = [];
      }
      if (initObj.hasOwnProperty('module_name')) {
        this.module_name = initObj.module_name
      }
      else {
        this.module_name = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type JointCtrlModule
    // Serialize message field [joint_name]
    bufferOffset = _arraySerializer.string(obj.joint_name, buffer, bufferOffset, null);
    // Serialize message field [module_name]
    bufferOffset = _arraySerializer.string(obj.module_name, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type JointCtrlModule
    let len;
    let data = new JointCtrlModule(null);
    // Deserialize message field [joint_name]
    data.joint_name = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [module_name]
    data.module_name = _arrayDeserializer.string(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.joint_name.forEach((val) => {
      length += 4 + val.length;
    });
    object.module_name.forEach((val) => {
      length += 4 + val.length;
    });
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'robotis_controller_msgs/JointCtrlModule';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1f9dc32600ec95afe667839e777ecbdd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string[] joint_name
    string[] module_name
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new JointCtrlModule(null);
    if (msg.joint_name !== undefined) {
      resolved.joint_name = msg.joint_name;
    }
    else {
      resolved.joint_name = []
    }

    if (msg.module_name !== undefined) {
      resolved.module_name = msg.module_name;
    }
    else {
      resolved.module_name = []
    }

    return resolved;
    }
};

module.exports = JointCtrlModule;
