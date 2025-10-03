// Auto-generated. Do not edit!

// (in-package op3_tuning_module_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let JointTorqueOnOff = require('./JointTorqueOnOff.js');

//-----------------------------------------------------------

class JointTorqueOnOffArray {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.torque_enable_data = null;
    }
    else {
      if (initObj.hasOwnProperty('torque_enable_data')) {
        this.torque_enable_data = initObj.torque_enable_data
      }
      else {
        this.torque_enable_data = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type JointTorqueOnOffArray
    // Serialize message field [torque_enable_data]
    // Serialize the length for message field [torque_enable_data]
    bufferOffset = _serializer.uint32(obj.torque_enable_data.length, buffer, bufferOffset);
    obj.torque_enable_data.forEach((val) => {
      bufferOffset = JointTorqueOnOff.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type JointTorqueOnOffArray
    let len;
    let data = new JointTorqueOnOffArray(null);
    // Deserialize message field [torque_enable_data]
    // Deserialize array length for message field [torque_enable_data]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.torque_enable_data = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.torque_enable_data[i] = JointTorqueOnOff.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.torque_enable_data.forEach((val) => {
      length += JointTorqueOnOff.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'op3_tuning_module_msgs/JointTorqueOnOffArray';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1ca4db772b4d802ac00aebf4469fc8bf';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    JointTorqueOnOff[] torque_enable_data
    ================================================================================
    MSG: op3_tuning_module_msgs/JointTorqueOnOff
    string  joint_name
    bool    torque_enable
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new JointTorqueOnOffArray(null);
    if (msg.torque_enable_data !== undefined) {
      resolved.torque_enable_data = new Array(msg.torque_enable_data.length);
      for (let i = 0; i < resolved.torque_enable_data.length; ++i) {
        resolved.torque_enable_data[i] = JointTorqueOnOff.Resolve(msg.torque_enable_data[i]);
      }
    }
    else {
      resolved.torque_enable_data = []
    }

    return resolved;
    }
};

module.exports = JointTorqueOnOffArray;
