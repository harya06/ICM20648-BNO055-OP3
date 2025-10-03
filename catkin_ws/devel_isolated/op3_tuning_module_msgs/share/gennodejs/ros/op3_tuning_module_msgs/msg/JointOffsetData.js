// Auto-generated. Do not edit!

// (in-package op3_tuning_module_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class JointOffsetData {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.joint_name = null;
      this.goal_value = null;
      this.offset_value = null;
      this.p_gain = null;
      this.i_gain = null;
      this.d_gain = null;
    }
    else {
      if (initObj.hasOwnProperty('joint_name')) {
        this.joint_name = initObj.joint_name
      }
      else {
        this.joint_name = '';
      }
      if (initObj.hasOwnProperty('goal_value')) {
        this.goal_value = initObj.goal_value
      }
      else {
        this.goal_value = 0.0;
      }
      if (initObj.hasOwnProperty('offset_value')) {
        this.offset_value = initObj.offset_value
      }
      else {
        this.offset_value = 0.0;
      }
      if (initObj.hasOwnProperty('p_gain')) {
        this.p_gain = initObj.p_gain
      }
      else {
        this.p_gain = 0;
      }
      if (initObj.hasOwnProperty('i_gain')) {
        this.i_gain = initObj.i_gain
      }
      else {
        this.i_gain = 0;
      }
      if (initObj.hasOwnProperty('d_gain')) {
        this.d_gain = initObj.d_gain
      }
      else {
        this.d_gain = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type JointOffsetData
    // Serialize message field [joint_name]
    bufferOffset = _serializer.string(obj.joint_name, buffer, bufferOffset);
    // Serialize message field [goal_value]
    bufferOffset = _serializer.float64(obj.goal_value, buffer, bufferOffset);
    // Serialize message field [offset_value]
    bufferOffset = _serializer.float64(obj.offset_value, buffer, bufferOffset);
    // Serialize message field [p_gain]
    bufferOffset = _serializer.int32(obj.p_gain, buffer, bufferOffset);
    // Serialize message field [i_gain]
    bufferOffset = _serializer.int32(obj.i_gain, buffer, bufferOffset);
    // Serialize message field [d_gain]
    bufferOffset = _serializer.int32(obj.d_gain, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type JointOffsetData
    let len;
    let data = new JointOffsetData(null);
    // Deserialize message field [joint_name]
    data.joint_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [goal_value]
    data.goal_value = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [offset_value]
    data.offset_value = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [p_gain]
    data.p_gain = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [i_gain]
    data.i_gain = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [d_gain]
    data.d_gain = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.joint_name.length;
    return length + 32;
  }

  static datatype() {
    // Returns string type for a message object
    return 'op3_tuning_module_msgs/JointOffsetData';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f2ced0a4562683b45a9aab2f0e3e1f84';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string  joint_name
    float64 goal_value
    float64 offset_value
    int32   p_gain
    int32   i_gain
    int32   d_gain
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new JointOffsetData(null);
    if (msg.joint_name !== undefined) {
      resolved.joint_name = msg.joint_name;
    }
    else {
      resolved.joint_name = ''
    }

    if (msg.goal_value !== undefined) {
      resolved.goal_value = msg.goal_value;
    }
    else {
      resolved.goal_value = 0.0
    }

    if (msg.offset_value !== undefined) {
      resolved.offset_value = msg.offset_value;
    }
    else {
      resolved.offset_value = 0.0
    }

    if (msg.p_gain !== undefined) {
      resolved.p_gain = msg.p_gain;
    }
    else {
      resolved.p_gain = 0
    }

    if (msg.i_gain !== undefined) {
      resolved.i_gain = msg.i_gain;
    }
    else {
      resolved.i_gain = 0
    }

    if (msg.d_gain !== undefined) {
      resolved.d_gain = msg.d_gain;
    }
    else {
      resolved.d_gain = 0
    }

    return resolved;
    }
};

module.exports = JointOffsetData;
