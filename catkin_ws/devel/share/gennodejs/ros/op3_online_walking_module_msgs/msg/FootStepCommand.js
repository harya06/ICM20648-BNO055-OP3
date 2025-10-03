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

class FootStepCommand {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.command = null;
      this.start_leg = null;
      this.step_num = null;
      this.step_time = null;
      this.step_length = null;
      this.side_length = null;
      this.step_angle = null;
    }
    else {
      if (initObj.hasOwnProperty('command')) {
        this.command = initObj.command
      }
      else {
        this.command = '';
      }
      if (initObj.hasOwnProperty('start_leg')) {
        this.start_leg = initObj.start_leg
      }
      else {
        this.start_leg = '';
      }
      if (initObj.hasOwnProperty('step_num')) {
        this.step_num = initObj.step_num
      }
      else {
        this.step_num = 0;
      }
      if (initObj.hasOwnProperty('step_time')) {
        this.step_time = initObj.step_time
      }
      else {
        this.step_time = 0.0;
      }
      if (initObj.hasOwnProperty('step_length')) {
        this.step_length = initObj.step_length
      }
      else {
        this.step_length = 0.0;
      }
      if (initObj.hasOwnProperty('side_length')) {
        this.side_length = initObj.side_length
      }
      else {
        this.side_length = 0.0;
      }
      if (initObj.hasOwnProperty('step_angle')) {
        this.step_angle = initObj.step_angle
      }
      else {
        this.step_angle = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FootStepCommand
    // Serialize message field [command]
    bufferOffset = _serializer.string(obj.command, buffer, bufferOffset);
    // Serialize message field [start_leg]
    bufferOffset = _serializer.string(obj.start_leg, buffer, bufferOffset);
    // Serialize message field [step_num]
    bufferOffset = _serializer.int32(obj.step_num, buffer, bufferOffset);
    // Serialize message field [step_time]
    bufferOffset = _serializer.float64(obj.step_time, buffer, bufferOffset);
    // Serialize message field [step_length]
    bufferOffset = _serializer.float64(obj.step_length, buffer, bufferOffset);
    // Serialize message field [side_length]
    bufferOffset = _serializer.float64(obj.side_length, buffer, bufferOffset);
    // Serialize message field [step_angle]
    bufferOffset = _serializer.float64(obj.step_angle, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FootStepCommand
    let len;
    let data = new FootStepCommand(null);
    // Deserialize message field [command]
    data.command = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [start_leg]
    data.start_leg = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [step_num]
    data.step_num = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [step_time]
    data.step_time = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [step_length]
    data.step_length = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [side_length]
    data.side_length = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [step_angle]
    data.step_angle = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.command.length;
    length += object.start_leg.length;
    return length + 44;
  }

  static datatype() {
    // Returns string type for a message object
    return 'op3_online_walking_module_msgs/FootStepCommand';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6b1e994ec1a40ca11db9d34d679a3f8c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string  command
    string  start_leg
    int32   step_num
    float64 step_time
    float64 step_length
    float64 side_length
    float64 step_angle
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FootStepCommand(null);
    if (msg.command !== undefined) {
      resolved.command = msg.command;
    }
    else {
      resolved.command = ''
    }

    if (msg.start_leg !== undefined) {
      resolved.start_leg = msg.start_leg;
    }
    else {
      resolved.start_leg = ''
    }

    if (msg.step_num !== undefined) {
      resolved.step_num = msg.step_num;
    }
    else {
      resolved.step_num = 0
    }

    if (msg.step_time !== undefined) {
      resolved.step_time = msg.step_time;
    }
    else {
      resolved.step_time = 0.0
    }

    if (msg.step_length !== undefined) {
      resolved.step_length = msg.step_length;
    }
    else {
      resolved.step_length = 0.0
    }

    if (msg.side_length !== undefined) {
      resolved.side_length = msg.side_length;
    }
    else {
      resolved.side_length = 0.0
    }

    if (msg.step_angle !== undefined) {
      resolved.step_angle = msg.step_angle;
    }
    else {
      resolved.step_angle = 0.0
    }

    return resolved;
    }
};

module.exports = FootStepCommand;
