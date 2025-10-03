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

class WriteControlTable {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.joint_name = null;
      this.start_item_name = null;
      this.data_length = null;
      this.data = null;
    }
    else {
      if (initObj.hasOwnProperty('joint_name')) {
        this.joint_name = initObj.joint_name
      }
      else {
        this.joint_name = '';
      }
      if (initObj.hasOwnProperty('start_item_name')) {
        this.start_item_name = initObj.start_item_name
      }
      else {
        this.start_item_name = '';
      }
      if (initObj.hasOwnProperty('data_length')) {
        this.data_length = initObj.data_length
      }
      else {
        this.data_length = 0;
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
    // Serializes a message object of type WriteControlTable
    // Serialize message field [joint_name]
    bufferOffset = _serializer.string(obj.joint_name, buffer, bufferOffset);
    // Serialize message field [start_item_name]
    bufferOffset = _serializer.string(obj.start_item_name, buffer, bufferOffset);
    // Serialize message field [data_length]
    bufferOffset = _serializer.uint16(obj.data_length, buffer, bufferOffset);
    // Serialize message field [data]
    bufferOffset = _arraySerializer.uint8(obj.data, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type WriteControlTable
    let len;
    let data = new WriteControlTable(null);
    // Deserialize message field [joint_name]
    data.joint_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [start_item_name]
    data.start_item_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [data_length]
    data.data_length = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [data]
    data.data = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.joint_name.length;
    length += object.start_item_name.length;
    length += object.data.length;
    return length + 14;
  }

  static datatype() {
    // Returns string type for a message object
    return 'robotis_controller_msgs/WriteControlTable';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0d4e0e3e28ede43d877e2fb8f0a1a011';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string joint_name
    string start_item_name
    uint16 data_length
    uint8[] data
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new WriteControlTable(null);
    if (msg.joint_name !== undefined) {
      resolved.joint_name = msg.joint_name;
    }
    else {
      resolved.joint_name = ''
    }

    if (msg.start_item_name !== undefined) {
      resolved.start_item_name = msg.start_item_name;
    }
    else {
      resolved.start_item_name = ''
    }

    if (msg.data_length !== undefined) {
      resolved.data_length = msg.data_length;
    }
    else {
      resolved.data_length = 0
    }

    if (msg.data !== undefined) {
      resolved.data = msg.data;
    }
    else {
      resolved.data = []
    }

    return resolved;
    }
};

module.exports = WriteControlTable;
