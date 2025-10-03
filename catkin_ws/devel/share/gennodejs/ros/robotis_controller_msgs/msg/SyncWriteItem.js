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

class SyncWriteItem {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.item_name = null;
      this.joint_name = null;
      this.value = null;
    }
    else {
      if (initObj.hasOwnProperty('item_name')) {
        this.item_name = initObj.item_name
      }
      else {
        this.item_name = '';
      }
      if (initObj.hasOwnProperty('joint_name')) {
        this.joint_name = initObj.joint_name
      }
      else {
        this.joint_name = [];
      }
      if (initObj.hasOwnProperty('value')) {
        this.value = initObj.value
      }
      else {
        this.value = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SyncWriteItem
    // Serialize message field [item_name]
    bufferOffset = _serializer.string(obj.item_name, buffer, bufferOffset);
    // Serialize message field [joint_name]
    bufferOffset = _arraySerializer.string(obj.joint_name, buffer, bufferOffset, null);
    // Serialize message field [value]
    bufferOffset = _arraySerializer.uint32(obj.value, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SyncWriteItem
    let len;
    let data = new SyncWriteItem(null);
    // Deserialize message field [item_name]
    data.item_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [joint_name]
    data.joint_name = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [value]
    data.value = _arrayDeserializer.uint32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.item_name.length;
    object.joint_name.forEach((val) => {
      length += 4 + val.length;
    });
    length += 4 * object.value.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'robotis_controller_msgs/SyncWriteItem';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f558e04f04dbcc25ce64aa1f45f3dbdd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string item_name
    string[] joint_name
    uint32[] value
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SyncWriteItem(null);
    if (msg.item_name !== undefined) {
      resolved.item_name = msg.item_name;
    }
    else {
      resolved.item_name = ''
    }

    if (msg.joint_name !== undefined) {
      resolved.joint_name = msg.joint_name;
    }
    else {
      resolved.joint_name = []
    }

    if (msg.value !== undefined) {
      resolved.value = msg.value;
    }
    else {
      resolved.value = []
    }

    return resolved;
    }
};

module.exports = SyncWriteItem;
