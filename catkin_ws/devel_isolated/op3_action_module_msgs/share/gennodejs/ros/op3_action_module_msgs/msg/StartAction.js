// Auto-generated. Do not edit!

// (in-package op3_action_module_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class StartAction {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.page_num = null;
      this.joint_name_array = null;
    }
    else {
      if (initObj.hasOwnProperty('page_num')) {
        this.page_num = initObj.page_num
      }
      else {
        this.page_num = 0;
      }
      if (initObj.hasOwnProperty('joint_name_array')) {
        this.joint_name_array = initObj.joint_name_array
      }
      else {
        this.joint_name_array = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type StartAction
    // Serialize message field [page_num]
    bufferOffset = _serializer.int32(obj.page_num, buffer, bufferOffset);
    // Serialize message field [joint_name_array]
    bufferOffset = _arraySerializer.string(obj.joint_name_array, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type StartAction
    let len;
    let data = new StartAction(null);
    // Deserialize message field [page_num]
    data.page_num = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [joint_name_array]
    data.joint_name_array = _arrayDeserializer.string(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.joint_name_array.forEach((val) => {
      length += 4 + val.length;
    });
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'op3_action_module_msgs/StartAction';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '089f02f04489a5eddf9886b2ae161539';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32     page_num
    string[]  joint_name_array
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new StartAction(null);
    if (msg.page_num !== undefined) {
      resolved.page_num = msg.page_num;
    }
    else {
      resolved.page_num = 0
    }

    if (msg.joint_name_array !== undefined) {
      resolved.joint_name_array = msg.joint_name_array;
    }
    else {
      resolved.joint_name_array = []
    }

    return resolved;
    }
};

module.exports = StartAction;
