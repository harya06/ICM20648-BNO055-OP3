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

class WalkingParam {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.dsp_ratio = null;
      this.lipm_height = null;
      this.foot_height_max = null;
      this.zmp_offset_x = null;
      this.zmp_offset_y = null;
    }
    else {
      if (initObj.hasOwnProperty('dsp_ratio')) {
        this.dsp_ratio = initObj.dsp_ratio
      }
      else {
        this.dsp_ratio = 0.0;
      }
      if (initObj.hasOwnProperty('lipm_height')) {
        this.lipm_height = initObj.lipm_height
      }
      else {
        this.lipm_height = 0.0;
      }
      if (initObj.hasOwnProperty('foot_height_max')) {
        this.foot_height_max = initObj.foot_height_max
      }
      else {
        this.foot_height_max = 0.0;
      }
      if (initObj.hasOwnProperty('zmp_offset_x')) {
        this.zmp_offset_x = initObj.zmp_offset_x
      }
      else {
        this.zmp_offset_x = 0.0;
      }
      if (initObj.hasOwnProperty('zmp_offset_y')) {
        this.zmp_offset_y = initObj.zmp_offset_y
      }
      else {
        this.zmp_offset_y = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type WalkingParam
    // Serialize message field [dsp_ratio]
    bufferOffset = _serializer.float64(obj.dsp_ratio, buffer, bufferOffset);
    // Serialize message field [lipm_height]
    bufferOffset = _serializer.float64(obj.lipm_height, buffer, bufferOffset);
    // Serialize message field [foot_height_max]
    bufferOffset = _serializer.float64(obj.foot_height_max, buffer, bufferOffset);
    // Serialize message field [zmp_offset_x]
    bufferOffset = _serializer.float64(obj.zmp_offset_x, buffer, bufferOffset);
    // Serialize message field [zmp_offset_y]
    bufferOffset = _serializer.float64(obj.zmp_offset_y, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type WalkingParam
    let len;
    let data = new WalkingParam(null);
    // Deserialize message field [dsp_ratio]
    data.dsp_ratio = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [lipm_height]
    data.lipm_height = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [foot_height_max]
    data.foot_height_max = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [zmp_offset_x]
    data.zmp_offset_x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [zmp_offset_y]
    data.zmp_offset_y = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 40;
  }

  static datatype() {
    // Returns string type for a message object
    return 'op3_online_walking_module_msgs/WalkingParam';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2197a6ee210540728363ab6b547935bd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 dsp_ratio
    float64 lipm_height
    float64 foot_height_max
    float64 zmp_offset_x
    float64 zmp_offset_y
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new WalkingParam(null);
    if (msg.dsp_ratio !== undefined) {
      resolved.dsp_ratio = msg.dsp_ratio;
    }
    else {
      resolved.dsp_ratio = 0.0
    }

    if (msg.lipm_height !== undefined) {
      resolved.lipm_height = msg.lipm_height;
    }
    else {
      resolved.lipm_height = 0.0
    }

    if (msg.foot_height_max !== undefined) {
      resolved.foot_height_max = msg.foot_height_max;
    }
    else {
      resolved.foot_height_max = 0.0
    }

    if (msg.zmp_offset_x !== undefined) {
      resolved.zmp_offset_x = msg.zmp_offset_x;
    }
    else {
      resolved.zmp_offset_x = 0.0
    }

    if (msg.zmp_offset_y !== undefined) {
      resolved.zmp_offset_y = msg.zmp_offset_y;
    }
    else {
      resolved.zmp_offset_y = 0.0
    }

    return resolved;
    }
};

module.exports = WalkingParam;
