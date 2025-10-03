// Auto-generated. Do not edit!

// (in-package op3_camera_setting_tool.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class CameraParams {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.brightness = null;
      this.contrast = null;
      this.saturation = null;
      this.sharpness = null;
      this.gain = null;
      this.focus_auto = null;
      this.focus_absolute = null;
      this.exposure_auto = null;
      this.exposure_absolute = null;
      this.white_balance_temperature_auto = null;
      this.white_balance_temperature = null;
    }
    else {
      if (initObj.hasOwnProperty('brightness')) {
        this.brightness = initObj.brightness
      }
      else {
        this.brightness = 0;
      }
      if (initObj.hasOwnProperty('contrast')) {
        this.contrast = initObj.contrast
      }
      else {
        this.contrast = 0;
      }
      if (initObj.hasOwnProperty('saturation')) {
        this.saturation = initObj.saturation
      }
      else {
        this.saturation = 0;
      }
      if (initObj.hasOwnProperty('sharpness')) {
        this.sharpness = initObj.sharpness
      }
      else {
        this.sharpness = 0;
      }
      if (initObj.hasOwnProperty('gain')) {
        this.gain = initObj.gain
      }
      else {
        this.gain = 0;
      }
      if (initObj.hasOwnProperty('focus_auto')) {
        this.focus_auto = initObj.focus_auto
      }
      else {
        this.focus_auto = false;
      }
      if (initObj.hasOwnProperty('focus_absolute')) {
        this.focus_absolute = initObj.focus_absolute
      }
      else {
        this.focus_absolute = 0;
      }
      if (initObj.hasOwnProperty('exposure_auto')) {
        this.exposure_auto = initObj.exposure_auto
      }
      else {
        this.exposure_auto = 0;
      }
      if (initObj.hasOwnProperty('exposure_absolute')) {
        this.exposure_absolute = initObj.exposure_absolute
      }
      else {
        this.exposure_absolute = 0;
      }
      if (initObj.hasOwnProperty('white_balance_temperature_auto')) {
        this.white_balance_temperature_auto = initObj.white_balance_temperature_auto
      }
      else {
        this.white_balance_temperature_auto = false;
      }
      if (initObj.hasOwnProperty('white_balance_temperature')) {
        this.white_balance_temperature = initObj.white_balance_temperature
      }
      else {
        this.white_balance_temperature = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CameraParams
    // Serialize message field [brightness]
    bufferOffset = _serializer.uint32(obj.brightness, buffer, bufferOffset);
    // Serialize message field [contrast]
    bufferOffset = _serializer.uint32(obj.contrast, buffer, bufferOffset);
    // Serialize message field [saturation]
    bufferOffset = _serializer.uint32(obj.saturation, buffer, bufferOffset);
    // Serialize message field [sharpness]
    bufferOffset = _serializer.uint32(obj.sharpness, buffer, bufferOffset);
    // Serialize message field [gain]
    bufferOffset = _serializer.uint32(obj.gain, buffer, bufferOffset);
    // Serialize message field [focus_auto]
    bufferOffset = _serializer.bool(obj.focus_auto, buffer, bufferOffset);
    // Serialize message field [focus_absolute]
    bufferOffset = _serializer.int32(obj.focus_absolute, buffer, bufferOffset);
    // Serialize message field [exposure_auto]
    bufferOffset = _serializer.uint32(obj.exposure_auto, buffer, bufferOffset);
    // Serialize message field [exposure_absolute]
    bufferOffset = _serializer.uint32(obj.exposure_absolute, buffer, bufferOffset);
    // Serialize message field [white_balance_temperature_auto]
    bufferOffset = _serializer.bool(obj.white_balance_temperature_auto, buffer, bufferOffset);
    // Serialize message field [white_balance_temperature]
    bufferOffset = _serializer.uint32(obj.white_balance_temperature, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CameraParams
    let len;
    let data = new CameraParams(null);
    // Deserialize message field [brightness]
    data.brightness = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [contrast]
    data.contrast = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [saturation]
    data.saturation = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [sharpness]
    data.sharpness = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [gain]
    data.gain = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [focus_auto]
    data.focus_auto = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [focus_absolute]
    data.focus_absolute = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [exposure_auto]
    data.exposure_auto = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [exposure_absolute]
    data.exposure_absolute = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [white_balance_temperature_auto]
    data.white_balance_temperature_auto = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [white_balance_temperature]
    data.white_balance_temperature = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 38;
  }

  static datatype() {
    // Returns string type for a message object
    return 'op3_camera_setting_tool/CameraParams';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e3e69e8ca87d52d03119a1d00655d838';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # This represents the parameters of usb camera
    
    uint32  brightness 							# 0 - 255
    uint32  contrast                            # 0 - 255
    uint32  saturation                          # 0 - 255
    uint32  sharpness                           # 0 - 255
    uint32  gain                                # 0 - 255
    bool    focus_auto                          #
    int32  focus_absolute                       # -1 - 255
    uint32  exposure_auto                       #
    uint32  exposure_absolute                   # 0 - 255
    bool    white_balance_temperature_auto      #
    uint32  white_balance_temperature           # 4000 - 6500
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CameraParams(null);
    if (msg.brightness !== undefined) {
      resolved.brightness = msg.brightness;
    }
    else {
      resolved.brightness = 0
    }

    if (msg.contrast !== undefined) {
      resolved.contrast = msg.contrast;
    }
    else {
      resolved.contrast = 0
    }

    if (msg.saturation !== undefined) {
      resolved.saturation = msg.saturation;
    }
    else {
      resolved.saturation = 0
    }

    if (msg.sharpness !== undefined) {
      resolved.sharpness = msg.sharpness;
    }
    else {
      resolved.sharpness = 0
    }

    if (msg.gain !== undefined) {
      resolved.gain = msg.gain;
    }
    else {
      resolved.gain = 0
    }

    if (msg.focus_auto !== undefined) {
      resolved.focus_auto = msg.focus_auto;
    }
    else {
      resolved.focus_auto = false
    }

    if (msg.focus_absolute !== undefined) {
      resolved.focus_absolute = msg.focus_absolute;
    }
    else {
      resolved.focus_absolute = 0
    }

    if (msg.exposure_auto !== undefined) {
      resolved.exposure_auto = msg.exposure_auto;
    }
    else {
      resolved.exposure_auto = 0
    }

    if (msg.exposure_absolute !== undefined) {
      resolved.exposure_absolute = msg.exposure_absolute;
    }
    else {
      resolved.exposure_absolute = 0
    }

    if (msg.white_balance_temperature_auto !== undefined) {
      resolved.white_balance_temperature_auto = msg.white_balance_temperature_auto;
    }
    else {
      resolved.white_balance_temperature_auto = false
    }

    if (msg.white_balance_temperature !== undefined) {
      resolved.white_balance_temperature = msg.white_balance_temperature;
    }
    else {
      resolved.white_balance_temperature = 0
    }

    return resolved;
    }
};

module.exports = CameraParams;
