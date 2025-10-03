// Auto-generated. Do not edit!

// (in-package op3_camera_setting_tool.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

let CameraParams = require('../msg/CameraParams.js');

//-----------------------------------------------------------

class GetParametersRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetParametersRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetParametersRequest
    let len;
    let data = new GetParametersRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'op3_camera_setting_tool/GetParametersRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetParametersRequest(null);
    return resolved;
    }
};

class GetParametersResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.returns = null;
    }
    else {
      if (initObj.hasOwnProperty('returns')) {
        this.returns = initObj.returns
      }
      else {
        this.returns = new CameraParams();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetParametersResponse
    // Serialize message field [returns]
    bufferOffset = CameraParams.serialize(obj.returns, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetParametersResponse
    let len;
    let data = new GetParametersResponse(null);
    // Deserialize message field [returns]
    data.returns = CameraParams.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 38;
  }

  static datatype() {
    // Returns string type for a service object
    return 'op3_camera_setting_tool/GetParametersResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ba9411ea8e0a1eee4b9a16def5c761b3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    CameraParams  returns
    
    
    ================================================================================
    MSG: op3_camera_setting_tool/CameraParams
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
    const resolved = new GetParametersResponse(null);
    if (msg.returns !== undefined) {
      resolved.returns = CameraParams.Resolve(msg.returns)
    }
    else {
      resolved.returns = new CameraParams()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetParametersRequest,
  Response: GetParametersResponse,
  md5sum() { return 'ba9411ea8e0a1eee4b9a16def5c761b3'; },
  datatype() { return 'op3_camera_setting_tool/GetParameters'; }
};
