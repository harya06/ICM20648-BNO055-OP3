// Auto-generated. Do not edit!

// (in-package op3_online_walking_module_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let PreviewRequest = require('../msg/PreviewRequest.js');

//-----------------------------------------------------------

let PreviewResponse = require('../msg/PreviewResponse.js');

//-----------------------------------------------------------

class GetPreviewMatrixRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.req = null;
    }
    else {
      if (initObj.hasOwnProperty('req')) {
        this.req = initObj.req
      }
      else {
        this.req = new PreviewRequest();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetPreviewMatrixRequest
    // Serialize message field [req]
    bufferOffset = PreviewRequest.serialize(obj.req, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetPreviewMatrixRequest
    let len;
    let data = new GetPreviewMatrixRequest(null);
    // Deserialize message field [req]
    data.req = PreviewRequest.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a service object
    return 'op3_online_walking_module_msgs/GetPreviewMatrixRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c53f64dcb60fc3295f4cc978d9f67b37';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    op3_online_walking_module_msgs/PreviewRequest req
    
    ================================================================================
    MSG: op3_online_walking_module_msgs/PreviewRequest
    float64 control_cycle
    float64 lipm_height
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetPreviewMatrixRequest(null);
    if (msg.req !== undefined) {
      resolved.req = PreviewRequest.Resolve(msg.req)
    }
    else {
      resolved.req = new PreviewRequest()
    }

    return resolved;
    }
};

class GetPreviewMatrixResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.res = null;
    }
    else {
      if (initObj.hasOwnProperty('res')) {
        this.res = initObj.res
      }
      else {
        this.res = new PreviewResponse();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetPreviewMatrixResponse
    // Serialize message field [res]
    bufferOffset = PreviewResponse.serialize(obj.res, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetPreviewMatrixResponse
    let len;
    let data = new GetPreviewMatrixResponse(null);
    // Deserialize message field [res]
    data.res = PreviewResponse.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += PreviewResponse.getMessageSize(object.res);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'op3_online_walking_module_msgs/GetPreviewMatrixResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7d75f439dc091946b1653b9d97860045';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    op3_online_walking_module_msgs/PreviewResponse res
    
    
    ================================================================================
    MSG: op3_online_walking_module_msgs/PreviewResponse
    int32 K_row
    int32 K_col
    float64[] K
    int32 P_row
    int32 P_col
    float64[] P
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetPreviewMatrixResponse(null);
    if (msg.res !== undefined) {
      resolved.res = PreviewResponse.Resolve(msg.res)
    }
    else {
      resolved.res = new PreviewResponse()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetPreviewMatrixRequest,
  Response: GetPreviewMatrixResponse,
  md5sum() { return '353f820108e54e41d7780433bc38ff53'; },
  datatype() { return 'op3_online_walking_module_msgs/GetPreviewMatrix'; }
};
