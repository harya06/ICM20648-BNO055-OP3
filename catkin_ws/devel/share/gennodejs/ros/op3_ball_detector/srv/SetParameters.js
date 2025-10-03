// Auto-generated. Do not edit!

// (in-package op3_ball_detector.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let BallDetectorParams = require('../msg/BallDetectorParams.js');

//-----------------------------------------------------------


//-----------------------------------------------------------

class SetParametersRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.params = null;
    }
    else {
      if (initObj.hasOwnProperty('params')) {
        this.params = initObj.params
      }
      else {
        this.params = new BallDetectorParams();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetParametersRequest
    // Serialize message field [params]
    bufferOffset = BallDetectorParams.serialize(obj.params, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetParametersRequest
    let len;
    let data = new SetParametersRequest(null);
    // Deserialize message field [params]
    data.params = BallDetectorParams.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 60;
  }

  static datatype() {
    // Returns string type for a service object
    return 'op3_ball_detector/SetParametersRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8817963a0699e805c54ef67e580cc1e6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    BallDetectorParams  params
    
    ================================================================================
    MSG: op3_ball_detector/BallDetectorParams
    # This represents the parameters of ball_detector
    
    uint32  gaussian_blur_size      # only odd number, 1 - 11
    float32 gaussian_blur_sigma     # 1 - 5
    float32 canny_edge_th           # 50 - 200
    float32 hough_accum_resolution  # 1 - 8
    float32 hough_accum_th          # 10 - 200
    float32 min_circle_dist         # 10 - 200
    uint32  min_radius              # 10 - 200
    uint32  max_radius              # 100 - 600
    uint32  filter_h_min            # 0 - 359
    uint32  filter_h_max
    uint32  filter_s_min            # 0 - 255
    uint32  filter_s_max
    uint32  filter_v_min            # 0 - 255
    uint32  filter_v_max
    uint32  ellipse_size            # 1 - 9
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetParametersRequest(null);
    if (msg.params !== undefined) {
      resolved.params = BallDetectorParams.Resolve(msg.params)
    }
    else {
      resolved.params = new BallDetectorParams()
    }

    return resolved;
    }
};

class SetParametersResponse {
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
        this.returns = new BallDetectorParams();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetParametersResponse
    // Serialize message field [returns]
    bufferOffset = BallDetectorParams.serialize(obj.returns, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetParametersResponse
    let len;
    let data = new SetParametersResponse(null);
    // Deserialize message field [returns]
    data.returns = BallDetectorParams.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 60;
  }

  static datatype() {
    // Returns string type for a service object
    return 'op3_ball_detector/SetParametersResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '17229f06a68a0d87fd525e4d8bdaee90';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    BallDetectorParams  returns
    
    
    ================================================================================
    MSG: op3_ball_detector/BallDetectorParams
    # This represents the parameters of ball_detector
    
    uint32  gaussian_blur_size      # only odd number, 1 - 11
    float32 gaussian_blur_sigma     # 1 - 5
    float32 canny_edge_th           # 50 - 200
    float32 hough_accum_resolution  # 1 - 8
    float32 hough_accum_th          # 10 - 200
    float32 min_circle_dist         # 10 - 200
    uint32  min_radius              # 10 - 200
    uint32  max_radius              # 100 - 600
    uint32  filter_h_min            # 0 - 359
    uint32  filter_h_max
    uint32  filter_s_min            # 0 - 255
    uint32  filter_s_max
    uint32  filter_v_min            # 0 - 255
    uint32  filter_v_max
    uint32  ellipse_size            # 1 - 9
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetParametersResponse(null);
    if (msg.returns !== undefined) {
      resolved.returns = BallDetectorParams.Resolve(msg.returns)
    }
    else {
      resolved.returns = new BallDetectorParams()
    }

    return resolved;
    }
};

module.exports = {
  Request: SetParametersRequest,
  Response: SetParametersResponse,
  md5sum() { return 'b144be2bfcda348236c05dcc43378583'; },
  datatype() { return 'op3_ball_detector/SetParameters'; }
};
