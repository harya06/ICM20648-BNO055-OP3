// Auto-generated. Do not edit!

// (in-package op3_ball_detector.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class BallDetectorParams {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.gaussian_blur_size = null;
      this.gaussian_blur_sigma = null;
      this.canny_edge_th = null;
      this.hough_accum_resolution = null;
      this.hough_accum_th = null;
      this.min_circle_dist = null;
      this.min_radius = null;
      this.max_radius = null;
      this.filter_h_min = null;
      this.filter_h_max = null;
      this.filter_s_min = null;
      this.filter_s_max = null;
      this.filter_v_min = null;
      this.filter_v_max = null;
      this.ellipse_size = null;
    }
    else {
      if (initObj.hasOwnProperty('gaussian_blur_size')) {
        this.gaussian_blur_size = initObj.gaussian_blur_size
      }
      else {
        this.gaussian_blur_size = 0;
      }
      if (initObj.hasOwnProperty('gaussian_blur_sigma')) {
        this.gaussian_blur_sigma = initObj.gaussian_blur_sigma
      }
      else {
        this.gaussian_blur_sigma = 0.0;
      }
      if (initObj.hasOwnProperty('canny_edge_th')) {
        this.canny_edge_th = initObj.canny_edge_th
      }
      else {
        this.canny_edge_th = 0.0;
      }
      if (initObj.hasOwnProperty('hough_accum_resolution')) {
        this.hough_accum_resolution = initObj.hough_accum_resolution
      }
      else {
        this.hough_accum_resolution = 0.0;
      }
      if (initObj.hasOwnProperty('hough_accum_th')) {
        this.hough_accum_th = initObj.hough_accum_th
      }
      else {
        this.hough_accum_th = 0.0;
      }
      if (initObj.hasOwnProperty('min_circle_dist')) {
        this.min_circle_dist = initObj.min_circle_dist
      }
      else {
        this.min_circle_dist = 0.0;
      }
      if (initObj.hasOwnProperty('min_radius')) {
        this.min_radius = initObj.min_radius
      }
      else {
        this.min_radius = 0;
      }
      if (initObj.hasOwnProperty('max_radius')) {
        this.max_radius = initObj.max_radius
      }
      else {
        this.max_radius = 0;
      }
      if (initObj.hasOwnProperty('filter_h_min')) {
        this.filter_h_min = initObj.filter_h_min
      }
      else {
        this.filter_h_min = 0;
      }
      if (initObj.hasOwnProperty('filter_h_max')) {
        this.filter_h_max = initObj.filter_h_max
      }
      else {
        this.filter_h_max = 0;
      }
      if (initObj.hasOwnProperty('filter_s_min')) {
        this.filter_s_min = initObj.filter_s_min
      }
      else {
        this.filter_s_min = 0;
      }
      if (initObj.hasOwnProperty('filter_s_max')) {
        this.filter_s_max = initObj.filter_s_max
      }
      else {
        this.filter_s_max = 0;
      }
      if (initObj.hasOwnProperty('filter_v_min')) {
        this.filter_v_min = initObj.filter_v_min
      }
      else {
        this.filter_v_min = 0;
      }
      if (initObj.hasOwnProperty('filter_v_max')) {
        this.filter_v_max = initObj.filter_v_max
      }
      else {
        this.filter_v_max = 0;
      }
      if (initObj.hasOwnProperty('ellipse_size')) {
        this.ellipse_size = initObj.ellipse_size
      }
      else {
        this.ellipse_size = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type BallDetectorParams
    // Serialize message field [gaussian_blur_size]
    bufferOffset = _serializer.uint32(obj.gaussian_blur_size, buffer, bufferOffset);
    // Serialize message field [gaussian_blur_sigma]
    bufferOffset = _serializer.float32(obj.gaussian_blur_sigma, buffer, bufferOffset);
    // Serialize message field [canny_edge_th]
    bufferOffset = _serializer.float32(obj.canny_edge_th, buffer, bufferOffset);
    // Serialize message field [hough_accum_resolution]
    bufferOffset = _serializer.float32(obj.hough_accum_resolution, buffer, bufferOffset);
    // Serialize message field [hough_accum_th]
    bufferOffset = _serializer.float32(obj.hough_accum_th, buffer, bufferOffset);
    // Serialize message field [min_circle_dist]
    bufferOffset = _serializer.float32(obj.min_circle_dist, buffer, bufferOffset);
    // Serialize message field [min_radius]
    bufferOffset = _serializer.uint32(obj.min_radius, buffer, bufferOffset);
    // Serialize message field [max_radius]
    bufferOffset = _serializer.uint32(obj.max_radius, buffer, bufferOffset);
    // Serialize message field [filter_h_min]
    bufferOffset = _serializer.uint32(obj.filter_h_min, buffer, bufferOffset);
    // Serialize message field [filter_h_max]
    bufferOffset = _serializer.uint32(obj.filter_h_max, buffer, bufferOffset);
    // Serialize message field [filter_s_min]
    bufferOffset = _serializer.uint32(obj.filter_s_min, buffer, bufferOffset);
    // Serialize message field [filter_s_max]
    bufferOffset = _serializer.uint32(obj.filter_s_max, buffer, bufferOffset);
    // Serialize message field [filter_v_min]
    bufferOffset = _serializer.uint32(obj.filter_v_min, buffer, bufferOffset);
    // Serialize message field [filter_v_max]
    bufferOffset = _serializer.uint32(obj.filter_v_max, buffer, bufferOffset);
    // Serialize message field [ellipse_size]
    bufferOffset = _serializer.uint32(obj.ellipse_size, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type BallDetectorParams
    let len;
    let data = new BallDetectorParams(null);
    // Deserialize message field [gaussian_blur_size]
    data.gaussian_blur_size = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [gaussian_blur_sigma]
    data.gaussian_blur_sigma = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [canny_edge_th]
    data.canny_edge_th = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [hough_accum_resolution]
    data.hough_accum_resolution = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [hough_accum_th]
    data.hough_accum_th = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [min_circle_dist]
    data.min_circle_dist = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [min_radius]
    data.min_radius = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [max_radius]
    data.max_radius = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [filter_h_min]
    data.filter_h_min = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [filter_h_max]
    data.filter_h_max = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [filter_s_min]
    data.filter_s_min = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [filter_s_max]
    data.filter_s_max = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [filter_v_min]
    data.filter_v_min = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [filter_v_max]
    data.filter_v_max = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [ellipse_size]
    data.ellipse_size = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 60;
  }

  static datatype() {
    // Returns string type for a message object
    return 'op3_ball_detector/BallDetectorParams';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '23a4c471d4f161f32a80066aac78a96c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new BallDetectorParams(null);
    if (msg.gaussian_blur_size !== undefined) {
      resolved.gaussian_blur_size = msg.gaussian_blur_size;
    }
    else {
      resolved.gaussian_blur_size = 0
    }

    if (msg.gaussian_blur_sigma !== undefined) {
      resolved.gaussian_blur_sigma = msg.gaussian_blur_sigma;
    }
    else {
      resolved.gaussian_blur_sigma = 0.0
    }

    if (msg.canny_edge_th !== undefined) {
      resolved.canny_edge_th = msg.canny_edge_th;
    }
    else {
      resolved.canny_edge_th = 0.0
    }

    if (msg.hough_accum_resolution !== undefined) {
      resolved.hough_accum_resolution = msg.hough_accum_resolution;
    }
    else {
      resolved.hough_accum_resolution = 0.0
    }

    if (msg.hough_accum_th !== undefined) {
      resolved.hough_accum_th = msg.hough_accum_th;
    }
    else {
      resolved.hough_accum_th = 0.0
    }

    if (msg.min_circle_dist !== undefined) {
      resolved.min_circle_dist = msg.min_circle_dist;
    }
    else {
      resolved.min_circle_dist = 0.0
    }

    if (msg.min_radius !== undefined) {
      resolved.min_radius = msg.min_radius;
    }
    else {
      resolved.min_radius = 0
    }

    if (msg.max_radius !== undefined) {
      resolved.max_radius = msg.max_radius;
    }
    else {
      resolved.max_radius = 0
    }

    if (msg.filter_h_min !== undefined) {
      resolved.filter_h_min = msg.filter_h_min;
    }
    else {
      resolved.filter_h_min = 0
    }

    if (msg.filter_h_max !== undefined) {
      resolved.filter_h_max = msg.filter_h_max;
    }
    else {
      resolved.filter_h_max = 0
    }

    if (msg.filter_s_min !== undefined) {
      resolved.filter_s_min = msg.filter_s_min;
    }
    else {
      resolved.filter_s_min = 0
    }

    if (msg.filter_s_max !== undefined) {
      resolved.filter_s_max = msg.filter_s_max;
    }
    else {
      resolved.filter_s_max = 0
    }

    if (msg.filter_v_min !== undefined) {
      resolved.filter_v_min = msg.filter_v_min;
    }
    else {
      resolved.filter_v_min = 0
    }

    if (msg.filter_v_max !== undefined) {
      resolved.filter_v_max = msg.filter_v_max;
    }
    else {
      resolved.filter_v_max = 0
    }

    if (msg.ellipse_size !== undefined) {
      resolved.ellipse_size = msg.ellipse_size;
    }
    else {
      resolved.ellipse_size = 0
    }

    return resolved;
    }
};

module.exports = BallDetectorParams;
