// Auto-generated. Do not edit!

// (in-package op3_online_walking_module_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let sensor_msgs = _finder('sensor_msgs');

//-----------------------------------------------------------

class JointPose {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.mov_time = null;
      this.pose = null;
    }
    else {
      if (initObj.hasOwnProperty('mov_time')) {
        this.mov_time = initObj.mov_time
      }
      else {
        this.mov_time = 0.0;
      }
      if (initObj.hasOwnProperty('pose')) {
        this.pose = initObj.pose
      }
      else {
        this.pose = new sensor_msgs.msg.JointState();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type JointPose
    // Serialize message field [mov_time]
    bufferOffset = _serializer.float64(obj.mov_time, buffer, bufferOffset);
    // Serialize message field [pose]
    bufferOffset = sensor_msgs.msg.JointState.serialize(obj.pose, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type JointPose
    let len;
    let data = new JointPose(null);
    // Deserialize message field [mov_time]
    data.mov_time = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [pose]
    data.pose = sensor_msgs.msg.JointState.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += sensor_msgs.msg.JointState.getMessageSize(object.pose);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'op3_online_walking_module_msgs/JointPose';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '93e783e02761adffe02faae8c7feff29';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64  mov_time
    sensor_msgs/JointState  pose
    
    ================================================================================
    MSG: sensor_msgs/JointState
    # This is a message that holds data to describe the state of a set of torque controlled joints. 
    #
    # The state of each joint (revolute or prismatic) is defined by:
    #  * the position of the joint (rad or m),
    #  * the velocity of the joint (rad/s or m/s) and 
    #  * the effort that is applied in the joint (Nm or N).
    #
    # Each joint is uniquely identified by its name
    # The header specifies the time at which the joint states were recorded. All the joint states
    # in one message have to be recorded at the same time.
    #
    # This message consists of a multiple arrays, one for each part of the joint state. 
    # The goal is to make each of the fields optional. When e.g. your joints have no
    # effort associated with them, you can leave the effort array empty. 
    #
    # All arrays in this message should have the same size, or be empty.
    # This is the only way to uniquely associate the joint name with the correct
    # states.
    
    
    Header header
    
    string[] name
    float64[] position
    float64[] velocity
    float64[] effort
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    # 0: no frame
    # 1: global frame
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new JointPose(null);
    if (msg.mov_time !== undefined) {
      resolved.mov_time = msg.mov_time;
    }
    else {
      resolved.mov_time = 0.0
    }

    if (msg.pose !== undefined) {
      resolved.pose = sensor_msgs.msg.JointState.Resolve(msg.pose)
    }
    else {
      resolved.pose = new sensor_msgs.msg.JointState()
    }

    return resolved;
    }
};

module.exports = JointPose;
