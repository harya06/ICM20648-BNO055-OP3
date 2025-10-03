; Auto-generated. Do not edit!


(cl:in-package op3_online_walking_module_msgs-srv)


;//! \htmlinclude GetJointPose-request.msg.html

(cl:defclass <GetJointPose-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetJointPose-request (<GetJointPose-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetJointPose-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetJointPose-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name op3_online_walking_module_msgs-srv:<GetJointPose-request> is deprecated: use op3_online_walking_module_msgs-srv:GetJointPose-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetJointPose-request>) ostream)
  "Serializes a message object of type '<GetJointPose-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetJointPose-request>) istream)
  "Deserializes a message object of type '<GetJointPose-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetJointPose-request>)))
  "Returns string type for a service object of type '<GetJointPose-request>"
  "op3_online_walking_module_msgs/GetJointPoseRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetJointPose-request)))
  "Returns string type for a service object of type 'GetJointPose-request"
  "op3_online_walking_module_msgs/GetJointPoseRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetJointPose-request>)))
  "Returns md5sum for a message object of type '<GetJointPose-request>"
  "c1260c47b7c0dc9a41639ef7730118eb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetJointPose-request)))
  "Returns md5sum for a message object of type 'GetJointPose-request"
  "c1260c47b7c0dc9a41639ef7730118eb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetJointPose-request>)))
  "Returns full string definition for message of type '<GetJointPose-request>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetJointPose-request)))
  "Returns full string definition for message of type 'GetJointPose-request"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetJointPose-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetJointPose-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetJointPose-request
))
;//! \htmlinclude GetJointPose-response.msg.html

(cl:defclass <GetJointPose-response> (roslisp-msg-protocol:ros-message)
  ((pose
    :reader pose
    :initarg :pose
    :type op3_online_walking_module_msgs-msg:JointPose
    :initform (cl:make-instance 'op3_online_walking_module_msgs-msg:JointPose)))
)

(cl:defclass GetJointPose-response (<GetJointPose-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetJointPose-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetJointPose-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name op3_online_walking_module_msgs-srv:<GetJointPose-response> is deprecated: use op3_online_walking_module_msgs-srv:GetJointPose-response instead.")))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <GetJointPose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_online_walking_module_msgs-srv:pose-val is deprecated.  Use op3_online_walking_module_msgs-srv:pose instead.")
  (pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetJointPose-response>) ostream)
  "Serializes a message object of type '<GetJointPose-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetJointPose-response>) istream)
  "Deserializes a message object of type '<GetJointPose-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetJointPose-response>)))
  "Returns string type for a service object of type '<GetJointPose-response>"
  "op3_online_walking_module_msgs/GetJointPoseResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetJointPose-response)))
  "Returns string type for a service object of type 'GetJointPose-response"
  "op3_online_walking_module_msgs/GetJointPoseResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetJointPose-response>)))
  "Returns md5sum for a message object of type '<GetJointPose-response>"
  "c1260c47b7c0dc9a41639ef7730118eb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetJointPose-response)))
  "Returns md5sum for a message object of type 'GetJointPose-response"
  "c1260c47b7c0dc9a41639ef7730118eb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetJointPose-response>)))
  "Returns full string definition for message of type '<GetJointPose-response>"
  (cl:format cl:nil "op3_online_walking_module_msgs/JointPose pose~%~%~%================================================================================~%MSG: op3_online_walking_module_msgs/JointPose~%float64  mov_time~%sensor_msgs/JointState  pose~%~%================================================================================~%MSG: sensor_msgs/JointState~%# This is a message that holds data to describe the state of a set of torque controlled joints. ~%#~%# The state of each joint (revolute or prismatic) is defined by:~%#  * the position of the joint (rad or m),~%#  * the velocity of the joint (rad/s or m/s) and ~%#  * the effort that is applied in the joint (Nm or N).~%#~%# Each joint is uniquely identified by its name~%# The header specifies the time at which the joint states were recorded. All the joint states~%# in one message have to be recorded at the same time.~%#~%# This message consists of a multiple arrays, one for each part of the joint state. ~%# The goal is to make each of the fields optional. When e.g. your joints have no~%# effort associated with them, you can leave the effort array empty. ~%#~%# All arrays in this message should have the same size, or be empty.~%# This is the only way to uniquely associate the joint name with the correct~%# states.~%~%~%Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] effort~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetJointPose-response)))
  "Returns full string definition for message of type 'GetJointPose-response"
  (cl:format cl:nil "op3_online_walking_module_msgs/JointPose pose~%~%~%================================================================================~%MSG: op3_online_walking_module_msgs/JointPose~%float64  mov_time~%sensor_msgs/JointState  pose~%~%================================================================================~%MSG: sensor_msgs/JointState~%# This is a message that holds data to describe the state of a set of torque controlled joints. ~%#~%# The state of each joint (revolute or prismatic) is defined by:~%#  * the position of the joint (rad or m),~%#  * the velocity of the joint (rad/s or m/s) and ~%#  * the effort that is applied in the joint (Nm or N).~%#~%# Each joint is uniquely identified by its name~%# The header specifies the time at which the joint states were recorded. All the joint states~%# in one message have to be recorded at the same time.~%#~%# This message consists of a multiple arrays, one for each part of the joint state. ~%# The goal is to make each of the fields optional. When e.g. your joints have no~%# effort associated with them, you can leave the effort array empty. ~%#~%# All arrays in this message should have the same size, or be empty.~%# This is the only way to uniquely associate the joint name with the correct~%# states.~%~%~%Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] effort~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetJointPose-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetJointPose-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetJointPose-response
    (cl:cons ':pose (pose msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetJointPose)))
  'GetJointPose-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetJointPose)))
  'GetJointPose-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetJointPose)))
  "Returns string type for a service object of type '<GetJointPose>"
  "op3_online_walking_module_msgs/GetJointPose")