; Auto-generated. Do not edit!


(cl:in-package op3_online_walking_module_msgs-msg)


;//! \htmlinclude JointPose.msg.html

(cl:defclass <JointPose> (roslisp-msg-protocol:ros-message)
  ((mov_time
    :reader mov_time
    :initarg :mov_time
    :type cl:float
    :initform 0.0)
   (pose
    :reader pose
    :initarg :pose
    :type sensor_msgs-msg:JointState
    :initform (cl:make-instance 'sensor_msgs-msg:JointState)))
)

(cl:defclass JointPose (<JointPose>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JointPose>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JointPose)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name op3_online_walking_module_msgs-msg:<JointPose> is deprecated: use op3_online_walking_module_msgs-msg:JointPose instead.")))

(cl:ensure-generic-function 'mov_time-val :lambda-list '(m))
(cl:defmethod mov_time-val ((m <JointPose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_online_walking_module_msgs-msg:mov_time-val is deprecated.  Use op3_online_walking_module_msgs-msg:mov_time instead.")
  (mov_time m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <JointPose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_online_walking_module_msgs-msg:pose-val is deprecated.  Use op3_online_walking_module_msgs-msg:pose instead.")
  (pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JointPose>) ostream)
  "Serializes a message object of type '<JointPose>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'mov_time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JointPose>) istream)
  "Deserializes a message object of type '<JointPose>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'mov_time) (roslisp-utils:decode-double-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JointPose>)))
  "Returns string type for a message object of type '<JointPose>"
  "op3_online_walking_module_msgs/JointPose")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JointPose)))
  "Returns string type for a message object of type 'JointPose"
  "op3_online_walking_module_msgs/JointPose")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JointPose>)))
  "Returns md5sum for a message object of type '<JointPose>"
  "93e783e02761adffe02faae8c7feff29")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JointPose)))
  "Returns md5sum for a message object of type 'JointPose"
  "93e783e02761adffe02faae8c7feff29")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JointPose>)))
  "Returns full string definition for message of type '<JointPose>"
  (cl:format cl:nil "float64  mov_time~%sensor_msgs/JointState  pose~%~%================================================================================~%MSG: sensor_msgs/JointState~%# This is a message that holds data to describe the state of a set of torque controlled joints. ~%#~%# The state of each joint (revolute or prismatic) is defined by:~%#  * the position of the joint (rad or m),~%#  * the velocity of the joint (rad/s or m/s) and ~%#  * the effort that is applied in the joint (Nm or N).~%#~%# Each joint is uniquely identified by its name~%# The header specifies the time at which the joint states were recorded. All the joint states~%# in one message have to be recorded at the same time.~%#~%# This message consists of a multiple arrays, one for each part of the joint state. ~%# The goal is to make each of the fields optional. When e.g. your joints have no~%# effort associated with them, you can leave the effort array empty. ~%#~%# All arrays in this message should have the same size, or be empty.~%# This is the only way to uniquely associate the joint name with the correct~%# states.~%~%~%Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] effort~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JointPose)))
  "Returns full string definition for message of type 'JointPose"
  (cl:format cl:nil "float64  mov_time~%sensor_msgs/JointState  pose~%~%================================================================================~%MSG: sensor_msgs/JointState~%# This is a message that holds data to describe the state of a set of torque controlled joints. ~%#~%# The state of each joint (revolute or prismatic) is defined by:~%#  * the position of the joint (rad or m),~%#  * the velocity of the joint (rad/s or m/s) and ~%#  * the effort that is applied in the joint (Nm or N).~%#~%# Each joint is uniquely identified by its name~%# The header specifies the time at which the joint states were recorded. All the joint states~%# in one message have to be recorded at the same time.~%#~%# This message consists of a multiple arrays, one for each part of the joint state. ~%# The goal is to make each of the fields optional. When e.g. your joints have no~%# effort associated with them, you can leave the effort array empty. ~%#~%# All arrays in this message should have the same size, or be empty.~%# This is the only way to uniquely associate the joint name with the correct~%# states.~%~%~%Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] effort~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JointPose>))
  (cl:+ 0
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JointPose>))
  "Converts a ROS message object to a list"
  (cl:list 'JointPose
    (cl:cons ':mov_time (mov_time msg))
    (cl:cons ':pose (pose msg))
))
