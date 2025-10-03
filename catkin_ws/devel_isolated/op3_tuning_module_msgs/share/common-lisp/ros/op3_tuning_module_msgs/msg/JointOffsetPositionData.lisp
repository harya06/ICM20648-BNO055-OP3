; Auto-generated. Do not edit!


(cl:in-package op3_tuning_module_msgs-msg)


;//! \htmlinclude JointOffsetPositionData.msg.html

(cl:defclass <JointOffsetPositionData> (roslisp-msg-protocol:ros-message)
  ((joint_name
    :reader joint_name
    :initarg :joint_name
    :type cl:string
    :initform "")
   (goal_value
    :reader goal_value
    :initarg :goal_value
    :type cl:float
    :initform 0.0)
   (offset_value
    :reader offset_value
    :initarg :offset_value
    :type cl:float
    :initform 0.0)
   (present_value
    :reader present_value
    :initarg :present_value
    :type cl:float
    :initform 0.0)
   (p_gain
    :reader p_gain
    :initarg :p_gain
    :type cl:integer
    :initform 0)
   (i_gain
    :reader i_gain
    :initarg :i_gain
    :type cl:integer
    :initform 0)
   (d_gain
    :reader d_gain
    :initarg :d_gain
    :type cl:integer
    :initform 0))
)

(cl:defclass JointOffsetPositionData (<JointOffsetPositionData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JointOffsetPositionData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JointOffsetPositionData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name op3_tuning_module_msgs-msg:<JointOffsetPositionData> is deprecated: use op3_tuning_module_msgs-msg:JointOffsetPositionData instead.")))

(cl:ensure-generic-function 'joint_name-val :lambda-list '(m))
(cl:defmethod joint_name-val ((m <JointOffsetPositionData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_tuning_module_msgs-msg:joint_name-val is deprecated.  Use op3_tuning_module_msgs-msg:joint_name instead.")
  (joint_name m))

(cl:ensure-generic-function 'goal_value-val :lambda-list '(m))
(cl:defmethod goal_value-val ((m <JointOffsetPositionData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_tuning_module_msgs-msg:goal_value-val is deprecated.  Use op3_tuning_module_msgs-msg:goal_value instead.")
  (goal_value m))

(cl:ensure-generic-function 'offset_value-val :lambda-list '(m))
(cl:defmethod offset_value-val ((m <JointOffsetPositionData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_tuning_module_msgs-msg:offset_value-val is deprecated.  Use op3_tuning_module_msgs-msg:offset_value instead.")
  (offset_value m))

(cl:ensure-generic-function 'present_value-val :lambda-list '(m))
(cl:defmethod present_value-val ((m <JointOffsetPositionData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_tuning_module_msgs-msg:present_value-val is deprecated.  Use op3_tuning_module_msgs-msg:present_value instead.")
  (present_value m))

(cl:ensure-generic-function 'p_gain-val :lambda-list '(m))
(cl:defmethod p_gain-val ((m <JointOffsetPositionData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_tuning_module_msgs-msg:p_gain-val is deprecated.  Use op3_tuning_module_msgs-msg:p_gain instead.")
  (p_gain m))

(cl:ensure-generic-function 'i_gain-val :lambda-list '(m))
(cl:defmethod i_gain-val ((m <JointOffsetPositionData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_tuning_module_msgs-msg:i_gain-val is deprecated.  Use op3_tuning_module_msgs-msg:i_gain instead.")
  (i_gain m))

(cl:ensure-generic-function 'd_gain-val :lambda-list '(m))
(cl:defmethod d_gain-val ((m <JointOffsetPositionData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_tuning_module_msgs-msg:d_gain-val is deprecated.  Use op3_tuning_module_msgs-msg:d_gain instead.")
  (d_gain m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JointOffsetPositionData>) ostream)
  "Serializes a message object of type '<JointOffsetPositionData>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'joint_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'joint_name))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'goal_value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'offset_value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'present_value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'p_gain)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'i_gain)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'd_gain)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JointOffsetPositionData>) istream)
  "Deserializes a message object of type '<JointOffsetPositionData>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'joint_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'joint_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'goal_value) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'offset_value) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'present_value) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'p_gain) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'i_gain) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'd_gain) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JointOffsetPositionData>)))
  "Returns string type for a message object of type '<JointOffsetPositionData>"
  "op3_tuning_module_msgs/JointOffsetPositionData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JointOffsetPositionData)))
  "Returns string type for a message object of type 'JointOffsetPositionData"
  "op3_tuning_module_msgs/JointOffsetPositionData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JointOffsetPositionData>)))
  "Returns md5sum for a message object of type '<JointOffsetPositionData>"
  "7fa2aed3bed37b6302a45de21d81e20b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JointOffsetPositionData)))
  "Returns md5sum for a message object of type 'JointOffsetPositionData"
  "7fa2aed3bed37b6302a45de21d81e20b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JointOffsetPositionData>)))
  "Returns full string definition for message of type '<JointOffsetPositionData>"
  (cl:format cl:nil "string  joint_name~%float64 goal_value~%float64 offset_value~%float64 present_value~%int32   p_gain~%int32   i_gain~%int32   d_gain~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JointOffsetPositionData)))
  "Returns full string definition for message of type 'JointOffsetPositionData"
  (cl:format cl:nil "string  joint_name~%float64 goal_value~%float64 offset_value~%float64 present_value~%int32   p_gain~%int32   i_gain~%int32   d_gain~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JointOffsetPositionData>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'joint_name))
     8
     8
     8
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JointOffsetPositionData>))
  "Converts a ROS message object to a list"
  (cl:list 'JointOffsetPositionData
    (cl:cons ':joint_name (joint_name msg))
    (cl:cons ':goal_value (goal_value msg))
    (cl:cons ':offset_value (offset_value msg))
    (cl:cons ':present_value (present_value msg))
    (cl:cons ':p_gain (p_gain msg))
    (cl:cons ':i_gain (i_gain msg))
    (cl:cons ':d_gain (d_gain msg))
))
