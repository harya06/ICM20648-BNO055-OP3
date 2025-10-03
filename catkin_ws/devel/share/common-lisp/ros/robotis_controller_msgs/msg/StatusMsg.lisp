; Auto-generated. Do not edit!


(cl:in-package robotis_controller_msgs-msg)


;//! \htmlinclude StatusMsg.msg.html

(cl:defclass <StatusMsg> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (type
    :reader type
    :initarg :type
    :type cl:fixnum
    :initform 0)
   (module_name
    :reader module_name
    :initarg :module_name
    :type cl:string
    :initform "")
   (status_msg
    :reader status_msg
    :initarg :status_msg
    :type cl:string
    :initform ""))
)

(cl:defclass StatusMsg (<StatusMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StatusMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StatusMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robotis_controller_msgs-msg:<StatusMsg> is deprecated: use robotis_controller_msgs-msg:StatusMsg instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <StatusMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotis_controller_msgs-msg:header-val is deprecated.  Use robotis_controller_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <StatusMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotis_controller_msgs-msg:type-val is deprecated.  Use robotis_controller_msgs-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'module_name-val :lambda-list '(m))
(cl:defmethod module_name-val ((m <StatusMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotis_controller_msgs-msg:module_name-val is deprecated.  Use robotis_controller_msgs-msg:module_name instead.")
  (module_name m))

(cl:ensure-generic-function 'status_msg-val :lambda-list '(m))
(cl:defmethod status_msg-val ((m <StatusMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotis_controller_msgs-msg:status_msg-val is deprecated.  Use robotis_controller_msgs-msg:status_msg instead.")
  (status_msg m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<StatusMsg>)))
    "Constants for message type '<StatusMsg>"
  '((:STATUS_UNKNOWN . 0)
    (:STATUS_INFO . 1)
    (:STATUS_WARN . 2)
    (:STATUS_ERROR . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'StatusMsg)))
    "Constants for message type 'StatusMsg"
  '((:STATUS_UNKNOWN . 0)
    (:STATUS_INFO . 1)
    (:STATUS_WARN . 2)
    (:STATUS_ERROR . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StatusMsg>) ostream)
  "Serializes a message object of type '<StatusMsg>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'module_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'module_name))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'status_msg))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'status_msg))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StatusMsg>) istream)
  "Deserializes a message object of type '<StatusMsg>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'module_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'module_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status_msg) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'status_msg) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StatusMsg>)))
  "Returns string type for a message object of type '<StatusMsg>"
  "robotis_controller_msgs/StatusMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StatusMsg)))
  "Returns string type for a message object of type 'StatusMsg"
  "robotis_controller_msgs/StatusMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StatusMsg>)))
  "Returns md5sum for a message object of type '<StatusMsg>"
  "8d546af394a35cb47516d4d064603220")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StatusMsg)))
  "Returns md5sum for a message object of type 'StatusMsg"
  "8d546af394a35cb47516d4d064603220")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StatusMsg>)))
  "Returns full string definition for message of type '<StatusMsg>"
  (cl:format cl:nil "# Status Constants~%uint8 STATUS_UNKNOWN = 0~%uint8 STATUS_INFO = 1~%uint8 STATUS_WARN = 2~%uint8 STATUS_ERROR = 3~%~%std_msgs/Header header~%uint8 type~%string module_name~%string status_msg~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StatusMsg)))
  "Returns full string definition for message of type 'StatusMsg"
  (cl:format cl:nil "# Status Constants~%uint8 STATUS_UNKNOWN = 0~%uint8 STATUS_INFO = 1~%uint8 STATUS_WARN = 2~%uint8 STATUS_ERROR = 3~%~%std_msgs/Header header~%uint8 type~%string module_name~%string status_msg~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StatusMsg>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     4 (cl:length (cl:slot-value msg 'module_name))
     4 (cl:length (cl:slot-value msg 'status_msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StatusMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'StatusMsg
    (cl:cons ':header (header msg))
    (cl:cons ':type (type msg))
    (cl:cons ':module_name (module_name msg))
    (cl:cons ':status_msg (status_msg msg))
))
