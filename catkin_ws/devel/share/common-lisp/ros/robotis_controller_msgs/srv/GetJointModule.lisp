; Auto-generated. Do not edit!


(cl:in-package robotis_controller_msgs-srv)


;//! \htmlinclude GetJointModule-request.msg.html

(cl:defclass <GetJointModule-request> (roslisp-msg-protocol:ros-message)
  ((joint_name
    :reader joint_name
    :initarg :joint_name
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass GetJointModule-request (<GetJointModule-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetJointModule-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetJointModule-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robotis_controller_msgs-srv:<GetJointModule-request> is deprecated: use robotis_controller_msgs-srv:GetJointModule-request instead.")))

(cl:ensure-generic-function 'joint_name-val :lambda-list '(m))
(cl:defmethod joint_name-val ((m <GetJointModule-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotis_controller_msgs-srv:joint_name-val is deprecated.  Use robotis_controller_msgs-srv:joint_name instead.")
  (joint_name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetJointModule-request>) ostream)
  "Serializes a message object of type '<GetJointModule-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'joint_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'joint_name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetJointModule-request>) istream)
  "Deserializes a message object of type '<GetJointModule-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'joint_name) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'joint_name)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetJointModule-request>)))
  "Returns string type for a service object of type '<GetJointModule-request>"
  "robotis_controller_msgs/GetJointModuleRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetJointModule-request)))
  "Returns string type for a service object of type 'GetJointModule-request"
  "robotis_controller_msgs/GetJointModuleRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetJointModule-request>)))
  "Returns md5sum for a message object of type '<GetJointModule-request>"
  "55513368a0fa6f6cbd22bf1bfb152faa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetJointModule-request)))
  "Returns md5sum for a message object of type 'GetJointModule-request"
  "55513368a0fa6f6cbd22bf1bfb152faa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetJointModule-request>)))
  "Returns full string definition for message of type '<GetJointModule-request>"
  (cl:format cl:nil "string[] joint_name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetJointModule-request)))
  "Returns full string definition for message of type 'GetJointModule-request"
  (cl:format cl:nil "string[] joint_name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetJointModule-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'joint_name) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetJointModule-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetJointModule-request
    (cl:cons ':joint_name (joint_name msg))
))
;//! \htmlinclude GetJointModule-response.msg.html

(cl:defclass <GetJointModule-response> (roslisp-msg-protocol:ros-message)
  ((joint_name
    :reader joint_name
    :initarg :joint_name
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (module_name
    :reader module_name
    :initarg :module_name
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass GetJointModule-response (<GetJointModule-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetJointModule-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetJointModule-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robotis_controller_msgs-srv:<GetJointModule-response> is deprecated: use robotis_controller_msgs-srv:GetJointModule-response instead.")))

(cl:ensure-generic-function 'joint_name-val :lambda-list '(m))
(cl:defmethod joint_name-val ((m <GetJointModule-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotis_controller_msgs-srv:joint_name-val is deprecated.  Use robotis_controller_msgs-srv:joint_name instead.")
  (joint_name m))

(cl:ensure-generic-function 'module_name-val :lambda-list '(m))
(cl:defmethod module_name-val ((m <GetJointModule-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotis_controller_msgs-srv:module_name-val is deprecated.  Use robotis_controller_msgs-srv:module_name instead.")
  (module_name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetJointModule-response>) ostream)
  "Serializes a message object of type '<GetJointModule-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'joint_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'joint_name))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'module_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'module_name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetJointModule-response>) istream)
  "Deserializes a message object of type '<GetJointModule-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'joint_name) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'joint_name)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'module_name) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'module_name)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetJointModule-response>)))
  "Returns string type for a service object of type '<GetJointModule-response>"
  "robotis_controller_msgs/GetJointModuleResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetJointModule-response)))
  "Returns string type for a service object of type 'GetJointModule-response"
  "robotis_controller_msgs/GetJointModuleResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetJointModule-response>)))
  "Returns md5sum for a message object of type '<GetJointModule-response>"
  "55513368a0fa6f6cbd22bf1bfb152faa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetJointModule-response)))
  "Returns md5sum for a message object of type 'GetJointModule-response"
  "55513368a0fa6f6cbd22bf1bfb152faa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetJointModule-response>)))
  "Returns full string definition for message of type '<GetJointModule-response>"
  (cl:format cl:nil "string[] joint_name~%string[] module_name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetJointModule-response)))
  "Returns full string definition for message of type 'GetJointModule-response"
  (cl:format cl:nil "string[] joint_name~%string[] module_name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetJointModule-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'joint_name) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'module_name) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetJointModule-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetJointModule-response
    (cl:cons ':joint_name (joint_name msg))
    (cl:cons ':module_name (module_name msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetJointModule)))
  'GetJointModule-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetJointModule)))
  'GetJointModule-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetJointModule)))
  "Returns string type for a service object of type '<GetJointModule>"
  "robotis_controller_msgs/GetJointModule")