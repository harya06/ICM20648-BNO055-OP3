; Auto-generated. Do not edit!


(cl:in-package robotis_controller_msgs-srv)


;//! \htmlinclude SetJointModule-request.msg.html

(cl:defclass <SetJointModule-request> (roslisp-msg-protocol:ros-message)
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

(cl:defclass SetJointModule-request (<SetJointModule-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetJointModule-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetJointModule-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robotis_controller_msgs-srv:<SetJointModule-request> is deprecated: use robotis_controller_msgs-srv:SetJointModule-request instead.")))

(cl:ensure-generic-function 'joint_name-val :lambda-list '(m))
(cl:defmethod joint_name-val ((m <SetJointModule-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotis_controller_msgs-srv:joint_name-val is deprecated.  Use robotis_controller_msgs-srv:joint_name instead.")
  (joint_name m))

(cl:ensure-generic-function 'module_name-val :lambda-list '(m))
(cl:defmethod module_name-val ((m <SetJointModule-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotis_controller_msgs-srv:module_name-val is deprecated.  Use robotis_controller_msgs-srv:module_name instead.")
  (module_name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetJointModule-request>) ostream)
  "Serializes a message object of type '<SetJointModule-request>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetJointModule-request>) istream)
  "Deserializes a message object of type '<SetJointModule-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetJointModule-request>)))
  "Returns string type for a service object of type '<SetJointModule-request>"
  "robotis_controller_msgs/SetJointModuleRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetJointModule-request)))
  "Returns string type for a service object of type 'SetJointModule-request"
  "robotis_controller_msgs/SetJointModuleRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetJointModule-request>)))
  "Returns md5sum for a message object of type '<SetJointModule-request>"
  "018bd00fbf52c888e80425821afd0a3d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetJointModule-request)))
  "Returns md5sum for a message object of type 'SetJointModule-request"
  "018bd00fbf52c888e80425821afd0a3d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetJointModule-request>)))
  "Returns full string definition for message of type '<SetJointModule-request>"
  (cl:format cl:nil "string[] joint_name~%string[] module_name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetJointModule-request)))
  "Returns full string definition for message of type 'SetJointModule-request"
  (cl:format cl:nil "string[] joint_name~%string[] module_name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetJointModule-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'joint_name) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'module_name) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetJointModule-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetJointModule-request
    (cl:cons ':joint_name (joint_name msg))
    (cl:cons ':module_name (module_name msg))
))
;//! \htmlinclude SetJointModule-response.msg.html

(cl:defclass <SetJointModule-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetJointModule-response (<SetJointModule-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetJointModule-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetJointModule-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robotis_controller_msgs-srv:<SetJointModule-response> is deprecated: use robotis_controller_msgs-srv:SetJointModule-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <SetJointModule-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotis_controller_msgs-srv:result-val is deprecated.  Use robotis_controller_msgs-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetJointModule-response>) ostream)
  "Serializes a message object of type '<SetJointModule-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'result) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetJointModule-response>) istream)
  "Deserializes a message object of type '<SetJointModule-response>"
    (cl:setf (cl:slot-value msg 'result) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetJointModule-response>)))
  "Returns string type for a service object of type '<SetJointModule-response>"
  "robotis_controller_msgs/SetJointModuleResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetJointModule-response)))
  "Returns string type for a service object of type 'SetJointModule-response"
  "robotis_controller_msgs/SetJointModuleResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetJointModule-response>)))
  "Returns md5sum for a message object of type '<SetJointModule-response>"
  "018bd00fbf52c888e80425821afd0a3d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetJointModule-response)))
  "Returns md5sum for a message object of type 'SetJointModule-response"
  "018bd00fbf52c888e80425821afd0a3d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetJointModule-response>)))
  "Returns full string definition for message of type '<SetJointModule-response>"
  (cl:format cl:nil "bool result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetJointModule-response)))
  "Returns full string definition for message of type 'SetJointModule-response"
  (cl:format cl:nil "bool result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetJointModule-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetJointModule-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetJointModule-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetJointModule)))
  'SetJointModule-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetJointModule)))
  'SetJointModule-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetJointModule)))
  "Returns string type for a service object of type '<SetJointModule>"
  "robotis_controller_msgs/SetJointModule")