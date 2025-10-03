; Auto-generated. Do not edit!


(cl:in-package robotis_controller_msgs-srv)


;//! \htmlinclude SetModule-request.msg.html

(cl:defclass <SetModule-request> (roslisp-msg-protocol:ros-message)
  ((module_name
    :reader module_name
    :initarg :module_name
    :type cl:string
    :initform ""))
)

(cl:defclass SetModule-request (<SetModule-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetModule-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetModule-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robotis_controller_msgs-srv:<SetModule-request> is deprecated: use robotis_controller_msgs-srv:SetModule-request instead.")))

(cl:ensure-generic-function 'module_name-val :lambda-list '(m))
(cl:defmethod module_name-val ((m <SetModule-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotis_controller_msgs-srv:module_name-val is deprecated.  Use robotis_controller_msgs-srv:module_name instead.")
  (module_name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetModule-request>) ostream)
  "Serializes a message object of type '<SetModule-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'module_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'module_name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetModule-request>) istream)
  "Deserializes a message object of type '<SetModule-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'module_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'module_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetModule-request>)))
  "Returns string type for a service object of type '<SetModule-request>"
  "robotis_controller_msgs/SetModuleRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetModule-request)))
  "Returns string type for a service object of type 'SetModule-request"
  "robotis_controller_msgs/SetModuleRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetModule-request>)))
  "Returns md5sum for a message object of type '<SetModule-request>"
  "fe41ec40765aa76b66cdabcf74584706")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetModule-request)))
  "Returns md5sum for a message object of type 'SetModule-request"
  "fe41ec40765aa76b66cdabcf74584706")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetModule-request>)))
  "Returns full string definition for message of type '<SetModule-request>"
  (cl:format cl:nil "string module_name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetModule-request)))
  "Returns full string definition for message of type 'SetModule-request"
  (cl:format cl:nil "string module_name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetModule-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'module_name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetModule-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetModule-request
    (cl:cons ':module_name (module_name msg))
))
;//! \htmlinclude SetModule-response.msg.html

(cl:defclass <SetModule-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetModule-response (<SetModule-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetModule-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetModule-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robotis_controller_msgs-srv:<SetModule-response> is deprecated: use robotis_controller_msgs-srv:SetModule-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <SetModule-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotis_controller_msgs-srv:result-val is deprecated.  Use robotis_controller_msgs-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetModule-response>) ostream)
  "Serializes a message object of type '<SetModule-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'result) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetModule-response>) istream)
  "Deserializes a message object of type '<SetModule-response>"
    (cl:setf (cl:slot-value msg 'result) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetModule-response>)))
  "Returns string type for a service object of type '<SetModule-response>"
  "robotis_controller_msgs/SetModuleResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetModule-response)))
  "Returns string type for a service object of type 'SetModule-response"
  "robotis_controller_msgs/SetModuleResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetModule-response>)))
  "Returns md5sum for a message object of type '<SetModule-response>"
  "fe41ec40765aa76b66cdabcf74584706")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetModule-response)))
  "Returns md5sum for a message object of type 'SetModule-response"
  "fe41ec40765aa76b66cdabcf74584706")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetModule-response>)))
  "Returns full string definition for message of type '<SetModule-response>"
  (cl:format cl:nil "bool result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetModule-response)))
  "Returns full string definition for message of type 'SetModule-response"
  (cl:format cl:nil "bool result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetModule-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetModule-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetModule-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetModule)))
  'SetModule-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetModule)))
  'SetModule-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetModule)))
  "Returns string type for a service object of type '<SetModule>"
  "robotis_controller_msgs/SetModule")