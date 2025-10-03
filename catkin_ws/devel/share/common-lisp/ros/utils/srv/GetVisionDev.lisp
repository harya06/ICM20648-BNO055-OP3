; Auto-generated. Do not edit!


(cl:in-package utils-srv)


;//! \htmlinclude GetVisionDev-request.msg.html

(cl:defclass <GetVisionDev-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetVisionDev-request (<GetVisionDev-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetVisionDev-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetVisionDev-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name utils-srv:<GetVisionDev-request> is deprecated: use utils-srv:GetVisionDev-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetVisionDev-request>) ostream)
  "Serializes a message object of type '<GetVisionDev-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetVisionDev-request>) istream)
  "Deserializes a message object of type '<GetVisionDev-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetVisionDev-request>)))
  "Returns string type for a service object of type '<GetVisionDev-request>"
  "utils/GetVisionDevRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetVisionDev-request)))
  "Returns string type for a service object of type 'GetVisionDev-request"
  "utils/GetVisionDevRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetVisionDev-request>)))
  "Returns md5sum for a message object of type '<GetVisionDev-request>"
  "28d20ec34b723ab37e9c2029519bbce4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetVisionDev-request)))
  "Returns md5sum for a message object of type 'GetVisionDev-request"
  "28d20ec34b723ab37e9c2029519bbce4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetVisionDev-request>)))
  "Returns full string definition for message of type '<GetVisionDev-request>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetVisionDev-request)))
  "Returns full string definition for message of type 'GetVisionDev-request"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetVisionDev-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetVisionDev-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetVisionDev-request
))
;//! \htmlinclude GetVisionDev-response.msg.html

(cl:defclass <GetVisionDev-response> (roslisp-msg-protocol:ros-message)
  ((dev
    :reader dev
    :initarg :dev
    :type cl:integer
    :initform 0))
)

(cl:defclass GetVisionDev-response (<GetVisionDev-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetVisionDev-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetVisionDev-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name utils-srv:<GetVisionDev-response> is deprecated: use utils-srv:GetVisionDev-response instead.")))

(cl:ensure-generic-function 'dev-val :lambda-list '(m))
(cl:defmethod dev-val ((m <GetVisionDev-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader utils-srv:dev-val is deprecated.  Use utils-srv:dev instead.")
  (dev m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetVisionDev-response>) ostream)
  "Serializes a message object of type '<GetVisionDev-response>"
  (cl:let* ((signed (cl:slot-value msg 'dev)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetVisionDev-response>) istream)
  "Deserializes a message object of type '<GetVisionDev-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'dev) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetVisionDev-response>)))
  "Returns string type for a service object of type '<GetVisionDev-response>"
  "utils/GetVisionDevResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetVisionDev-response)))
  "Returns string type for a service object of type 'GetVisionDev-response"
  "utils/GetVisionDevResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetVisionDev-response>)))
  "Returns md5sum for a message object of type '<GetVisionDev-response>"
  "28d20ec34b723ab37e9c2029519bbce4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetVisionDev-response)))
  "Returns md5sum for a message object of type 'GetVisionDev-response"
  "28d20ec34b723ab37e9c2029519bbce4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetVisionDev-response>)))
  "Returns full string definition for message of type '<GetVisionDev-response>"
  (cl:format cl:nil "int64 dev~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetVisionDev-response)))
  "Returns full string definition for message of type 'GetVisionDev-response"
  (cl:format cl:nil "int64 dev~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetVisionDev-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetVisionDev-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetVisionDev-response
    (cl:cons ':dev (dev msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetVisionDev)))
  'GetVisionDev-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetVisionDev)))
  'GetVisionDev-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetVisionDev)))
  "Returns string type for a service object of type '<GetVisionDev>"
  "utils/GetVisionDev")