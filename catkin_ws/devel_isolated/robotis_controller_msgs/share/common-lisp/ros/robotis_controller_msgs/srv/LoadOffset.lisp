; Auto-generated. Do not edit!


(cl:in-package robotis_controller_msgs-srv)


;//! \htmlinclude LoadOffset-request.msg.html

(cl:defclass <LoadOffset-request> (roslisp-msg-protocol:ros-message)
  ((file_path
    :reader file_path
    :initarg :file_path
    :type cl:string
    :initform ""))
)

(cl:defclass LoadOffset-request (<LoadOffset-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LoadOffset-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LoadOffset-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robotis_controller_msgs-srv:<LoadOffset-request> is deprecated: use robotis_controller_msgs-srv:LoadOffset-request instead.")))

(cl:ensure-generic-function 'file_path-val :lambda-list '(m))
(cl:defmethod file_path-val ((m <LoadOffset-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotis_controller_msgs-srv:file_path-val is deprecated.  Use robotis_controller_msgs-srv:file_path instead.")
  (file_path m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LoadOffset-request>) ostream)
  "Serializes a message object of type '<LoadOffset-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'file_path))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'file_path))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LoadOffset-request>) istream)
  "Deserializes a message object of type '<LoadOffset-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'file_path) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'file_path) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LoadOffset-request>)))
  "Returns string type for a service object of type '<LoadOffset-request>"
  "robotis_controller_msgs/LoadOffsetRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LoadOffset-request)))
  "Returns string type for a service object of type 'LoadOffset-request"
  "robotis_controller_msgs/LoadOffsetRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LoadOffset-request>)))
  "Returns md5sum for a message object of type '<LoadOffset-request>"
  "b223f245a1d13c9e179ae29717131752")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LoadOffset-request)))
  "Returns md5sum for a message object of type 'LoadOffset-request"
  "b223f245a1d13c9e179ae29717131752")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LoadOffset-request>)))
  "Returns full string definition for message of type '<LoadOffset-request>"
  (cl:format cl:nil "string file_path~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LoadOffset-request)))
  "Returns full string definition for message of type 'LoadOffset-request"
  (cl:format cl:nil "string file_path~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LoadOffset-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'file_path))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LoadOffset-request>))
  "Converts a ROS message object to a list"
  (cl:list 'LoadOffset-request
    (cl:cons ':file_path (file_path msg))
))
;//! \htmlinclude LoadOffset-response.msg.html

(cl:defclass <LoadOffset-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass LoadOffset-response (<LoadOffset-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LoadOffset-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LoadOffset-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robotis_controller_msgs-srv:<LoadOffset-response> is deprecated: use robotis_controller_msgs-srv:LoadOffset-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <LoadOffset-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotis_controller_msgs-srv:result-val is deprecated.  Use robotis_controller_msgs-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LoadOffset-response>) ostream)
  "Serializes a message object of type '<LoadOffset-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'result) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LoadOffset-response>) istream)
  "Deserializes a message object of type '<LoadOffset-response>"
    (cl:setf (cl:slot-value msg 'result) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LoadOffset-response>)))
  "Returns string type for a service object of type '<LoadOffset-response>"
  "robotis_controller_msgs/LoadOffsetResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LoadOffset-response)))
  "Returns string type for a service object of type 'LoadOffset-response"
  "robotis_controller_msgs/LoadOffsetResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LoadOffset-response>)))
  "Returns md5sum for a message object of type '<LoadOffset-response>"
  "b223f245a1d13c9e179ae29717131752")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LoadOffset-response)))
  "Returns md5sum for a message object of type 'LoadOffset-response"
  "b223f245a1d13c9e179ae29717131752")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LoadOffset-response>)))
  "Returns full string definition for message of type '<LoadOffset-response>"
  (cl:format cl:nil "bool result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LoadOffset-response)))
  "Returns full string definition for message of type 'LoadOffset-response"
  (cl:format cl:nil "bool result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LoadOffset-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LoadOffset-response>))
  "Converts a ROS message object to a list"
  (cl:list 'LoadOffset-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'LoadOffset)))
  'LoadOffset-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'LoadOffset)))
  'LoadOffset-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LoadOffset)))
  "Returns string type for a service object of type '<LoadOffset>"
  "robotis_controller_msgs/LoadOffset")