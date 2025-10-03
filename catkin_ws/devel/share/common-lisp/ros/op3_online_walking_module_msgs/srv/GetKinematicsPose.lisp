; Auto-generated. Do not edit!


(cl:in-package op3_online_walking_module_msgs-srv)


;//! \htmlinclude GetKinematicsPose-request.msg.html

(cl:defclass <GetKinematicsPose-request> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform ""))
)

(cl:defclass GetKinematicsPose-request (<GetKinematicsPose-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetKinematicsPose-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetKinematicsPose-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name op3_online_walking_module_msgs-srv:<GetKinematicsPose-request> is deprecated: use op3_online_walking_module_msgs-srv:GetKinematicsPose-request instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <GetKinematicsPose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_online_walking_module_msgs-srv:name-val is deprecated.  Use op3_online_walking_module_msgs-srv:name instead.")
  (name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetKinematicsPose-request>) ostream)
  "Serializes a message object of type '<GetKinematicsPose-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetKinematicsPose-request>) istream)
  "Deserializes a message object of type '<GetKinematicsPose-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetKinematicsPose-request>)))
  "Returns string type for a service object of type '<GetKinematicsPose-request>"
  "op3_online_walking_module_msgs/GetKinematicsPoseRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetKinematicsPose-request)))
  "Returns string type for a service object of type 'GetKinematicsPose-request"
  "op3_online_walking_module_msgs/GetKinematicsPoseRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetKinematicsPose-request>)))
  "Returns md5sum for a message object of type '<GetKinematicsPose-request>"
  "aad71eecbad83fbcb051be59f9fbb914")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetKinematicsPose-request)))
  "Returns md5sum for a message object of type 'GetKinematicsPose-request"
  "aad71eecbad83fbcb051be59f9fbb914")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetKinematicsPose-request>)))
  "Returns full string definition for message of type '<GetKinematicsPose-request>"
  (cl:format cl:nil "string    name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetKinematicsPose-request)))
  "Returns full string definition for message of type 'GetKinematicsPose-request"
  (cl:format cl:nil "string    name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetKinematicsPose-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetKinematicsPose-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetKinematicsPose-request
    (cl:cons ':name (name msg))
))
;//! \htmlinclude GetKinematicsPose-response.msg.html

(cl:defclass <GetKinematicsPose-response> (roslisp-msg-protocol:ros-message)
  ((pose
    :reader pose
    :initarg :pose
    :type op3_online_walking_module_msgs-msg:KinematicsPose
    :initform (cl:make-instance 'op3_online_walking_module_msgs-msg:KinematicsPose)))
)

(cl:defclass GetKinematicsPose-response (<GetKinematicsPose-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetKinematicsPose-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetKinematicsPose-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name op3_online_walking_module_msgs-srv:<GetKinematicsPose-response> is deprecated: use op3_online_walking_module_msgs-srv:GetKinematicsPose-response instead.")))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <GetKinematicsPose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_online_walking_module_msgs-srv:pose-val is deprecated.  Use op3_online_walking_module_msgs-srv:pose instead.")
  (pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetKinematicsPose-response>) ostream)
  "Serializes a message object of type '<GetKinematicsPose-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetKinematicsPose-response>) istream)
  "Deserializes a message object of type '<GetKinematicsPose-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetKinematicsPose-response>)))
  "Returns string type for a service object of type '<GetKinematicsPose-response>"
  "op3_online_walking_module_msgs/GetKinematicsPoseResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetKinematicsPose-response)))
  "Returns string type for a service object of type 'GetKinematicsPose-response"
  "op3_online_walking_module_msgs/GetKinematicsPoseResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetKinematicsPose-response>)))
  "Returns md5sum for a message object of type '<GetKinematicsPose-response>"
  "aad71eecbad83fbcb051be59f9fbb914")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetKinematicsPose-response)))
  "Returns md5sum for a message object of type 'GetKinematicsPose-response"
  "aad71eecbad83fbcb051be59f9fbb914")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetKinematicsPose-response>)))
  "Returns full string definition for message of type '<GetKinematicsPose-response>"
  (cl:format cl:nil "op3_online_walking_module_msgs/KinematicsPose pose~%~%~%================================================================================~%MSG: op3_online_walking_module_msgs/KinematicsPose~%string  name~%float64  mov_time~%geometry_msgs/Pose pose~%~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetKinematicsPose-response)))
  "Returns full string definition for message of type 'GetKinematicsPose-response"
  (cl:format cl:nil "op3_online_walking_module_msgs/KinematicsPose pose~%~%~%================================================================================~%MSG: op3_online_walking_module_msgs/KinematicsPose~%string  name~%float64  mov_time~%geometry_msgs/Pose pose~%~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetKinematicsPose-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetKinematicsPose-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetKinematicsPose-response
    (cl:cons ':pose (pose msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetKinematicsPose)))
  'GetKinematicsPose-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetKinematicsPose)))
  'GetKinematicsPose-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetKinematicsPose)))
  "Returns string type for a service object of type '<GetKinematicsPose>"
  "op3_online_walking_module_msgs/GetKinematicsPose")