; Auto-generated. Do not edit!


(cl:in-package op3_tuning_module_msgs-srv)


;//! \htmlinclude GetPresentJointOffsetData-request.msg.html

(cl:defclass <GetPresentJointOffsetData-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetPresentJointOffsetData-request (<GetPresentJointOffsetData-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetPresentJointOffsetData-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetPresentJointOffsetData-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name op3_tuning_module_msgs-srv:<GetPresentJointOffsetData-request> is deprecated: use op3_tuning_module_msgs-srv:GetPresentJointOffsetData-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetPresentJointOffsetData-request>) ostream)
  "Serializes a message object of type '<GetPresentJointOffsetData-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetPresentJointOffsetData-request>) istream)
  "Deserializes a message object of type '<GetPresentJointOffsetData-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetPresentJointOffsetData-request>)))
  "Returns string type for a service object of type '<GetPresentJointOffsetData-request>"
  "op3_tuning_module_msgs/GetPresentJointOffsetDataRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPresentJointOffsetData-request)))
  "Returns string type for a service object of type 'GetPresentJointOffsetData-request"
  "op3_tuning_module_msgs/GetPresentJointOffsetDataRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetPresentJointOffsetData-request>)))
  "Returns md5sum for a message object of type '<GetPresentJointOffsetData-request>"
  "fe3cab7abbd49468a5d502064853404e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetPresentJointOffsetData-request)))
  "Returns md5sum for a message object of type 'GetPresentJointOffsetData-request"
  "fe3cab7abbd49468a5d502064853404e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetPresentJointOffsetData-request>)))
  "Returns full string definition for message of type '<GetPresentJointOffsetData-request>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetPresentJointOffsetData-request)))
  "Returns full string definition for message of type 'GetPresentJointOffsetData-request"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetPresentJointOffsetData-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetPresentJointOffsetData-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetPresentJointOffsetData-request
))
;//! \htmlinclude GetPresentJointOffsetData-response.msg.html

(cl:defclass <GetPresentJointOffsetData-response> (roslisp-msg-protocol:ros-message)
  ((present_data_array
    :reader present_data_array
    :initarg :present_data_array
    :type (cl:vector op3_tuning_module_msgs-msg:JointOffsetPositionData)
   :initform (cl:make-array 0 :element-type 'op3_tuning_module_msgs-msg:JointOffsetPositionData :initial-element (cl:make-instance 'op3_tuning_module_msgs-msg:JointOffsetPositionData))))
)

(cl:defclass GetPresentJointOffsetData-response (<GetPresentJointOffsetData-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetPresentJointOffsetData-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetPresentJointOffsetData-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name op3_tuning_module_msgs-srv:<GetPresentJointOffsetData-response> is deprecated: use op3_tuning_module_msgs-srv:GetPresentJointOffsetData-response instead.")))

(cl:ensure-generic-function 'present_data_array-val :lambda-list '(m))
(cl:defmethod present_data_array-val ((m <GetPresentJointOffsetData-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_tuning_module_msgs-srv:present_data_array-val is deprecated.  Use op3_tuning_module_msgs-srv:present_data_array instead.")
  (present_data_array m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetPresentJointOffsetData-response>) ostream)
  "Serializes a message object of type '<GetPresentJointOffsetData-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'present_data_array))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'present_data_array))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetPresentJointOffsetData-response>) istream)
  "Deserializes a message object of type '<GetPresentJointOffsetData-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'present_data_array) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'present_data_array)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'op3_tuning_module_msgs-msg:JointOffsetPositionData))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetPresentJointOffsetData-response>)))
  "Returns string type for a service object of type '<GetPresentJointOffsetData-response>"
  "op3_tuning_module_msgs/GetPresentJointOffsetDataResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPresentJointOffsetData-response)))
  "Returns string type for a service object of type 'GetPresentJointOffsetData-response"
  "op3_tuning_module_msgs/GetPresentJointOffsetDataResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetPresentJointOffsetData-response>)))
  "Returns md5sum for a message object of type '<GetPresentJointOffsetData-response>"
  "fe3cab7abbd49468a5d502064853404e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetPresentJointOffsetData-response)))
  "Returns md5sum for a message object of type 'GetPresentJointOffsetData-response"
  "fe3cab7abbd49468a5d502064853404e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetPresentJointOffsetData-response>)))
  "Returns full string definition for message of type '<GetPresentJointOffsetData-response>"
  (cl:format cl:nil "JointOffsetPositionData[] present_data_array~%~%================================================================================~%MSG: op3_tuning_module_msgs/JointOffsetPositionData~%string  joint_name~%float64 goal_value~%float64 offset_value~%float64 present_value~%int32   p_gain~%int32   i_gain~%int32   d_gain~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetPresentJointOffsetData-response)))
  "Returns full string definition for message of type 'GetPresentJointOffsetData-response"
  (cl:format cl:nil "JointOffsetPositionData[] present_data_array~%~%================================================================================~%MSG: op3_tuning_module_msgs/JointOffsetPositionData~%string  joint_name~%float64 goal_value~%float64 offset_value~%float64 present_value~%int32   p_gain~%int32   i_gain~%int32   d_gain~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetPresentJointOffsetData-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'present_data_array) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetPresentJointOffsetData-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetPresentJointOffsetData-response
    (cl:cons ':present_data_array (present_data_array msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetPresentJointOffsetData)))
  'GetPresentJointOffsetData-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetPresentJointOffsetData)))
  'GetPresentJointOffsetData-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPresentJointOffsetData)))
  "Returns string type for a service object of type '<GetPresentJointOffsetData>"
  "op3_tuning_module_msgs/GetPresentJointOffsetData")