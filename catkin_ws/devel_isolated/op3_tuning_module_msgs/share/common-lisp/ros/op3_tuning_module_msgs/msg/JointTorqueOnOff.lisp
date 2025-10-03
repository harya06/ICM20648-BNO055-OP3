; Auto-generated. Do not edit!


(cl:in-package op3_tuning_module_msgs-msg)


;//! \htmlinclude JointTorqueOnOff.msg.html

(cl:defclass <JointTorqueOnOff> (roslisp-msg-protocol:ros-message)
  ((joint_name
    :reader joint_name
    :initarg :joint_name
    :type cl:string
    :initform "")
   (torque_enable
    :reader torque_enable
    :initarg :torque_enable
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass JointTorqueOnOff (<JointTorqueOnOff>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JointTorqueOnOff>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JointTorqueOnOff)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name op3_tuning_module_msgs-msg:<JointTorqueOnOff> is deprecated: use op3_tuning_module_msgs-msg:JointTorqueOnOff instead.")))

(cl:ensure-generic-function 'joint_name-val :lambda-list '(m))
(cl:defmethod joint_name-val ((m <JointTorqueOnOff>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_tuning_module_msgs-msg:joint_name-val is deprecated.  Use op3_tuning_module_msgs-msg:joint_name instead.")
  (joint_name m))

(cl:ensure-generic-function 'torque_enable-val :lambda-list '(m))
(cl:defmethod torque_enable-val ((m <JointTorqueOnOff>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_tuning_module_msgs-msg:torque_enable-val is deprecated.  Use op3_tuning_module_msgs-msg:torque_enable instead.")
  (torque_enable m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JointTorqueOnOff>) ostream)
  "Serializes a message object of type '<JointTorqueOnOff>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'joint_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'joint_name))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'torque_enable) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JointTorqueOnOff>) istream)
  "Deserializes a message object of type '<JointTorqueOnOff>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'joint_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'joint_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'torque_enable) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JointTorqueOnOff>)))
  "Returns string type for a message object of type '<JointTorqueOnOff>"
  "op3_tuning_module_msgs/JointTorqueOnOff")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JointTorqueOnOff)))
  "Returns string type for a message object of type 'JointTorqueOnOff"
  "op3_tuning_module_msgs/JointTorqueOnOff")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JointTorqueOnOff>)))
  "Returns md5sum for a message object of type '<JointTorqueOnOff>"
  "b5a5bf39f4a0958f049fc5cc9d8fbd8b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JointTorqueOnOff)))
  "Returns md5sum for a message object of type 'JointTorqueOnOff"
  "b5a5bf39f4a0958f049fc5cc9d8fbd8b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JointTorqueOnOff>)))
  "Returns full string definition for message of type '<JointTorqueOnOff>"
  (cl:format cl:nil "string  joint_name~%bool    torque_enable~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JointTorqueOnOff)))
  "Returns full string definition for message of type 'JointTorqueOnOff"
  (cl:format cl:nil "string  joint_name~%bool    torque_enable~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JointTorqueOnOff>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'joint_name))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JointTorqueOnOff>))
  "Converts a ROS message object to a list"
  (cl:list 'JointTorqueOnOff
    (cl:cons ':joint_name (joint_name msg))
    (cl:cons ':torque_enable (torque_enable msg))
))
