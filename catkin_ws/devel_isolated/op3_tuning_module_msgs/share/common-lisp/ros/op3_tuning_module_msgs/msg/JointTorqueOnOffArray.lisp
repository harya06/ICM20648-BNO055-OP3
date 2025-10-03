; Auto-generated. Do not edit!


(cl:in-package op3_tuning_module_msgs-msg)


;//! \htmlinclude JointTorqueOnOffArray.msg.html

(cl:defclass <JointTorqueOnOffArray> (roslisp-msg-protocol:ros-message)
  ((torque_enable_data
    :reader torque_enable_data
    :initarg :torque_enable_data
    :type (cl:vector op3_tuning_module_msgs-msg:JointTorqueOnOff)
   :initform (cl:make-array 0 :element-type 'op3_tuning_module_msgs-msg:JointTorqueOnOff :initial-element (cl:make-instance 'op3_tuning_module_msgs-msg:JointTorqueOnOff))))
)

(cl:defclass JointTorqueOnOffArray (<JointTorqueOnOffArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JointTorqueOnOffArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JointTorqueOnOffArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name op3_tuning_module_msgs-msg:<JointTorqueOnOffArray> is deprecated: use op3_tuning_module_msgs-msg:JointTorqueOnOffArray instead.")))

(cl:ensure-generic-function 'torque_enable_data-val :lambda-list '(m))
(cl:defmethod torque_enable_data-val ((m <JointTorqueOnOffArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_tuning_module_msgs-msg:torque_enable_data-val is deprecated.  Use op3_tuning_module_msgs-msg:torque_enable_data instead.")
  (torque_enable_data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JointTorqueOnOffArray>) ostream)
  "Serializes a message object of type '<JointTorqueOnOffArray>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'torque_enable_data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'torque_enable_data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JointTorqueOnOffArray>) istream)
  "Deserializes a message object of type '<JointTorqueOnOffArray>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'torque_enable_data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'torque_enable_data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'op3_tuning_module_msgs-msg:JointTorqueOnOff))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JointTorqueOnOffArray>)))
  "Returns string type for a message object of type '<JointTorqueOnOffArray>"
  "op3_tuning_module_msgs/JointTorqueOnOffArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JointTorqueOnOffArray)))
  "Returns string type for a message object of type 'JointTorqueOnOffArray"
  "op3_tuning_module_msgs/JointTorqueOnOffArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JointTorqueOnOffArray>)))
  "Returns md5sum for a message object of type '<JointTorqueOnOffArray>"
  "1ca4db772b4d802ac00aebf4469fc8bf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JointTorqueOnOffArray)))
  "Returns md5sum for a message object of type 'JointTorqueOnOffArray"
  "1ca4db772b4d802ac00aebf4469fc8bf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JointTorqueOnOffArray>)))
  "Returns full string definition for message of type '<JointTorqueOnOffArray>"
  (cl:format cl:nil "JointTorqueOnOff[] torque_enable_data~%================================================================================~%MSG: op3_tuning_module_msgs/JointTorqueOnOff~%string  joint_name~%bool    torque_enable~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JointTorqueOnOffArray)))
  "Returns full string definition for message of type 'JointTorqueOnOffArray"
  (cl:format cl:nil "JointTorqueOnOff[] torque_enable_data~%================================================================================~%MSG: op3_tuning_module_msgs/JointTorqueOnOff~%string  joint_name~%bool    torque_enable~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JointTorqueOnOffArray>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'torque_enable_data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JointTorqueOnOffArray>))
  "Converts a ROS message object to a list"
  (cl:list 'JointTorqueOnOffArray
    (cl:cons ':torque_enable_data (torque_enable_data msg))
))
