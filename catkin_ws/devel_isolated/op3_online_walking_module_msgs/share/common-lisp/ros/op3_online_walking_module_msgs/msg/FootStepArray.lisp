; Auto-generated. Do not edit!


(cl:in-package op3_online_walking_module_msgs-msg)


;//! \htmlinclude FootStepArray.msg.html

(cl:defclass <FootStepArray> (roslisp-msg-protocol:ros-message)
  ((moving_foot
    :reader moving_foot
    :initarg :moving_foot
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (data
    :reader data
    :initarg :data
    :type (cl:vector geometry_msgs-msg:Pose2D)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Pose2D :initial-element (cl:make-instance 'geometry_msgs-msg:Pose2D))))
)

(cl:defclass FootStepArray (<FootStepArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FootStepArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FootStepArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name op3_online_walking_module_msgs-msg:<FootStepArray> is deprecated: use op3_online_walking_module_msgs-msg:FootStepArray instead.")))

(cl:ensure-generic-function 'moving_foot-val :lambda-list '(m))
(cl:defmethod moving_foot-val ((m <FootStepArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_online_walking_module_msgs-msg:moving_foot-val is deprecated.  Use op3_online_walking_module_msgs-msg:moving_foot instead.")
  (moving_foot m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <FootStepArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_online_walking_module_msgs-msg:data-val is deprecated.  Use op3_online_walking_module_msgs-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<FootStepArray>)))
    "Constants for message type '<FootStepArray>"
  '((:LEFT_FOOT . 0)
    (:RIGHT_FOOT . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'FootStepArray)))
    "Constants for message type 'FootStepArray"
  '((:LEFT_FOOT . 0)
    (:RIGHT_FOOT . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FootStepArray>) ostream)
  "Serializes a message object of type '<FootStepArray>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'moving_foot))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'moving_foot))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FootStepArray>) istream)
  "Deserializes a message object of type '<FootStepArray>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'moving_foot) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'moving_foot)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Pose2D))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FootStepArray>)))
  "Returns string type for a message object of type '<FootStepArray>"
  "op3_online_walking_module_msgs/FootStepArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FootStepArray)))
  "Returns string type for a message object of type 'FootStepArray"
  "op3_online_walking_module_msgs/FootStepArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FootStepArray>)))
  "Returns md5sum for a message object of type '<FootStepArray>"
  "d024e75ed0a26f72c71cd63c2720fb63")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FootStepArray)))
  "Returns md5sum for a message object of type 'FootStepArray"
  "d024e75ed0a26f72c71cd63c2720fb63")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FootStepArray>)))
  "Returns full string definition for message of type '<FootStepArray>"
  (cl:format cl:nil "~%int32 LEFT_FOOT  = 0 # Left foot constant~%int32 RIGHT_FOOT = 1 # Right foot constant~%~%int32[]                moving_foot~%geometry_msgs/Pose2D[] data~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FootStepArray)))
  "Returns full string definition for message of type 'FootStepArray"
  (cl:format cl:nil "~%int32 LEFT_FOOT  = 0 # Left foot constant~%int32 RIGHT_FOOT = 1 # Right foot constant~%~%int32[]                moving_foot~%geometry_msgs/Pose2D[] data~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FootStepArray>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'moving_foot) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FootStepArray>))
  "Converts a ROS message object to a list"
  (cl:list 'FootStepArray
    (cl:cons ':moving_foot (moving_foot msg))
    (cl:cons ':data (data msg))
))
