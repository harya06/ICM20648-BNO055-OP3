; Auto-generated. Do not edit!


(cl:in-package op3_online_walking_module_msgs-msg)


;//! \htmlinclude Step2DArray.msg.html

(cl:defclass <Step2DArray> (roslisp-msg-protocol:ros-message)
  ((step_time
    :reader step_time
    :initarg :step_time
    :type cl:float
    :initform 0.0)
   (footsteps_2d
    :reader footsteps_2d
    :initarg :footsteps_2d
    :type (cl:vector op3_online_walking_module_msgs-msg:Step2D)
   :initform (cl:make-array 0 :element-type 'op3_online_walking_module_msgs-msg:Step2D :initial-element (cl:make-instance 'op3_online_walking_module_msgs-msg:Step2D))))
)

(cl:defclass Step2DArray (<Step2DArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Step2DArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Step2DArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name op3_online_walking_module_msgs-msg:<Step2DArray> is deprecated: use op3_online_walking_module_msgs-msg:Step2DArray instead.")))

(cl:ensure-generic-function 'step_time-val :lambda-list '(m))
(cl:defmethod step_time-val ((m <Step2DArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_online_walking_module_msgs-msg:step_time-val is deprecated.  Use op3_online_walking_module_msgs-msg:step_time instead.")
  (step_time m))

(cl:ensure-generic-function 'footsteps_2d-val :lambda-list '(m))
(cl:defmethod footsteps_2d-val ((m <Step2DArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_online_walking_module_msgs-msg:footsteps_2d-val is deprecated.  Use op3_online_walking_module_msgs-msg:footsteps_2d instead.")
  (footsteps_2d m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Step2DArray>) ostream)
  "Serializes a message object of type '<Step2DArray>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'step_time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'footsteps_2d))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'footsteps_2d))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Step2DArray>) istream)
  "Deserializes a message object of type '<Step2DArray>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'step_time) (roslisp-utils:decode-double-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'footsteps_2d) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'footsteps_2d)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'op3_online_walking_module_msgs-msg:Step2D))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Step2DArray>)))
  "Returns string type for a message object of type '<Step2DArray>"
  "op3_online_walking_module_msgs/Step2DArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Step2DArray)))
  "Returns string type for a message object of type 'Step2DArray"
  "op3_online_walking_module_msgs/Step2DArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Step2DArray>)))
  "Returns md5sum for a message object of type '<Step2DArray>"
  "598423594f1c61377b299ae8d55d0f04")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Step2DArray)))
  "Returns md5sum for a message object of type 'Step2DArray"
  "598423594f1c61377b299ae8d55d0f04")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Step2DArray>)))
  "Returns full string definition for message of type '<Step2DArray>"
  (cl:format cl:nil "float64 step_time~%Step2D[] footsteps_2d~%~%================================================================================~%MSG: op3_online_walking_module_msgs/Step2D~%#2D StepData~%~%geometry_msgs/Pose2D step2d   # step pose as relative offset to last leg~%~%~%# which leg to be used (left/right/no, see below)~%uint8 moving_foot   ~%~%uint8 LEFT_FOOT_SWING  = 1 # Left foot constant~%uint8 RIGHT_FOOT_SWING = 2 # Right foot constant~%uint8 STANDING         = 3 # Standing constant~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Step2DArray)))
  "Returns full string definition for message of type 'Step2DArray"
  (cl:format cl:nil "float64 step_time~%Step2D[] footsteps_2d~%~%================================================================================~%MSG: op3_online_walking_module_msgs/Step2D~%#2D StepData~%~%geometry_msgs/Pose2D step2d   # step pose as relative offset to last leg~%~%~%# which leg to be used (left/right/no, see below)~%uint8 moving_foot   ~%~%uint8 LEFT_FOOT_SWING  = 1 # Left foot constant~%uint8 RIGHT_FOOT_SWING = 2 # Right foot constant~%uint8 STANDING         = 3 # Standing constant~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Step2DArray>))
  (cl:+ 0
     8
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'footsteps_2d) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Step2DArray>))
  "Converts a ROS message object to a list"
  (cl:list 'Step2DArray
    (cl:cons ':step_time (step_time msg))
    (cl:cons ':footsteps_2d (footsteps_2d msg))
))
