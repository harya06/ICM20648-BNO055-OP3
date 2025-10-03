; Auto-generated. Do not edit!


(cl:in-package op3_online_walking_module_msgs-msg)


;//! \htmlinclude Step2D.msg.html

(cl:defclass <Step2D> (roslisp-msg-protocol:ros-message)
  ((step2d
    :reader step2d
    :initarg :step2d
    :type geometry_msgs-msg:Pose2D
    :initform (cl:make-instance 'geometry_msgs-msg:Pose2D))
   (moving_foot
    :reader moving_foot
    :initarg :moving_foot
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Step2D (<Step2D>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Step2D>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Step2D)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name op3_online_walking_module_msgs-msg:<Step2D> is deprecated: use op3_online_walking_module_msgs-msg:Step2D instead.")))

(cl:ensure-generic-function 'step2d-val :lambda-list '(m))
(cl:defmethod step2d-val ((m <Step2D>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_online_walking_module_msgs-msg:step2d-val is deprecated.  Use op3_online_walking_module_msgs-msg:step2d instead.")
  (step2d m))

(cl:ensure-generic-function 'moving_foot-val :lambda-list '(m))
(cl:defmethod moving_foot-val ((m <Step2D>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_online_walking_module_msgs-msg:moving_foot-val is deprecated.  Use op3_online_walking_module_msgs-msg:moving_foot instead.")
  (moving_foot m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Step2D>)))
    "Constants for message type '<Step2D>"
  '((:LEFT_FOOT_SWING . 1)
    (:RIGHT_FOOT_SWING . 2)
    (:STANDING . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Step2D)))
    "Constants for message type 'Step2D"
  '((:LEFT_FOOT_SWING . 1)
    (:RIGHT_FOOT_SWING . 2)
    (:STANDING . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Step2D>) ostream)
  "Serializes a message object of type '<Step2D>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'step2d) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'moving_foot)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Step2D>) istream)
  "Deserializes a message object of type '<Step2D>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'step2d) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'moving_foot)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Step2D>)))
  "Returns string type for a message object of type '<Step2D>"
  "op3_online_walking_module_msgs/Step2D")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Step2D)))
  "Returns string type for a message object of type 'Step2D"
  "op3_online_walking_module_msgs/Step2D")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Step2D>)))
  "Returns md5sum for a message object of type '<Step2D>"
  "8b716dffcd181458918724c59549dd00")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Step2D)))
  "Returns md5sum for a message object of type 'Step2D"
  "8b716dffcd181458918724c59549dd00")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Step2D>)))
  "Returns full string definition for message of type '<Step2D>"
  (cl:format cl:nil "#2D StepData~%~%geometry_msgs/Pose2D step2d   # step pose as relative offset to last leg~%~%~%# which leg to be used (left/right/no, see below)~%uint8 moving_foot   ~%~%uint8 LEFT_FOOT_SWING  = 1 # Left foot constant~%uint8 RIGHT_FOOT_SWING = 2 # Right foot constant~%uint8 STANDING         = 3 # Standing constant~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Step2D)))
  "Returns full string definition for message of type 'Step2D"
  (cl:format cl:nil "#2D StepData~%~%geometry_msgs/Pose2D step2d   # step pose as relative offset to last leg~%~%~%# which leg to be used (left/right/no, see below)~%uint8 moving_foot   ~%~%uint8 LEFT_FOOT_SWING  = 1 # Left foot constant~%uint8 RIGHT_FOOT_SWING = 2 # Right foot constant~%uint8 STANDING         = 3 # Standing constant~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Step2D>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'step2d))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Step2D>))
  "Converts a ROS message object to a list"
  (cl:list 'Step2D
    (cl:cons ':step2d (step2d msg))
    (cl:cons ':moving_foot (moving_foot msg))
))
