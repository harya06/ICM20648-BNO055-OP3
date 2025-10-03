; Auto-generated. Do not edit!


(cl:in-package op3_walking_module_msgs-srv)


;//! \htmlinclude SetWalkingParam-request.msg.html

(cl:defclass <SetWalkingParam-request> (roslisp-msg-protocol:ros-message)
  ((parameters
    :reader parameters
    :initarg :parameters
    :type op3_walking_module_msgs-msg:WalkingParam
    :initform (cl:make-instance 'op3_walking_module_msgs-msg:WalkingParam)))
)

(cl:defclass SetWalkingParam-request (<SetWalkingParam-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetWalkingParam-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetWalkingParam-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name op3_walking_module_msgs-srv:<SetWalkingParam-request> is deprecated: use op3_walking_module_msgs-srv:SetWalkingParam-request instead.")))

(cl:ensure-generic-function 'parameters-val :lambda-list '(m))
(cl:defmethod parameters-val ((m <SetWalkingParam-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_walking_module_msgs-srv:parameters-val is deprecated.  Use op3_walking_module_msgs-srv:parameters instead.")
  (parameters m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetWalkingParam-request>) ostream)
  "Serializes a message object of type '<SetWalkingParam-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'parameters) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetWalkingParam-request>) istream)
  "Deserializes a message object of type '<SetWalkingParam-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'parameters) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetWalkingParam-request>)))
  "Returns string type for a service object of type '<SetWalkingParam-request>"
  "op3_walking_module_msgs/SetWalkingParamRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetWalkingParam-request)))
  "Returns string type for a service object of type 'SetWalkingParam-request"
  "op3_walking_module_msgs/SetWalkingParamRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetWalkingParam-request>)))
  "Returns md5sum for a message object of type '<SetWalkingParam-request>"
  "5bdce659174df89e66795c32f4aa3cfb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetWalkingParam-request)))
  "Returns md5sum for a message object of type 'SetWalkingParam-request"
  "5bdce659174df89e66795c32f4aa3cfb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetWalkingParam-request>)))
  "Returns full string definition for message of type '<SetWalkingParam-request>"
  (cl:format cl:nil "WalkingParam    parameters~%~%================================================================================~%MSG: op3_walking_module_msgs/WalkingParam~%####### walking init pose #######~%float32 init_x_offset~%float32 init_y_offset~%float32 init_z_offset~%float32 init_roll_offset~%float32 init_pitch_offset~%float32 init_yaw_offset~%~%####### time parameter #####~%float32 period_time~%float32 dsp_ratio~%float32 step_fb_ratio~%~%########## walking parameter ########~%float32 x_move_amplitude~%float32 y_move_amplitude~%float32 z_move_amplitude~%float32 angle_move_amplitude~%bool move_aim_on~%~%########## balance parameter ##########~%bool balance_enable~%float32 balance_hip_roll_gain~%float32 balance_knee_gain~%float32 balance_ankle_roll_gain~%float32 balance_ankle_pitch_gain~%float32 y_swap_amplitude~%float32 z_swap_amplitude~%float32 arm_swing_gain~%float32 pelvis_offset~%float32 hip_pitch_offset~%~%########## gain parameter ##########~%int32 p_gain~%int32 i_gain~%int32 d_gain~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetWalkingParam-request)))
  "Returns full string definition for message of type 'SetWalkingParam-request"
  (cl:format cl:nil "WalkingParam    parameters~%~%================================================================================~%MSG: op3_walking_module_msgs/WalkingParam~%####### walking init pose #######~%float32 init_x_offset~%float32 init_y_offset~%float32 init_z_offset~%float32 init_roll_offset~%float32 init_pitch_offset~%float32 init_yaw_offset~%~%####### time parameter #####~%float32 period_time~%float32 dsp_ratio~%float32 step_fb_ratio~%~%########## walking parameter ########~%float32 x_move_amplitude~%float32 y_move_amplitude~%float32 z_move_amplitude~%float32 angle_move_amplitude~%bool move_aim_on~%~%########## balance parameter ##########~%bool balance_enable~%float32 balance_hip_roll_gain~%float32 balance_knee_gain~%float32 balance_ankle_roll_gain~%float32 balance_ankle_pitch_gain~%float32 y_swap_amplitude~%float32 z_swap_amplitude~%float32 arm_swing_gain~%float32 pelvis_offset~%float32 hip_pitch_offset~%~%########## gain parameter ##########~%int32 p_gain~%int32 i_gain~%int32 d_gain~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetWalkingParam-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'parameters))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetWalkingParam-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetWalkingParam-request
    (cl:cons ':parameters (parameters msg))
))
;//! \htmlinclude SetWalkingParam-response.msg.html

(cl:defclass <SetWalkingParam-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetWalkingParam-response (<SetWalkingParam-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetWalkingParam-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetWalkingParam-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name op3_walking_module_msgs-srv:<SetWalkingParam-response> is deprecated: use op3_walking_module_msgs-srv:SetWalkingParam-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <SetWalkingParam-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_walking_module_msgs-srv:result-val is deprecated.  Use op3_walking_module_msgs-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetWalkingParam-response>) ostream)
  "Serializes a message object of type '<SetWalkingParam-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'result) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetWalkingParam-response>) istream)
  "Deserializes a message object of type '<SetWalkingParam-response>"
    (cl:setf (cl:slot-value msg 'result) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetWalkingParam-response>)))
  "Returns string type for a service object of type '<SetWalkingParam-response>"
  "op3_walking_module_msgs/SetWalkingParamResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetWalkingParam-response)))
  "Returns string type for a service object of type 'SetWalkingParam-response"
  "op3_walking_module_msgs/SetWalkingParamResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetWalkingParam-response>)))
  "Returns md5sum for a message object of type '<SetWalkingParam-response>"
  "5bdce659174df89e66795c32f4aa3cfb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetWalkingParam-response)))
  "Returns md5sum for a message object of type 'SetWalkingParam-response"
  "5bdce659174df89e66795c32f4aa3cfb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetWalkingParam-response>)))
  "Returns full string definition for message of type '<SetWalkingParam-response>"
  (cl:format cl:nil "bool            result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetWalkingParam-response)))
  "Returns full string definition for message of type 'SetWalkingParam-response"
  (cl:format cl:nil "bool            result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetWalkingParam-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetWalkingParam-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetWalkingParam-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetWalkingParam)))
  'SetWalkingParam-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetWalkingParam)))
  'SetWalkingParam-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetWalkingParam)))
  "Returns string type for a service object of type '<SetWalkingParam>"
  "op3_walking_module_msgs/SetWalkingParam")