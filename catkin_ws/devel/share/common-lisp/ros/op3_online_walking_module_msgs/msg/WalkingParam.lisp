; Auto-generated. Do not edit!


(cl:in-package op3_online_walking_module_msgs-msg)


;//! \htmlinclude WalkingParam.msg.html

(cl:defclass <WalkingParam> (roslisp-msg-protocol:ros-message)
  ((dsp_ratio
    :reader dsp_ratio
    :initarg :dsp_ratio
    :type cl:float
    :initform 0.0)
   (lipm_height
    :reader lipm_height
    :initarg :lipm_height
    :type cl:float
    :initform 0.0)
   (foot_height_max
    :reader foot_height_max
    :initarg :foot_height_max
    :type cl:float
    :initform 0.0)
   (zmp_offset_x
    :reader zmp_offset_x
    :initarg :zmp_offset_x
    :type cl:float
    :initform 0.0)
   (zmp_offset_y
    :reader zmp_offset_y
    :initarg :zmp_offset_y
    :type cl:float
    :initform 0.0))
)

(cl:defclass WalkingParam (<WalkingParam>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WalkingParam>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WalkingParam)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name op3_online_walking_module_msgs-msg:<WalkingParam> is deprecated: use op3_online_walking_module_msgs-msg:WalkingParam instead.")))

(cl:ensure-generic-function 'dsp_ratio-val :lambda-list '(m))
(cl:defmethod dsp_ratio-val ((m <WalkingParam>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_online_walking_module_msgs-msg:dsp_ratio-val is deprecated.  Use op3_online_walking_module_msgs-msg:dsp_ratio instead.")
  (dsp_ratio m))

(cl:ensure-generic-function 'lipm_height-val :lambda-list '(m))
(cl:defmethod lipm_height-val ((m <WalkingParam>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_online_walking_module_msgs-msg:lipm_height-val is deprecated.  Use op3_online_walking_module_msgs-msg:lipm_height instead.")
  (lipm_height m))

(cl:ensure-generic-function 'foot_height_max-val :lambda-list '(m))
(cl:defmethod foot_height_max-val ((m <WalkingParam>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_online_walking_module_msgs-msg:foot_height_max-val is deprecated.  Use op3_online_walking_module_msgs-msg:foot_height_max instead.")
  (foot_height_max m))

(cl:ensure-generic-function 'zmp_offset_x-val :lambda-list '(m))
(cl:defmethod zmp_offset_x-val ((m <WalkingParam>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_online_walking_module_msgs-msg:zmp_offset_x-val is deprecated.  Use op3_online_walking_module_msgs-msg:zmp_offset_x instead.")
  (zmp_offset_x m))

(cl:ensure-generic-function 'zmp_offset_y-val :lambda-list '(m))
(cl:defmethod zmp_offset_y-val ((m <WalkingParam>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_online_walking_module_msgs-msg:zmp_offset_y-val is deprecated.  Use op3_online_walking_module_msgs-msg:zmp_offset_y instead.")
  (zmp_offset_y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WalkingParam>) ostream)
  "Serializes a message object of type '<WalkingParam>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'dsp_ratio))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'lipm_height))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'foot_height_max))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'zmp_offset_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'zmp_offset_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WalkingParam>) istream)
  "Deserializes a message object of type '<WalkingParam>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dsp_ratio) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'lipm_height) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'foot_height_max) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'zmp_offset_x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'zmp_offset_y) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WalkingParam>)))
  "Returns string type for a message object of type '<WalkingParam>"
  "op3_online_walking_module_msgs/WalkingParam")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WalkingParam)))
  "Returns string type for a message object of type 'WalkingParam"
  "op3_online_walking_module_msgs/WalkingParam")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WalkingParam>)))
  "Returns md5sum for a message object of type '<WalkingParam>"
  "2197a6ee210540728363ab6b547935bd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WalkingParam)))
  "Returns md5sum for a message object of type 'WalkingParam"
  "2197a6ee210540728363ab6b547935bd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WalkingParam>)))
  "Returns full string definition for message of type '<WalkingParam>"
  (cl:format cl:nil "float64 dsp_ratio~%float64 lipm_height~%float64 foot_height_max~%float64 zmp_offset_x~%float64 zmp_offset_y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WalkingParam)))
  "Returns full string definition for message of type 'WalkingParam"
  (cl:format cl:nil "float64 dsp_ratio~%float64 lipm_height~%float64 foot_height_max~%float64 zmp_offset_x~%float64 zmp_offset_y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WalkingParam>))
  (cl:+ 0
     8
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WalkingParam>))
  "Converts a ROS message object to a list"
  (cl:list 'WalkingParam
    (cl:cons ':dsp_ratio (dsp_ratio msg))
    (cl:cons ':lipm_height (lipm_height msg))
    (cl:cons ':foot_height_max (foot_height_max msg))
    (cl:cons ':zmp_offset_x (zmp_offset_x msg))
    (cl:cons ':zmp_offset_y (zmp_offset_y msg))
))
