; Auto-generated. Do not edit!


(cl:in-package op3_online_walking_module_msgs-msg)


;//! \htmlinclude PreviewRequest.msg.html

(cl:defclass <PreviewRequest> (roslisp-msg-protocol:ros-message)
  ((control_cycle
    :reader control_cycle
    :initarg :control_cycle
    :type cl:float
    :initform 0.0)
   (lipm_height
    :reader lipm_height
    :initarg :lipm_height
    :type cl:float
    :initform 0.0))
)

(cl:defclass PreviewRequest (<PreviewRequest>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PreviewRequest>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PreviewRequest)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name op3_online_walking_module_msgs-msg:<PreviewRequest> is deprecated: use op3_online_walking_module_msgs-msg:PreviewRequest instead.")))

(cl:ensure-generic-function 'control_cycle-val :lambda-list '(m))
(cl:defmethod control_cycle-val ((m <PreviewRequest>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_online_walking_module_msgs-msg:control_cycle-val is deprecated.  Use op3_online_walking_module_msgs-msg:control_cycle instead.")
  (control_cycle m))

(cl:ensure-generic-function 'lipm_height-val :lambda-list '(m))
(cl:defmethod lipm_height-val ((m <PreviewRequest>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_online_walking_module_msgs-msg:lipm_height-val is deprecated.  Use op3_online_walking_module_msgs-msg:lipm_height instead.")
  (lipm_height m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PreviewRequest>) ostream)
  "Serializes a message object of type '<PreviewRequest>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'control_cycle))))
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
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PreviewRequest>) istream)
  "Deserializes a message object of type '<PreviewRequest>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'control_cycle) (roslisp-utils:decode-double-float-bits bits)))
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PreviewRequest>)))
  "Returns string type for a message object of type '<PreviewRequest>"
  "op3_online_walking_module_msgs/PreviewRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PreviewRequest)))
  "Returns string type for a message object of type 'PreviewRequest"
  "op3_online_walking_module_msgs/PreviewRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PreviewRequest>)))
  "Returns md5sum for a message object of type '<PreviewRequest>"
  "5dd8c29c2c1ccae01b0503604675335f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PreviewRequest)))
  "Returns md5sum for a message object of type 'PreviewRequest"
  "5dd8c29c2c1ccae01b0503604675335f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PreviewRequest>)))
  "Returns full string definition for message of type '<PreviewRequest>"
  (cl:format cl:nil "float64 control_cycle~%float64 lipm_height~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PreviewRequest)))
  "Returns full string definition for message of type 'PreviewRequest"
  (cl:format cl:nil "float64 control_cycle~%float64 lipm_height~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PreviewRequest>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PreviewRequest>))
  "Converts a ROS message object to a list"
  (cl:list 'PreviewRequest
    (cl:cons ':control_cycle (control_cycle msg))
    (cl:cons ':lipm_height (lipm_height msg))
))
