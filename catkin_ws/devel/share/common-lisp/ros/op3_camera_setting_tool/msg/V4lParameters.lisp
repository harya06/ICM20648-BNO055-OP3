; Auto-generated. Do not edit!


(cl:in-package op3_camera_setting_tool-msg)


;//! \htmlinclude V4lParameters.msg.html

(cl:defclass <V4lParameters> (roslisp-msg-protocol:ros-message)
  ((video_parameter
    :reader video_parameter
    :initarg :video_parameter
    :type (cl:vector op3_camera_setting_tool-msg:V4lParameter)
   :initform (cl:make-array 0 :element-type 'op3_camera_setting_tool-msg:V4lParameter :initial-element (cl:make-instance 'op3_camera_setting_tool-msg:V4lParameter))))
)

(cl:defclass V4lParameters (<V4lParameters>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <V4lParameters>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'V4lParameters)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name op3_camera_setting_tool-msg:<V4lParameters> is deprecated: use op3_camera_setting_tool-msg:V4lParameters instead.")))

(cl:ensure-generic-function 'video_parameter-val :lambda-list '(m))
(cl:defmethod video_parameter-val ((m <V4lParameters>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_camera_setting_tool-msg:video_parameter-val is deprecated.  Use op3_camera_setting_tool-msg:video_parameter instead.")
  (video_parameter m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <V4lParameters>) ostream)
  "Serializes a message object of type '<V4lParameters>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'video_parameter))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'video_parameter))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <V4lParameters>) istream)
  "Deserializes a message object of type '<V4lParameters>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'video_parameter) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'video_parameter)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'op3_camera_setting_tool-msg:V4lParameter))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<V4lParameters>)))
  "Returns string type for a message object of type '<V4lParameters>"
  "op3_camera_setting_tool/V4lParameters")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'V4lParameters)))
  "Returns string type for a message object of type 'V4lParameters"
  "op3_camera_setting_tool/V4lParameters")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<V4lParameters>)))
  "Returns md5sum for a message object of type '<V4lParameters>"
  "becf7b1c441fba0b43309667fa8477b4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'V4lParameters)))
  "Returns md5sum for a message object of type 'V4lParameters"
  "becf7b1c441fba0b43309667fa8477b4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<V4lParameters>)))
  "Returns full string definition for message of type '<V4lParameters>"
  (cl:format cl:nil "V4lParameter[]    video_parameter~%================================================================================~%MSG: op3_camera_setting_tool/V4lParameter~%string  name~%int32   value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'V4lParameters)))
  "Returns full string definition for message of type 'V4lParameters"
  (cl:format cl:nil "V4lParameter[]    video_parameter~%================================================================================~%MSG: op3_camera_setting_tool/V4lParameter~%string  name~%int32   value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <V4lParameters>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'video_parameter) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <V4lParameters>))
  "Converts a ROS message object to a list"
  (cl:list 'V4lParameters
    (cl:cons ':video_parameter (video_parameter msg))
))
