; Auto-generated. Do not edit!


(cl:in-package op3_camera_setting_tool-srv)


;//! \htmlinclude SetParameters-request.msg.html

(cl:defclass <SetParameters-request> (roslisp-msg-protocol:ros-message)
  ((params
    :reader params
    :initarg :params
    :type op3_camera_setting_tool-msg:CameraParams
    :initform (cl:make-instance 'op3_camera_setting_tool-msg:CameraParams)))
)

(cl:defclass SetParameters-request (<SetParameters-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetParameters-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetParameters-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name op3_camera_setting_tool-srv:<SetParameters-request> is deprecated: use op3_camera_setting_tool-srv:SetParameters-request instead.")))

(cl:ensure-generic-function 'params-val :lambda-list '(m))
(cl:defmethod params-val ((m <SetParameters-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_camera_setting_tool-srv:params-val is deprecated.  Use op3_camera_setting_tool-srv:params instead.")
  (params m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetParameters-request>) ostream)
  "Serializes a message object of type '<SetParameters-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'params) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetParameters-request>) istream)
  "Deserializes a message object of type '<SetParameters-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'params) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetParameters-request>)))
  "Returns string type for a service object of type '<SetParameters-request>"
  "op3_camera_setting_tool/SetParametersRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetParameters-request)))
  "Returns string type for a service object of type 'SetParameters-request"
  "op3_camera_setting_tool/SetParametersRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetParameters-request>)))
  "Returns md5sum for a message object of type '<SetParameters-request>"
  "f3fa160e05eef0ff6c5a3f1ecaa69cbc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetParameters-request)))
  "Returns md5sum for a message object of type 'SetParameters-request"
  "f3fa160e05eef0ff6c5a3f1ecaa69cbc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetParameters-request>)))
  "Returns full string definition for message of type '<SetParameters-request>"
  (cl:format cl:nil "CameraParams  params~%~%================================================================================~%MSG: op3_camera_setting_tool/CameraParams~%# This represents the parameters of usb camera~%~%uint32  brightness 							# 0 - 255~%uint32  contrast                            # 0 - 255~%uint32  saturation                          # 0 - 255~%uint32  sharpness                           # 0 - 255~%uint32  gain                                # 0 - 255~%bool    focus_auto                          #~%int32  focus_absolute                       # -1 - 255~%uint32  exposure_auto                       #~%uint32  exposure_absolute                   # 0 - 255~%bool    white_balance_temperature_auto      #~%uint32  white_balance_temperature           # 4000 - 6500~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetParameters-request)))
  "Returns full string definition for message of type 'SetParameters-request"
  (cl:format cl:nil "CameraParams  params~%~%================================================================================~%MSG: op3_camera_setting_tool/CameraParams~%# This represents the parameters of usb camera~%~%uint32  brightness 							# 0 - 255~%uint32  contrast                            # 0 - 255~%uint32  saturation                          # 0 - 255~%uint32  sharpness                           # 0 - 255~%uint32  gain                                # 0 - 255~%bool    focus_auto                          #~%int32  focus_absolute                       # -1 - 255~%uint32  exposure_auto                       #~%uint32  exposure_absolute                   # 0 - 255~%bool    white_balance_temperature_auto      #~%uint32  white_balance_temperature           # 4000 - 6500~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetParameters-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'params))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetParameters-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetParameters-request
    (cl:cons ':params (params msg))
))
;//! \htmlinclude SetParameters-response.msg.html

(cl:defclass <SetParameters-response> (roslisp-msg-protocol:ros-message)
  ((returns
    :reader returns
    :initarg :returns
    :type op3_camera_setting_tool-msg:CameraParams
    :initform (cl:make-instance 'op3_camera_setting_tool-msg:CameraParams)))
)

(cl:defclass SetParameters-response (<SetParameters-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetParameters-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetParameters-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name op3_camera_setting_tool-srv:<SetParameters-response> is deprecated: use op3_camera_setting_tool-srv:SetParameters-response instead.")))

(cl:ensure-generic-function 'returns-val :lambda-list '(m))
(cl:defmethod returns-val ((m <SetParameters-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_camera_setting_tool-srv:returns-val is deprecated.  Use op3_camera_setting_tool-srv:returns instead.")
  (returns m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetParameters-response>) ostream)
  "Serializes a message object of type '<SetParameters-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'returns) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetParameters-response>) istream)
  "Deserializes a message object of type '<SetParameters-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'returns) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetParameters-response>)))
  "Returns string type for a service object of type '<SetParameters-response>"
  "op3_camera_setting_tool/SetParametersResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetParameters-response)))
  "Returns string type for a service object of type 'SetParameters-response"
  "op3_camera_setting_tool/SetParametersResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetParameters-response>)))
  "Returns md5sum for a message object of type '<SetParameters-response>"
  "f3fa160e05eef0ff6c5a3f1ecaa69cbc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetParameters-response)))
  "Returns md5sum for a message object of type 'SetParameters-response"
  "f3fa160e05eef0ff6c5a3f1ecaa69cbc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetParameters-response>)))
  "Returns full string definition for message of type '<SetParameters-response>"
  (cl:format cl:nil "CameraParams  returns~%~%~%================================================================================~%MSG: op3_camera_setting_tool/CameraParams~%# This represents the parameters of usb camera~%~%uint32  brightness 							# 0 - 255~%uint32  contrast                            # 0 - 255~%uint32  saturation                          # 0 - 255~%uint32  sharpness                           # 0 - 255~%uint32  gain                                # 0 - 255~%bool    focus_auto                          #~%int32  focus_absolute                       # -1 - 255~%uint32  exposure_auto                       #~%uint32  exposure_absolute                   # 0 - 255~%bool    white_balance_temperature_auto      #~%uint32  white_balance_temperature           # 4000 - 6500~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetParameters-response)))
  "Returns full string definition for message of type 'SetParameters-response"
  (cl:format cl:nil "CameraParams  returns~%~%~%================================================================================~%MSG: op3_camera_setting_tool/CameraParams~%# This represents the parameters of usb camera~%~%uint32  brightness 							# 0 - 255~%uint32  contrast                            # 0 - 255~%uint32  saturation                          # 0 - 255~%uint32  sharpness                           # 0 - 255~%uint32  gain                                # 0 - 255~%bool    focus_auto                          #~%int32  focus_absolute                       # -1 - 255~%uint32  exposure_auto                       #~%uint32  exposure_absolute                   # 0 - 255~%bool    white_balance_temperature_auto      #~%uint32  white_balance_temperature           # 4000 - 6500~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetParameters-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'returns))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetParameters-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetParameters-response
    (cl:cons ':returns (returns msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetParameters)))
  'SetParameters-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetParameters)))
  'SetParameters-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetParameters)))
  "Returns string type for a service object of type '<SetParameters>"
  "op3_camera_setting_tool/SetParameters")