; Auto-generated. Do not edit!


(cl:in-package op3_camera_setting_tool-msg)


;//! \htmlinclude CameraParams.msg.html

(cl:defclass <CameraParams> (roslisp-msg-protocol:ros-message)
  ((brightness
    :reader brightness
    :initarg :brightness
    :type cl:integer
    :initform 0)
   (contrast
    :reader contrast
    :initarg :contrast
    :type cl:integer
    :initform 0)
   (saturation
    :reader saturation
    :initarg :saturation
    :type cl:integer
    :initform 0)
   (sharpness
    :reader sharpness
    :initarg :sharpness
    :type cl:integer
    :initform 0)
   (gain
    :reader gain
    :initarg :gain
    :type cl:integer
    :initform 0)
   (focus_auto
    :reader focus_auto
    :initarg :focus_auto
    :type cl:boolean
    :initform cl:nil)
   (focus_absolute
    :reader focus_absolute
    :initarg :focus_absolute
    :type cl:integer
    :initform 0)
   (exposure_auto
    :reader exposure_auto
    :initarg :exposure_auto
    :type cl:integer
    :initform 0)
   (exposure_absolute
    :reader exposure_absolute
    :initarg :exposure_absolute
    :type cl:integer
    :initform 0)
   (white_balance_temperature_auto
    :reader white_balance_temperature_auto
    :initarg :white_balance_temperature_auto
    :type cl:boolean
    :initform cl:nil)
   (white_balance_temperature
    :reader white_balance_temperature
    :initarg :white_balance_temperature
    :type cl:integer
    :initform 0))
)

(cl:defclass CameraParams (<CameraParams>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CameraParams>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CameraParams)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name op3_camera_setting_tool-msg:<CameraParams> is deprecated: use op3_camera_setting_tool-msg:CameraParams instead.")))

(cl:ensure-generic-function 'brightness-val :lambda-list '(m))
(cl:defmethod brightness-val ((m <CameraParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_camera_setting_tool-msg:brightness-val is deprecated.  Use op3_camera_setting_tool-msg:brightness instead.")
  (brightness m))

(cl:ensure-generic-function 'contrast-val :lambda-list '(m))
(cl:defmethod contrast-val ((m <CameraParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_camera_setting_tool-msg:contrast-val is deprecated.  Use op3_camera_setting_tool-msg:contrast instead.")
  (contrast m))

(cl:ensure-generic-function 'saturation-val :lambda-list '(m))
(cl:defmethod saturation-val ((m <CameraParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_camera_setting_tool-msg:saturation-val is deprecated.  Use op3_camera_setting_tool-msg:saturation instead.")
  (saturation m))

(cl:ensure-generic-function 'sharpness-val :lambda-list '(m))
(cl:defmethod sharpness-val ((m <CameraParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_camera_setting_tool-msg:sharpness-val is deprecated.  Use op3_camera_setting_tool-msg:sharpness instead.")
  (sharpness m))

(cl:ensure-generic-function 'gain-val :lambda-list '(m))
(cl:defmethod gain-val ((m <CameraParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_camera_setting_tool-msg:gain-val is deprecated.  Use op3_camera_setting_tool-msg:gain instead.")
  (gain m))

(cl:ensure-generic-function 'focus_auto-val :lambda-list '(m))
(cl:defmethod focus_auto-val ((m <CameraParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_camera_setting_tool-msg:focus_auto-val is deprecated.  Use op3_camera_setting_tool-msg:focus_auto instead.")
  (focus_auto m))

(cl:ensure-generic-function 'focus_absolute-val :lambda-list '(m))
(cl:defmethod focus_absolute-val ((m <CameraParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_camera_setting_tool-msg:focus_absolute-val is deprecated.  Use op3_camera_setting_tool-msg:focus_absolute instead.")
  (focus_absolute m))

(cl:ensure-generic-function 'exposure_auto-val :lambda-list '(m))
(cl:defmethod exposure_auto-val ((m <CameraParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_camera_setting_tool-msg:exposure_auto-val is deprecated.  Use op3_camera_setting_tool-msg:exposure_auto instead.")
  (exposure_auto m))

(cl:ensure-generic-function 'exposure_absolute-val :lambda-list '(m))
(cl:defmethod exposure_absolute-val ((m <CameraParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_camera_setting_tool-msg:exposure_absolute-val is deprecated.  Use op3_camera_setting_tool-msg:exposure_absolute instead.")
  (exposure_absolute m))

(cl:ensure-generic-function 'white_balance_temperature_auto-val :lambda-list '(m))
(cl:defmethod white_balance_temperature_auto-val ((m <CameraParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_camera_setting_tool-msg:white_balance_temperature_auto-val is deprecated.  Use op3_camera_setting_tool-msg:white_balance_temperature_auto instead.")
  (white_balance_temperature_auto m))

(cl:ensure-generic-function 'white_balance_temperature-val :lambda-list '(m))
(cl:defmethod white_balance_temperature-val ((m <CameraParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_camera_setting_tool-msg:white_balance_temperature-val is deprecated.  Use op3_camera_setting_tool-msg:white_balance_temperature instead.")
  (white_balance_temperature m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CameraParams>) ostream)
  "Serializes a message object of type '<CameraParams>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'brightness)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'brightness)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'brightness)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'brightness)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'contrast)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'contrast)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'contrast)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'contrast)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'saturation)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'saturation)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'saturation)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'saturation)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sharpness)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'sharpness)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'sharpness)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'sharpness)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gain)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'gain)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'gain)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'gain)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'focus_auto) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'focus_absolute)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'exposure_auto)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'exposure_auto)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'exposure_auto)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'exposure_auto)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'exposure_absolute)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'exposure_absolute)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'exposure_absolute)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'exposure_absolute)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'white_balance_temperature_auto) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'white_balance_temperature)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'white_balance_temperature)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'white_balance_temperature)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'white_balance_temperature)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CameraParams>) istream)
  "Deserializes a message object of type '<CameraParams>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'brightness)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'brightness)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'brightness)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'brightness)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'contrast)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'contrast)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'contrast)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'contrast)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'saturation)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'saturation)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'saturation)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'saturation)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sharpness)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'sharpness)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'sharpness)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'sharpness)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gain)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'gain)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'gain)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'gain)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'focus_auto) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'focus_absolute) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'exposure_auto)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'exposure_auto)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'exposure_auto)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'exposure_auto)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'exposure_absolute)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'exposure_absolute)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'exposure_absolute)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'exposure_absolute)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'white_balance_temperature_auto) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'white_balance_temperature)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'white_balance_temperature)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'white_balance_temperature)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'white_balance_temperature)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CameraParams>)))
  "Returns string type for a message object of type '<CameraParams>"
  "op3_camera_setting_tool/CameraParams")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CameraParams)))
  "Returns string type for a message object of type 'CameraParams"
  "op3_camera_setting_tool/CameraParams")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CameraParams>)))
  "Returns md5sum for a message object of type '<CameraParams>"
  "e3e69e8ca87d52d03119a1d00655d838")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CameraParams)))
  "Returns md5sum for a message object of type 'CameraParams"
  "e3e69e8ca87d52d03119a1d00655d838")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CameraParams>)))
  "Returns full string definition for message of type '<CameraParams>"
  (cl:format cl:nil "# This represents the parameters of usb camera~%~%uint32  brightness 							# 0 - 255~%uint32  contrast                            # 0 - 255~%uint32  saturation                          # 0 - 255~%uint32  sharpness                           # 0 - 255~%uint32  gain                                # 0 - 255~%bool    focus_auto                          #~%int32  focus_absolute                       # -1 - 255~%uint32  exposure_auto                       #~%uint32  exposure_absolute                   # 0 - 255~%bool    white_balance_temperature_auto      #~%uint32  white_balance_temperature           # 4000 - 6500~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CameraParams)))
  "Returns full string definition for message of type 'CameraParams"
  (cl:format cl:nil "# This represents the parameters of usb camera~%~%uint32  brightness 							# 0 - 255~%uint32  contrast                            # 0 - 255~%uint32  saturation                          # 0 - 255~%uint32  sharpness                           # 0 - 255~%uint32  gain                                # 0 - 255~%bool    focus_auto                          #~%int32  focus_absolute                       # -1 - 255~%uint32  exposure_auto                       #~%uint32  exposure_absolute                   # 0 - 255~%bool    white_balance_temperature_auto      #~%uint32  white_balance_temperature           # 4000 - 6500~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CameraParams>))
  (cl:+ 0
     4
     4
     4
     4
     4
     1
     4
     4
     4
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CameraParams>))
  "Converts a ROS message object to a list"
  (cl:list 'CameraParams
    (cl:cons ':brightness (brightness msg))
    (cl:cons ':contrast (contrast msg))
    (cl:cons ':saturation (saturation msg))
    (cl:cons ':sharpness (sharpness msg))
    (cl:cons ':gain (gain msg))
    (cl:cons ':focus_auto (focus_auto msg))
    (cl:cons ':focus_absolute (focus_absolute msg))
    (cl:cons ':exposure_auto (exposure_auto msg))
    (cl:cons ':exposure_absolute (exposure_absolute msg))
    (cl:cons ':white_balance_temperature_auto (white_balance_temperature_auto msg))
    (cl:cons ':white_balance_temperature (white_balance_temperature msg))
))
