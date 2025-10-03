;; Auto-generated. Do not edit!


(when (boundp 'op3_camera_setting_tool::GetParameters)
  (if (not (find-package "OP3_CAMERA_SETTING_TOOL"))
    (make-package "OP3_CAMERA_SETTING_TOOL"))
  (shadow 'GetParameters (find-package "OP3_CAMERA_SETTING_TOOL")))
(unless (find-package "OP3_CAMERA_SETTING_TOOL::GETPARAMETERS")
  (make-package "OP3_CAMERA_SETTING_TOOL::GETPARAMETERS"))
(unless (find-package "OP3_CAMERA_SETTING_TOOL::GETPARAMETERSREQUEST")
  (make-package "OP3_CAMERA_SETTING_TOOL::GETPARAMETERSREQUEST"))
(unless (find-package "OP3_CAMERA_SETTING_TOOL::GETPARAMETERSRESPONSE")
  (make-package "OP3_CAMERA_SETTING_TOOL::GETPARAMETERSRESPONSE"))

(in-package "ROS")





(defclass op3_camera_setting_tool::GetParametersRequest
  :super ros::object
  :slots ())

(defmethod op3_camera_setting_tool::GetParametersRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass op3_camera_setting_tool::GetParametersResponse
  :super ros::object
  :slots (_returns ))

(defmethod op3_camera_setting_tool::GetParametersResponse
  (:init
   (&key
    ((:returns __returns) (instance op3_camera_setting_tool::CameraParams :init))
    )
   (send-super :init)
   (setq _returns __returns)
   self)
  (:returns
   (&rest __returns)
   (if (keywordp (car __returns))
       (send* _returns __returns)
     (progn
       (if __returns (setq _returns (car __returns)))
       _returns)))
  (:serialization-length
   ()
   (+
    ;; op3_camera_setting_tool/CameraParams _returns
    (send _returns :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; op3_camera_setting_tool/CameraParams _returns
       (send _returns :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; op3_camera_setting_tool/CameraParams _returns
     (send _returns :deserialize buf ptr-) (incf ptr- (send _returns :serialization-length))
   ;;
   self)
  )

(defclass op3_camera_setting_tool::GetParameters
  :super ros::object
  :slots ())

(setf (get op3_camera_setting_tool::GetParameters :md5sum-) "ba9411ea8e0a1eee4b9a16def5c761b3")
(setf (get op3_camera_setting_tool::GetParameters :datatype-) "op3_camera_setting_tool/GetParameters")
(setf (get op3_camera_setting_tool::GetParameters :request) op3_camera_setting_tool::GetParametersRequest)
(setf (get op3_camera_setting_tool::GetParameters :response) op3_camera_setting_tool::GetParametersResponse)

(defmethod op3_camera_setting_tool::GetParametersRequest
  (:response () (instance op3_camera_setting_tool::GetParametersResponse :init)))

(setf (get op3_camera_setting_tool::GetParametersRequest :md5sum-) "ba9411ea8e0a1eee4b9a16def5c761b3")
(setf (get op3_camera_setting_tool::GetParametersRequest :datatype-) "op3_camera_setting_tool/GetParametersRequest")
(setf (get op3_camera_setting_tool::GetParametersRequest :definition-)
      "
---
CameraParams  returns


================================================================================
MSG: op3_camera_setting_tool/CameraParams
# This represents the parameters of usb camera

uint32  brightness 							# 0 - 255
uint32  contrast                            # 0 - 255
uint32  saturation                          # 0 - 255
uint32  sharpness                           # 0 - 255
uint32  gain                                # 0 - 255
bool    focus_auto                          #
int32  focus_absolute                       # -1 - 255
uint32  exposure_auto                       #
uint32  exposure_absolute                   # 0 - 255
bool    white_balance_temperature_auto      #
uint32  white_balance_temperature           # 4000 - 6500
")

(setf (get op3_camera_setting_tool::GetParametersResponse :md5sum-) "ba9411ea8e0a1eee4b9a16def5c761b3")
(setf (get op3_camera_setting_tool::GetParametersResponse :datatype-) "op3_camera_setting_tool/GetParametersResponse")
(setf (get op3_camera_setting_tool::GetParametersResponse :definition-)
      "
---
CameraParams  returns


================================================================================
MSG: op3_camera_setting_tool/CameraParams
# This represents the parameters of usb camera

uint32  brightness 							# 0 - 255
uint32  contrast                            # 0 - 255
uint32  saturation                          # 0 - 255
uint32  sharpness                           # 0 - 255
uint32  gain                                # 0 - 255
bool    focus_auto                          #
int32  focus_absolute                       # -1 - 255
uint32  exposure_auto                       #
uint32  exposure_absolute                   # 0 - 255
bool    white_balance_temperature_auto      #
uint32  white_balance_temperature           # 4000 - 6500
")



(provide :op3_camera_setting_tool/GetParameters "ba9411ea8e0a1eee4b9a16def5c761b3")


