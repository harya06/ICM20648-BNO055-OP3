;; Auto-generated. Do not edit!


(when (boundp 'op3_camera_setting_tool::SetParameters)
  (if (not (find-package "OP3_CAMERA_SETTING_TOOL"))
    (make-package "OP3_CAMERA_SETTING_TOOL"))
  (shadow 'SetParameters (find-package "OP3_CAMERA_SETTING_TOOL")))
(unless (find-package "OP3_CAMERA_SETTING_TOOL::SETPARAMETERS")
  (make-package "OP3_CAMERA_SETTING_TOOL::SETPARAMETERS"))
(unless (find-package "OP3_CAMERA_SETTING_TOOL::SETPARAMETERSREQUEST")
  (make-package "OP3_CAMERA_SETTING_TOOL::SETPARAMETERSREQUEST"))
(unless (find-package "OP3_CAMERA_SETTING_TOOL::SETPARAMETERSRESPONSE")
  (make-package "OP3_CAMERA_SETTING_TOOL::SETPARAMETERSRESPONSE"))

(in-package "ROS")





(defclass op3_camera_setting_tool::SetParametersRequest
  :super ros::object
  :slots (_params ))

(defmethod op3_camera_setting_tool::SetParametersRequest
  (:init
   (&key
    ((:params __params) (instance op3_camera_setting_tool::CameraParams :init))
    )
   (send-super :init)
   (setq _params __params)
   self)
  (:params
   (&rest __params)
   (if (keywordp (car __params))
       (send* _params __params)
     (progn
       (if __params (setq _params (car __params)))
       _params)))
  (:serialization-length
   ()
   (+
    ;; op3_camera_setting_tool/CameraParams _params
    (send _params :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; op3_camera_setting_tool/CameraParams _params
       (send _params :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; op3_camera_setting_tool/CameraParams _params
     (send _params :deserialize buf ptr-) (incf ptr- (send _params :serialization-length))
   ;;
   self)
  )

(defclass op3_camera_setting_tool::SetParametersResponse
  :super ros::object
  :slots (_returns ))

(defmethod op3_camera_setting_tool::SetParametersResponse
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

(defclass op3_camera_setting_tool::SetParameters
  :super ros::object
  :slots ())

(setf (get op3_camera_setting_tool::SetParameters :md5sum-) "f3fa160e05eef0ff6c5a3f1ecaa69cbc")
(setf (get op3_camera_setting_tool::SetParameters :datatype-) "op3_camera_setting_tool/SetParameters")
(setf (get op3_camera_setting_tool::SetParameters :request) op3_camera_setting_tool::SetParametersRequest)
(setf (get op3_camera_setting_tool::SetParameters :response) op3_camera_setting_tool::SetParametersResponse)

(defmethod op3_camera_setting_tool::SetParametersRequest
  (:response () (instance op3_camera_setting_tool::SetParametersResponse :init)))

(setf (get op3_camera_setting_tool::SetParametersRequest :md5sum-) "f3fa160e05eef0ff6c5a3f1ecaa69cbc")
(setf (get op3_camera_setting_tool::SetParametersRequest :datatype-) "op3_camera_setting_tool/SetParametersRequest")
(setf (get op3_camera_setting_tool::SetParametersRequest :definition-)
      "CameraParams  params

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

(setf (get op3_camera_setting_tool::SetParametersResponse :md5sum-) "f3fa160e05eef0ff6c5a3f1ecaa69cbc")
(setf (get op3_camera_setting_tool::SetParametersResponse :datatype-) "op3_camera_setting_tool/SetParametersResponse")
(setf (get op3_camera_setting_tool::SetParametersResponse :definition-)
      "CameraParams  params

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



(provide :op3_camera_setting_tool/SetParameters "f3fa160e05eef0ff6c5a3f1ecaa69cbc")


