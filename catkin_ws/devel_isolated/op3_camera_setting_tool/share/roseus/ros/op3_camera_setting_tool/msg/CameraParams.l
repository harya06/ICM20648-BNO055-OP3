;; Auto-generated. Do not edit!


(when (boundp 'op3_camera_setting_tool::CameraParams)
  (if (not (find-package "OP3_CAMERA_SETTING_TOOL"))
    (make-package "OP3_CAMERA_SETTING_TOOL"))
  (shadow 'CameraParams (find-package "OP3_CAMERA_SETTING_TOOL")))
(unless (find-package "OP3_CAMERA_SETTING_TOOL::CAMERAPARAMS")
  (make-package "OP3_CAMERA_SETTING_TOOL::CAMERAPARAMS"))

(in-package "ROS")
;;//! \htmlinclude CameraParams.msg.html


(defclass op3_camera_setting_tool::CameraParams
  :super ros::object
  :slots (_brightness _contrast _saturation _sharpness _gain _focus_auto _focus_absolute _exposure_auto _exposure_absolute _white_balance_temperature_auto _white_balance_temperature ))

(defmethod op3_camera_setting_tool::CameraParams
  (:init
   (&key
    ((:brightness __brightness) 0)
    ((:contrast __contrast) 0)
    ((:saturation __saturation) 0)
    ((:sharpness __sharpness) 0)
    ((:gain __gain) 0)
    ((:focus_auto __focus_auto) nil)
    ((:focus_absolute __focus_absolute) 0)
    ((:exposure_auto __exposure_auto) 0)
    ((:exposure_absolute __exposure_absolute) 0)
    ((:white_balance_temperature_auto __white_balance_temperature_auto) nil)
    ((:white_balance_temperature __white_balance_temperature) 0)
    )
   (send-super :init)
   (setq _brightness (round __brightness))
   (setq _contrast (round __contrast))
   (setq _saturation (round __saturation))
   (setq _sharpness (round __sharpness))
   (setq _gain (round __gain))
   (setq _focus_auto __focus_auto)
   (setq _focus_absolute (round __focus_absolute))
   (setq _exposure_auto (round __exposure_auto))
   (setq _exposure_absolute (round __exposure_absolute))
   (setq _white_balance_temperature_auto __white_balance_temperature_auto)
   (setq _white_balance_temperature (round __white_balance_temperature))
   self)
  (:brightness
   (&optional __brightness)
   (if __brightness (setq _brightness __brightness)) _brightness)
  (:contrast
   (&optional __contrast)
   (if __contrast (setq _contrast __contrast)) _contrast)
  (:saturation
   (&optional __saturation)
   (if __saturation (setq _saturation __saturation)) _saturation)
  (:sharpness
   (&optional __sharpness)
   (if __sharpness (setq _sharpness __sharpness)) _sharpness)
  (:gain
   (&optional __gain)
   (if __gain (setq _gain __gain)) _gain)
  (:focus_auto
   (&optional __focus_auto)
   (if __focus_auto (setq _focus_auto __focus_auto)) _focus_auto)
  (:focus_absolute
   (&optional __focus_absolute)
   (if __focus_absolute (setq _focus_absolute __focus_absolute)) _focus_absolute)
  (:exposure_auto
   (&optional __exposure_auto)
   (if __exposure_auto (setq _exposure_auto __exposure_auto)) _exposure_auto)
  (:exposure_absolute
   (&optional __exposure_absolute)
   (if __exposure_absolute (setq _exposure_absolute __exposure_absolute)) _exposure_absolute)
  (:white_balance_temperature_auto
   (&optional __white_balance_temperature_auto)
   (if __white_balance_temperature_auto (setq _white_balance_temperature_auto __white_balance_temperature_auto)) _white_balance_temperature_auto)
  (:white_balance_temperature
   (&optional __white_balance_temperature)
   (if __white_balance_temperature (setq _white_balance_temperature __white_balance_temperature)) _white_balance_temperature)
  (:serialization-length
   ()
   (+
    ;; uint32 _brightness
    4
    ;; uint32 _contrast
    4
    ;; uint32 _saturation
    4
    ;; uint32 _sharpness
    4
    ;; uint32 _gain
    4
    ;; bool _focus_auto
    1
    ;; int32 _focus_absolute
    4
    ;; uint32 _exposure_auto
    4
    ;; uint32 _exposure_absolute
    4
    ;; bool _white_balance_temperature_auto
    1
    ;; uint32 _white_balance_temperature
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _brightness
       (write-long _brightness s)
     ;; uint32 _contrast
       (write-long _contrast s)
     ;; uint32 _saturation
       (write-long _saturation s)
     ;; uint32 _sharpness
       (write-long _sharpness s)
     ;; uint32 _gain
       (write-long _gain s)
     ;; bool _focus_auto
       (if _focus_auto (write-byte -1 s) (write-byte 0 s))
     ;; int32 _focus_absolute
       (write-long _focus_absolute s)
     ;; uint32 _exposure_auto
       (write-long _exposure_auto s)
     ;; uint32 _exposure_absolute
       (write-long _exposure_absolute s)
     ;; bool _white_balance_temperature_auto
       (if _white_balance_temperature_auto (write-byte -1 s) (write-byte 0 s))
     ;; uint32 _white_balance_temperature
       (write-long _white_balance_temperature s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _brightness
     (setq _brightness (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _contrast
     (setq _contrast (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _saturation
     (setq _saturation (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _sharpness
     (setq _sharpness (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _gain
     (setq _gain (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _focus_auto
     (setq _focus_auto (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int32 _focus_absolute
     (setq _focus_absolute (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _exposure_auto
     (setq _exposure_auto (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _exposure_absolute
     (setq _exposure_absolute (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _white_balance_temperature_auto
     (setq _white_balance_temperature_auto (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint32 _white_balance_temperature
     (setq _white_balance_temperature (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get op3_camera_setting_tool::CameraParams :md5sum-) "e3e69e8ca87d52d03119a1d00655d838")
(setf (get op3_camera_setting_tool::CameraParams :datatype-) "op3_camera_setting_tool/CameraParams")
(setf (get op3_camera_setting_tool::CameraParams :definition-)
      "# This represents the parameters of usb camera

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



(provide :op3_camera_setting_tool/CameraParams "e3e69e8ca87d52d03119a1d00655d838")


