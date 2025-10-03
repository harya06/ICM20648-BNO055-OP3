;; Auto-generated. Do not edit!


(when (boundp 'op3_walking_module_msgs::SetWalkingParam)
  (if (not (find-package "OP3_WALKING_MODULE_MSGS"))
    (make-package "OP3_WALKING_MODULE_MSGS"))
  (shadow 'SetWalkingParam (find-package "OP3_WALKING_MODULE_MSGS")))
(unless (find-package "OP3_WALKING_MODULE_MSGS::SETWALKINGPARAM")
  (make-package "OP3_WALKING_MODULE_MSGS::SETWALKINGPARAM"))
(unless (find-package "OP3_WALKING_MODULE_MSGS::SETWALKINGPARAMREQUEST")
  (make-package "OP3_WALKING_MODULE_MSGS::SETWALKINGPARAMREQUEST"))
(unless (find-package "OP3_WALKING_MODULE_MSGS::SETWALKINGPARAMRESPONSE")
  (make-package "OP3_WALKING_MODULE_MSGS::SETWALKINGPARAMRESPONSE"))

(in-package "ROS")





(defclass op3_walking_module_msgs::SetWalkingParamRequest
  :super ros::object
  :slots (_parameters ))

(defmethod op3_walking_module_msgs::SetWalkingParamRequest
  (:init
   (&key
    ((:parameters __parameters) (instance op3_walking_module_msgs::WalkingParam :init))
    )
   (send-super :init)
   (setq _parameters __parameters)
   self)
  (:parameters
   (&rest __parameters)
   (if (keywordp (car __parameters))
       (send* _parameters __parameters)
     (progn
       (if __parameters (setq _parameters (car __parameters)))
       _parameters)))
  (:serialization-length
   ()
   (+
    ;; op3_walking_module_msgs/WalkingParam _parameters
    (send _parameters :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; op3_walking_module_msgs/WalkingParam _parameters
       (send _parameters :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; op3_walking_module_msgs/WalkingParam _parameters
     (send _parameters :deserialize buf ptr-) (incf ptr- (send _parameters :serialization-length))
   ;;
   self)
  )

(defclass op3_walking_module_msgs::SetWalkingParamResponse
  :super ros::object
  :slots (_result ))

(defmethod op3_walking_module_msgs::SetWalkingParamResponse
  (:init
   (&key
    ((:result __result) nil)
    )
   (send-super :init)
   (setq _result __result)
   self)
  (:result
   (&optional __result)
   (if __result (setq _result __result)) _result)
  (:serialization-length
   ()
   (+
    ;; bool _result
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _result
       (if _result (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _result
     (setq _result (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass op3_walking_module_msgs::SetWalkingParam
  :super ros::object
  :slots ())

(setf (get op3_walking_module_msgs::SetWalkingParam :md5sum-) "5bdce659174df89e66795c32f4aa3cfb")
(setf (get op3_walking_module_msgs::SetWalkingParam :datatype-) "op3_walking_module_msgs/SetWalkingParam")
(setf (get op3_walking_module_msgs::SetWalkingParam :request) op3_walking_module_msgs::SetWalkingParamRequest)
(setf (get op3_walking_module_msgs::SetWalkingParam :response) op3_walking_module_msgs::SetWalkingParamResponse)

(defmethod op3_walking_module_msgs::SetWalkingParamRequest
  (:response () (instance op3_walking_module_msgs::SetWalkingParamResponse :init)))

(setf (get op3_walking_module_msgs::SetWalkingParamRequest :md5sum-) "5bdce659174df89e66795c32f4aa3cfb")
(setf (get op3_walking_module_msgs::SetWalkingParamRequest :datatype-) "op3_walking_module_msgs/SetWalkingParamRequest")
(setf (get op3_walking_module_msgs::SetWalkingParamRequest :definition-)
      "WalkingParam    parameters

================================================================================
MSG: op3_walking_module_msgs/WalkingParam
####### walking init pose #######
float32 init_x_offset
float32 init_y_offset
float32 init_z_offset
float32 init_roll_offset
float32 init_pitch_offset
float32 init_yaw_offset

####### time parameter #####
float32 period_time
float32 dsp_ratio
float32 step_fb_ratio

########## walking parameter ########
float32 x_move_amplitude
float32 y_move_amplitude
float32 z_move_amplitude
float32 angle_move_amplitude
bool move_aim_on

########## balance parameter ##########
bool balance_enable
float32 balance_hip_roll_gain
float32 balance_knee_gain
float32 balance_ankle_roll_gain
float32 balance_ankle_pitch_gain
float32 y_swap_amplitude
float32 z_swap_amplitude
float32 arm_swing_gain
float32 pelvis_offset
float32 hip_pitch_offset

########## gain parameter ##########
int32 p_gain
int32 i_gain
---
bool            result
")

(setf (get op3_walking_module_msgs::SetWalkingParamResponse :md5sum-) "5bdce659174df89e66795c32f4aa3cfb")
(setf (get op3_walking_module_msgs::SetWalkingParamResponse :datatype-) "op3_walking_module_msgs/SetWalkingParamResponse")
(setf (get op3_walking_module_msgs::SetWalkingParamResponse :definition-)
      "WalkingParam    parameters

================================================================================
MSG: op3_walking_module_msgs/WalkingParam
####### walking init pose #######
float32 init_x_offset
float32 init_y_offset
float32 init_z_offset
float32 init_roll_offset
float32 init_pitch_offset
float32 init_yaw_offset

####### time parameter #####
float32 period_time
float32 dsp_ratio
float32 step_fb_ratio

########## walking parameter ########
float32 x_move_amplitude
float32 y_move_amplitude
float32 z_move_amplitude
float32 angle_move_amplitude
bool move_aim_on

########## balance parameter ##########
bool balance_enable
float32 balance_hip_roll_gain
float32 balance_knee_gain
float32 balance_ankle_roll_gain
float32 balance_ankle_pitch_gain
float32 y_swap_amplitude
float32 z_swap_amplitude
float32 arm_swing_gain
float32 pelvis_offset
float32 hip_pitch_offset

########## gain parameter ##########
int32 p_gain
int32 i_gain
---
bool            result
")



(provide :op3_walking_module_msgs/SetWalkingParam "5bdce659174df89e66795c32f4aa3cfb")


