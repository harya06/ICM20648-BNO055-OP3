;; Auto-generated. Do not edit!


(when (boundp 'op3_walking_module_msgs::WalkingParam)
  (if (not (find-package "OP3_WALKING_MODULE_MSGS"))
    (make-package "OP3_WALKING_MODULE_MSGS"))
  (shadow 'WalkingParam (find-package "OP3_WALKING_MODULE_MSGS")))
(unless (find-package "OP3_WALKING_MODULE_MSGS::WALKINGPARAM")
  (make-package "OP3_WALKING_MODULE_MSGS::WALKINGPARAM"))

(in-package "ROS")
;;//! \htmlinclude WalkingParam.msg.html


(defclass op3_walking_module_msgs::WalkingParam
  :super ros::object
  :slots (_init_x_offset _init_y_offset _init_z_offset _init_roll_offset _init_pitch_offset _init_yaw_offset _period_time _dsp_ratio _step_fb_ratio _x_move_amplitude _y_move_amplitude _z_move_amplitude _angle_move_amplitude _move_aim_on _balance_enable _balance_hip_roll_gain _balance_knee_gain _balance_ankle_roll_gain _balance_ankle_pitch_gain _y_swap_amplitude _z_swap_amplitude _arm_swing_gain _pelvis_offset _hip_pitch_offset _p_gain _i_gain _d_gain ))

(defmethod op3_walking_module_msgs::WalkingParam
  (:init
   (&key
    ((:init_x_offset __init_x_offset) 0.0)
    ((:init_y_offset __init_y_offset) 0.0)
    ((:init_z_offset __init_z_offset) 0.0)
    ((:init_roll_offset __init_roll_offset) 0.0)
    ((:init_pitch_offset __init_pitch_offset) 0.0)
    ((:init_yaw_offset __init_yaw_offset) 0.0)
    ((:period_time __period_time) 0.0)
    ((:dsp_ratio __dsp_ratio) 0.0)
    ((:step_fb_ratio __step_fb_ratio) 0.0)
    ((:x_move_amplitude __x_move_amplitude) 0.0)
    ((:y_move_amplitude __y_move_amplitude) 0.0)
    ((:z_move_amplitude __z_move_amplitude) 0.0)
    ((:angle_move_amplitude __angle_move_amplitude) 0.0)
    ((:move_aim_on __move_aim_on) nil)
    ((:balance_enable __balance_enable) nil)
    ((:balance_hip_roll_gain __balance_hip_roll_gain) 0.0)
    ((:balance_knee_gain __balance_knee_gain) 0.0)
    ((:balance_ankle_roll_gain __balance_ankle_roll_gain) 0.0)
    ((:balance_ankle_pitch_gain __balance_ankle_pitch_gain) 0.0)
    ((:y_swap_amplitude __y_swap_amplitude) 0.0)
    ((:z_swap_amplitude __z_swap_amplitude) 0.0)
    ((:arm_swing_gain __arm_swing_gain) 0.0)
    ((:pelvis_offset __pelvis_offset) 0.0)
    ((:hip_pitch_offset __hip_pitch_offset) 0.0)
    ((:p_gain __p_gain) 0)
    ((:i_gain __i_gain) 0)
    ((:d_gain __d_gain) 0)
    )
   (send-super :init)
   (setq _init_x_offset (float __init_x_offset))
   (setq _init_y_offset (float __init_y_offset))
   (setq _init_z_offset (float __init_z_offset))
   (setq _init_roll_offset (float __init_roll_offset))
   (setq _init_pitch_offset (float __init_pitch_offset))
   (setq _init_yaw_offset (float __init_yaw_offset))
   (setq _period_time (float __period_time))
   (setq _dsp_ratio (float __dsp_ratio))
   (setq _step_fb_ratio (float __step_fb_ratio))
   (setq _x_move_amplitude (float __x_move_amplitude))
   (setq _y_move_amplitude (float __y_move_amplitude))
   (setq _z_move_amplitude (float __z_move_amplitude))
   (setq _angle_move_amplitude (float __angle_move_amplitude))
   (setq _move_aim_on __move_aim_on)
   (setq _balance_enable __balance_enable)
   (setq _balance_hip_roll_gain (float __balance_hip_roll_gain))
   (setq _balance_knee_gain (float __balance_knee_gain))
   (setq _balance_ankle_roll_gain (float __balance_ankle_roll_gain))
   (setq _balance_ankle_pitch_gain (float __balance_ankle_pitch_gain))
   (setq _y_swap_amplitude (float __y_swap_amplitude))
   (setq _z_swap_amplitude (float __z_swap_amplitude))
   (setq _arm_swing_gain (float __arm_swing_gain))
   (setq _pelvis_offset (float __pelvis_offset))
   (setq _hip_pitch_offset (float __hip_pitch_offset))
   (setq _p_gain (round __p_gain))
   (setq _i_gain (round __i_gain))
   (setq _d_gain (round __d_gain))
   self)
  (:init_x_offset
   (&optional __init_x_offset)
   (if __init_x_offset (setq _init_x_offset __init_x_offset)) _init_x_offset)
  (:init_y_offset
   (&optional __init_y_offset)
   (if __init_y_offset (setq _init_y_offset __init_y_offset)) _init_y_offset)
  (:init_z_offset
   (&optional __init_z_offset)
   (if __init_z_offset (setq _init_z_offset __init_z_offset)) _init_z_offset)
  (:init_roll_offset
   (&optional __init_roll_offset)
   (if __init_roll_offset (setq _init_roll_offset __init_roll_offset)) _init_roll_offset)
  (:init_pitch_offset
   (&optional __init_pitch_offset)
   (if __init_pitch_offset (setq _init_pitch_offset __init_pitch_offset)) _init_pitch_offset)
  (:init_yaw_offset
   (&optional __init_yaw_offset)
   (if __init_yaw_offset (setq _init_yaw_offset __init_yaw_offset)) _init_yaw_offset)
  (:period_time
   (&optional __period_time)
   (if __period_time (setq _period_time __period_time)) _period_time)
  (:dsp_ratio
   (&optional __dsp_ratio)
   (if __dsp_ratio (setq _dsp_ratio __dsp_ratio)) _dsp_ratio)
  (:step_fb_ratio
   (&optional __step_fb_ratio)
   (if __step_fb_ratio (setq _step_fb_ratio __step_fb_ratio)) _step_fb_ratio)
  (:x_move_amplitude
   (&optional __x_move_amplitude)
   (if __x_move_amplitude (setq _x_move_amplitude __x_move_amplitude)) _x_move_amplitude)
  (:y_move_amplitude
   (&optional __y_move_amplitude)
   (if __y_move_amplitude (setq _y_move_amplitude __y_move_amplitude)) _y_move_amplitude)
  (:z_move_amplitude
   (&optional __z_move_amplitude)
   (if __z_move_amplitude (setq _z_move_amplitude __z_move_amplitude)) _z_move_amplitude)
  (:angle_move_amplitude
   (&optional __angle_move_amplitude)
   (if __angle_move_amplitude (setq _angle_move_amplitude __angle_move_amplitude)) _angle_move_amplitude)
  (:move_aim_on
   (&optional __move_aim_on)
   (if __move_aim_on (setq _move_aim_on __move_aim_on)) _move_aim_on)
  (:balance_enable
   (&optional __balance_enable)
   (if __balance_enable (setq _balance_enable __balance_enable)) _balance_enable)
  (:balance_hip_roll_gain
   (&optional __balance_hip_roll_gain)
   (if __balance_hip_roll_gain (setq _balance_hip_roll_gain __balance_hip_roll_gain)) _balance_hip_roll_gain)
  (:balance_knee_gain
   (&optional __balance_knee_gain)
   (if __balance_knee_gain (setq _balance_knee_gain __balance_knee_gain)) _balance_knee_gain)
  (:balance_ankle_roll_gain
   (&optional __balance_ankle_roll_gain)
   (if __balance_ankle_roll_gain (setq _balance_ankle_roll_gain __balance_ankle_roll_gain)) _balance_ankle_roll_gain)
  (:balance_ankle_pitch_gain
   (&optional __balance_ankle_pitch_gain)
   (if __balance_ankle_pitch_gain (setq _balance_ankle_pitch_gain __balance_ankle_pitch_gain)) _balance_ankle_pitch_gain)
  (:y_swap_amplitude
   (&optional __y_swap_amplitude)
   (if __y_swap_amplitude (setq _y_swap_amplitude __y_swap_amplitude)) _y_swap_amplitude)
  (:z_swap_amplitude
   (&optional __z_swap_amplitude)
   (if __z_swap_amplitude (setq _z_swap_amplitude __z_swap_amplitude)) _z_swap_amplitude)
  (:arm_swing_gain
   (&optional __arm_swing_gain)
   (if __arm_swing_gain (setq _arm_swing_gain __arm_swing_gain)) _arm_swing_gain)
  (:pelvis_offset
   (&optional __pelvis_offset)
   (if __pelvis_offset (setq _pelvis_offset __pelvis_offset)) _pelvis_offset)
  (:hip_pitch_offset
   (&optional __hip_pitch_offset)
   (if __hip_pitch_offset (setq _hip_pitch_offset __hip_pitch_offset)) _hip_pitch_offset)
  (:p_gain
   (&optional __p_gain)
   (if __p_gain (setq _p_gain __p_gain)) _p_gain)
  (:i_gain
   (&optional __i_gain)
   (if __i_gain (setq _i_gain __i_gain)) _i_gain)
  (:d_gain
   (&optional __d_gain)
   (if __d_gain (setq _d_gain __d_gain)) _d_gain)
  (:serialization-length
   ()
   (+
    ;; float32 _init_x_offset
    4
    ;; float32 _init_y_offset
    4
    ;; float32 _init_z_offset
    4
    ;; float32 _init_roll_offset
    4
    ;; float32 _init_pitch_offset
    4
    ;; float32 _init_yaw_offset
    4
    ;; float32 _period_time
    4
    ;; float32 _dsp_ratio
    4
    ;; float32 _step_fb_ratio
    4
    ;; float32 _x_move_amplitude
    4
    ;; float32 _y_move_amplitude
    4
    ;; float32 _z_move_amplitude
    4
    ;; float32 _angle_move_amplitude
    4
    ;; bool _move_aim_on
    1
    ;; bool _balance_enable
    1
    ;; float32 _balance_hip_roll_gain
    4
    ;; float32 _balance_knee_gain
    4
    ;; float32 _balance_ankle_roll_gain
    4
    ;; float32 _balance_ankle_pitch_gain
    4
    ;; float32 _y_swap_amplitude
    4
    ;; float32 _z_swap_amplitude
    4
    ;; float32 _arm_swing_gain
    4
    ;; float32 _pelvis_offset
    4
    ;; float32 _hip_pitch_offset
    4
    ;; int32 _p_gain
    4
    ;; int32 _i_gain
    4
    ;; int32 _d_gain
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _init_x_offset
       (sys::poke _init_x_offset (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _init_y_offset
       (sys::poke _init_y_offset (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _init_z_offset
       (sys::poke _init_z_offset (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _init_roll_offset
       (sys::poke _init_roll_offset (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _init_pitch_offset
       (sys::poke _init_pitch_offset (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _init_yaw_offset
       (sys::poke _init_yaw_offset (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _period_time
       (sys::poke _period_time (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _dsp_ratio
       (sys::poke _dsp_ratio (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _step_fb_ratio
       (sys::poke _step_fb_ratio (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _x_move_amplitude
       (sys::poke _x_move_amplitude (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y_move_amplitude
       (sys::poke _y_move_amplitude (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _z_move_amplitude
       (sys::poke _z_move_amplitude (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angle_move_amplitude
       (sys::poke _angle_move_amplitude (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _move_aim_on
       (if _move_aim_on (write-byte -1 s) (write-byte 0 s))
     ;; bool _balance_enable
       (if _balance_enable (write-byte -1 s) (write-byte 0 s))
     ;; float32 _balance_hip_roll_gain
       (sys::poke _balance_hip_roll_gain (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _balance_knee_gain
       (sys::poke _balance_knee_gain (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _balance_ankle_roll_gain
       (sys::poke _balance_ankle_roll_gain (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _balance_ankle_pitch_gain
       (sys::poke _balance_ankle_pitch_gain (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y_swap_amplitude
       (sys::poke _y_swap_amplitude (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _z_swap_amplitude
       (sys::poke _z_swap_amplitude (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _arm_swing_gain
       (sys::poke _arm_swing_gain (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pelvis_offset
       (sys::poke _pelvis_offset (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _hip_pitch_offset
       (sys::poke _hip_pitch_offset (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _p_gain
       (write-long _p_gain s)
     ;; int32 _i_gain
       (write-long _i_gain s)
     ;; int32 _d_gain
       (write-long _d_gain s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _init_x_offset
     (setq _init_x_offset (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _init_y_offset
     (setq _init_y_offset (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _init_z_offset
     (setq _init_z_offset (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _init_roll_offset
     (setq _init_roll_offset (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _init_pitch_offset
     (setq _init_pitch_offset (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _init_yaw_offset
     (setq _init_yaw_offset (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _period_time
     (setq _period_time (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _dsp_ratio
     (setq _dsp_ratio (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _step_fb_ratio
     (setq _step_fb_ratio (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _x_move_amplitude
     (setq _x_move_amplitude (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y_move_amplitude
     (setq _y_move_amplitude (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _z_move_amplitude
     (setq _z_move_amplitude (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angle_move_amplitude
     (setq _angle_move_amplitude (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _move_aim_on
     (setq _move_aim_on (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _balance_enable
     (setq _balance_enable (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _balance_hip_roll_gain
     (setq _balance_hip_roll_gain (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _balance_knee_gain
     (setq _balance_knee_gain (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _balance_ankle_roll_gain
     (setq _balance_ankle_roll_gain (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _balance_ankle_pitch_gain
     (setq _balance_ankle_pitch_gain (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y_swap_amplitude
     (setq _y_swap_amplitude (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _z_swap_amplitude
     (setq _z_swap_amplitude (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _arm_swing_gain
     (setq _arm_swing_gain (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pelvis_offset
     (setq _pelvis_offset (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _hip_pitch_offset
     (setq _hip_pitch_offset (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _p_gain
     (setq _p_gain (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _i_gain
     (setq _i_gain (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _d_gain
     (setq _d_gain (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get op3_walking_module_msgs::WalkingParam :md5sum-) "f136f28fe424a614afd1c68b67d3a9e8")
(setf (get op3_walking_module_msgs::WalkingParam :datatype-) "op3_walking_module_msgs/WalkingParam")
(setf (get op3_walking_module_msgs::WalkingParam :definition-)
      "####### walking init pose #######
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
int32 d_gain
")



(provide :op3_walking_module_msgs/WalkingParam "f136f28fe424a614afd1c68b67d3a9e8")


