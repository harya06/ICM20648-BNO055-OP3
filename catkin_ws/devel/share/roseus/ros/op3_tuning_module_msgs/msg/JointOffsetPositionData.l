;; Auto-generated. Do not edit!


(when (boundp 'op3_tuning_module_msgs::JointOffsetPositionData)
  (if (not (find-package "OP3_TUNING_MODULE_MSGS"))
    (make-package "OP3_TUNING_MODULE_MSGS"))
  (shadow 'JointOffsetPositionData (find-package "OP3_TUNING_MODULE_MSGS")))
(unless (find-package "OP3_TUNING_MODULE_MSGS::JOINTOFFSETPOSITIONDATA")
  (make-package "OP3_TUNING_MODULE_MSGS::JOINTOFFSETPOSITIONDATA"))

(in-package "ROS")
;;//! \htmlinclude JointOffsetPositionData.msg.html


(defclass op3_tuning_module_msgs::JointOffsetPositionData
  :super ros::object
  :slots (_joint_name _goal_value _offset_value _present_value _p_gain _i_gain _d_gain ))

(defmethod op3_tuning_module_msgs::JointOffsetPositionData
  (:init
   (&key
    ((:joint_name __joint_name) "")
    ((:goal_value __goal_value) 0.0)
    ((:offset_value __offset_value) 0.0)
    ((:present_value __present_value) 0.0)
    ((:p_gain __p_gain) 0)
    ((:i_gain __i_gain) 0)
    ((:d_gain __d_gain) 0)
    )
   (send-super :init)
   (setq _joint_name (string __joint_name))
   (setq _goal_value (float __goal_value))
   (setq _offset_value (float __offset_value))
   (setq _present_value (float __present_value))
   (setq _p_gain (round __p_gain))
   (setq _i_gain (round __i_gain))
   (setq _d_gain (round __d_gain))
   self)
  (:joint_name
   (&optional __joint_name)
   (if __joint_name (setq _joint_name __joint_name)) _joint_name)
  (:goal_value
   (&optional __goal_value)
   (if __goal_value (setq _goal_value __goal_value)) _goal_value)
  (:offset_value
   (&optional __offset_value)
   (if __offset_value (setq _offset_value __offset_value)) _offset_value)
  (:present_value
   (&optional __present_value)
   (if __present_value (setq _present_value __present_value)) _present_value)
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
    ;; string _joint_name
    4 (length _joint_name)
    ;; float64 _goal_value
    8
    ;; float64 _offset_value
    8
    ;; float64 _present_value
    8
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
     ;; string _joint_name
       (write-long (length _joint_name) s) (princ _joint_name s)
     ;; float64 _goal_value
       (sys::poke _goal_value (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _offset_value
       (sys::poke _offset_value (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _present_value
       (sys::poke _present_value (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
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
   ;; string _joint_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _joint_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _goal_value
     (setq _goal_value (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _offset_value
     (setq _offset_value (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _present_value
     (setq _present_value (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int32 _p_gain
     (setq _p_gain (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _i_gain
     (setq _i_gain (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _d_gain
     (setq _d_gain (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get op3_tuning_module_msgs::JointOffsetPositionData :md5sum-) "7fa2aed3bed37b6302a45de21d81e20b")
(setf (get op3_tuning_module_msgs::JointOffsetPositionData :datatype-) "op3_tuning_module_msgs/JointOffsetPositionData")
(setf (get op3_tuning_module_msgs::JointOffsetPositionData :definition-)
      "string  joint_name
float64 goal_value
float64 offset_value
float64 present_value
int32   p_gain
int32   i_gain
int32   d_gain
")



(provide :op3_tuning_module_msgs/JointOffsetPositionData "7fa2aed3bed37b6302a45de21d81e20b")


