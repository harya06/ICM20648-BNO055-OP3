;; Auto-generated. Do not edit!


(when (boundp 'op3_online_walking_module_msgs::FootStepCommand)
  (if (not (find-package "OP3_ONLINE_WALKING_MODULE_MSGS"))
    (make-package "OP3_ONLINE_WALKING_MODULE_MSGS"))
  (shadow 'FootStepCommand (find-package "OP3_ONLINE_WALKING_MODULE_MSGS")))
(unless (find-package "OP3_ONLINE_WALKING_MODULE_MSGS::FOOTSTEPCOMMAND")
  (make-package "OP3_ONLINE_WALKING_MODULE_MSGS::FOOTSTEPCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude FootStepCommand.msg.html


(defclass op3_online_walking_module_msgs::FootStepCommand
  :super ros::object
  :slots (_command _start_leg _step_num _step_time _step_length _side_length _step_angle ))

(defmethod op3_online_walking_module_msgs::FootStepCommand
  (:init
   (&key
    ((:command __command) "")
    ((:start_leg __start_leg) "")
    ((:step_num __step_num) 0)
    ((:step_time __step_time) 0.0)
    ((:step_length __step_length) 0.0)
    ((:side_length __side_length) 0.0)
    ((:step_angle __step_angle) 0.0)
    )
   (send-super :init)
   (setq _command (string __command))
   (setq _start_leg (string __start_leg))
   (setq _step_num (round __step_num))
   (setq _step_time (float __step_time))
   (setq _step_length (float __step_length))
   (setq _side_length (float __side_length))
   (setq _step_angle (float __step_angle))
   self)
  (:command
   (&optional __command)
   (if __command (setq _command __command)) _command)
  (:start_leg
   (&optional __start_leg)
   (if __start_leg (setq _start_leg __start_leg)) _start_leg)
  (:step_num
   (&optional __step_num)
   (if __step_num (setq _step_num __step_num)) _step_num)
  (:step_time
   (&optional __step_time)
   (if __step_time (setq _step_time __step_time)) _step_time)
  (:step_length
   (&optional __step_length)
   (if __step_length (setq _step_length __step_length)) _step_length)
  (:side_length
   (&optional __side_length)
   (if __side_length (setq _side_length __side_length)) _side_length)
  (:step_angle
   (&optional __step_angle)
   (if __step_angle (setq _step_angle __step_angle)) _step_angle)
  (:serialization-length
   ()
   (+
    ;; string _command
    4 (length _command)
    ;; string _start_leg
    4 (length _start_leg)
    ;; int32 _step_num
    4
    ;; float64 _step_time
    8
    ;; float64 _step_length
    8
    ;; float64 _side_length
    8
    ;; float64 _step_angle
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _command
       (write-long (length _command) s) (princ _command s)
     ;; string _start_leg
       (write-long (length _start_leg) s) (princ _start_leg s)
     ;; int32 _step_num
       (write-long _step_num s)
     ;; float64 _step_time
       (sys::poke _step_time (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _step_length
       (sys::poke _step_length (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _side_length
       (sys::poke _side_length (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _step_angle
       (sys::poke _step_angle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _command
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _command (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _start_leg
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _start_leg (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _step_num
     (setq _step_num (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _step_time
     (setq _step_time (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _step_length
     (setq _step_length (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _side_length
     (setq _side_length (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _step_angle
     (setq _step_angle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get op3_online_walking_module_msgs::FootStepCommand :md5sum-) "6b1e994ec1a40ca11db9d34d679a3f8c")
(setf (get op3_online_walking_module_msgs::FootStepCommand :datatype-) "op3_online_walking_module_msgs/FootStepCommand")
(setf (get op3_online_walking_module_msgs::FootStepCommand :definition-)
      "string  command
string  start_leg
int32   step_num
float64 step_time
float64 step_length
float64 side_length
float64 step_angle

")



(provide :op3_online_walking_module_msgs/FootStepCommand "6b1e994ec1a40ca11db9d34d679a3f8c")


