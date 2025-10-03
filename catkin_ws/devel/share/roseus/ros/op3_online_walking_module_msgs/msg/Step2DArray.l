;; Auto-generated. Do not edit!


(when (boundp 'op3_online_walking_module_msgs::Step2DArray)
  (if (not (find-package "OP3_ONLINE_WALKING_MODULE_MSGS"))
    (make-package "OP3_ONLINE_WALKING_MODULE_MSGS"))
  (shadow 'Step2DArray (find-package "OP3_ONLINE_WALKING_MODULE_MSGS")))
(unless (find-package "OP3_ONLINE_WALKING_MODULE_MSGS::STEP2DARRAY")
  (make-package "OP3_ONLINE_WALKING_MODULE_MSGS::STEP2DARRAY"))

(in-package "ROS")
;;//! \htmlinclude Step2DArray.msg.html


(defclass op3_online_walking_module_msgs::Step2DArray
  :super ros::object
  :slots (_step_time _footsteps_2d ))

(defmethod op3_online_walking_module_msgs::Step2DArray
  (:init
   (&key
    ((:step_time __step_time) 0.0)
    ((:footsteps_2d __footsteps_2d) (let (r) (dotimes (i 0) (push (instance op3_online_walking_module_msgs::Step2D :init) r)) r))
    )
   (send-super :init)
   (setq _step_time (float __step_time))
   (setq _footsteps_2d __footsteps_2d)
   self)
  (:step_time
   (&optional __step_time)
   (if __step_time (setq _step_time __step_time)) _step_time)
  (:footsteps_2d
   (&rest __footsteps_2d)
   (if (keywordp (car __footsteps_2d))
       (send* _footsteps_2d __footsteps_2d)
     (progn
       (if __footsteps_2d (setq _footsteps_2d (car __footsteps_2d)))
       _footsteps_2d)))
  (:serialization-length
   ()
   (+
    ;; float64 _step_time
    8
    ;; op3_online_walking_module_msgs/Step2D[] _footsteps_2d
    (apply #'+ (send-all _footsteps_2d :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _step_time
       (sys::poke _step_time (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; op3_online_walking_module_msgs/Step2D[] _footsteps_2d
     (write-long (length _footsteps_2d) s)
     (dolist (elem _footsteps_2d)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _step_time
     (setq _step_time (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; op3_online_walking_module_msgs/Step2D[] _footsteps_2d
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _footsteps_2d (let (r) (dotimes (i n) (push (instance op3_online_walking_module_msgs::Step2D :init) r)) r))
     (dolist (elem- _footsteps_2d)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get op3_online_walking_module_msgs::Step2DArray :md5sum-) "598423594f1c61377b299ae8d55d0f04")
(setf (get op3_online_walking_module_msgs::Step2DArray :datatype-) "op3_online_walking_module_msgs/Step2DArray")
(setf (get op3_online_walking_module_msgs::Step2DArray :definition-)
      "float64 step_time
Step2D[] footsteps_2d

================================================================================
MSG: op3_online_walking_module_msgs/Step2D
#2D StepData

geometry_msgs/Pose2D step2d   # step pose as relative offset to last leg


# which leg to be used (left/right/no, see below)
uint8 moving_foot   

uint8 LEFT_FOOT_SWING  = 1 # Left foot constant
uint8 RIGHT_FOOT_SWING = 2 # Right foot constant
uint8 STANDING         = 3 # Standing constant

================================================================================
MSG: geometry_msgs/Pose2D
# This expresses a position and orientation on a 2D manifold.

float64 x
float64 y
float64 theta
")



(provide :op3_online_walking_module_msgs/Step2DArray "598423594f1c61377b299ae8d55d0f04")


