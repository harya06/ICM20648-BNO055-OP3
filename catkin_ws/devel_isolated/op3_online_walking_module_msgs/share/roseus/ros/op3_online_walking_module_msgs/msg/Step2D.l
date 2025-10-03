;; Auto-generated. Do not edit!


(when (boundp 'op3_online_walking_module_msgs::Step2D)
  (if (not (find-package "OP3_ONLINE_WALKING_MODULE_MSGS"))
    (make-package "OP3_ONLINE_WALKING_MODULE_MSGS"))
  (shadow 'Step2D (find-package "OP3_ONLINE_WALKING_MODULE_MSGS")))
(unless (find-package "OP3_ONLINE_WALKING_MODULE_MSGS::STEP2D")
  (make-package "OP3_ONLINE_WALKING_MODULE_MSGS::STEP2D"))

(in-package "ROS")
;;//! \htmlinclude Step2D.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(intern "*LEFT_FOOT_SWING*" (find-package "OP3_ONLINE_WALKING_MODULE_MSGS::STEP2D"))
(shadow '*LEFT_FOOT_SWING* (find-package "OP3_ONLINE_WALKING_MODULE_MSGS::STEP2D"))
(defconstant op3_online_walking_module_msgs::Step2D::*LEFT_FOOT_SWING* 1)
(intern "*RIGHT_FOOT_SWING*" (find-package "OP3_ONLINE_WALKING_MODULE_MSGS::STEP2D"))
(shadow '*RIGHT_FOOT_SWING* (find-package "OP3_ONLINE_WALKING_MODULE_MSGS::STEP2D"))
(defconstant op3_online_walking_module_msgs::Step2D::*RIGHT_FOOT_SWING* 2)
(intern "*STANDING*" (find-package "OP3_ONLINE_WALKING_MODULE_MSGS::STEP2D"))
(shadow '*STANDING* (find-package "OP3_ONLINE_WALKING_MODULE_MSGS::STEP2D"))
(defconstant op3_online_walking_module_msgs::Step2D::*STANDING* 3)
(defclass op3_online_walking_module_msgs::Step2D
  :super ros::object
  :slots (_step2d _moving_foot ))

(defmethod op3_online_walking_module_msgs::Step2D
  (:init
   (&key
    ((:step2d __step2d) (instance geometry_msgs::Pose2D :init))
    ((:moving_foot __moving_foot) 0)
    )
   (send-super :init)
   (setq _step2d __step2d)
   (setq _moving_foot (round __moving_foot))
   self)
  (:step2d
   (&rest __step2d)
   (if (keywordp (car __step2d))
       (send* _step2d __step2d)
     (progn
       (if __step2d (setq _step2d (car __step2d)))
       _step2d)))
  (:moving_foot
   (&optional __moving_foot)
   (if __moving_foot (setq _moving_foot __moving_foot)) _moving_foot)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Pose2D _step2d
    (send _step2d :serialization-length)
    ;; uint8 _moving_foot
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Pose2D _step2d
       (send _step2d :serialize s)
     ;; uint8 _moving_foot
       (write-byte _moving_foot s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Pose2D _step2d
     (send _step2d :deserialize buf ptr-) (incf ptr- (send _step2d :serialization-length))
   ;; uint8 _moving_foot
     (setq _moving_foot (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get op3_online_walking_module_msgs::Step2D :md5sum-) "8b716dffcd181458918724c59549dd00")
(setf (get op3_online_walking_module_msgs::Step2D :datatype-) "op3_online_walking_module_msgs/Step2D")
(setf (get op3_online_walking_module_msgs::Step2D :definition-)
      "#2D StepData

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



(provide :op3_online_walking_module_msgs/Step2D "8b716dffcd181458918724c59549dd00")


