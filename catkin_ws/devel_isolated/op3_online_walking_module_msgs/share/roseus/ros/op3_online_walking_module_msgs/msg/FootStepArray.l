;; Auto-generated. Do not edit!


(when (boundp 'op3_online_walking_module_msgs::FootStepArray)
  (if (not (find-package "OP3_ONLINE_WALKING_MODULE_MSGS"))
    (make-package "OP3_ONLINE_WALKING_MODULE_MSGS"))
  (shadow 'FootStepArray (find-package "OP3_ONLINE_WALKING_MODULE_MSGS")))
(unless (find-package "OP3_ONLINE_WALKING_MODULE_MSGS::FOOTSTEPARRAY")
  (make-package "OP3_ONLINE_WALKING_MODULE_MSGS::FOOTSTEPARRAY"))

(in-package "ROS")
;;//! \htmlinclude FootStepArray.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(intern "*LEFT_FOOT*" (find-package "OP3_ONLINE_WALKING_MODULE_MSGS::FOOTSTEPARRAY"))
(shadow '*LEFT_FOOT* (find-package "OP3_ONLINE_WALKING_MODULE_MSGS::FOOTSTEPARRAY"))
(defconstant op3_online_walking_module_msgs::FootStepArray::*LEFT_FOOT* 0)
(intern "*RIGHT_FOOT*" (find-package "OP3_ONLINE_WALKING_MODULE_MSGS::FOOTSTEPARRAY"))
(shadow '*RIGHT_FOOT* (find-package "OP3_ONLINE_WALKING_MODULE_MSGS::FOOTSTEPARRAY"))
(defconstant op3_online_walking_module_msgs::FootStepArray::*RIGHT_FOOT* 1)
(defclass op3_online_walking_module_msgs::FootStepArray
  :super ros::object
  :slots (_moving_foot _data ))

(defmethod op3_online_walking_module_msgs::FootStepArray
  (:init
   (&key
    ((:moving_foot __moving_foot) (make-array 0 :initial-element 0 :element-type :integer))
    ((:data __data) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Pose2D :init) r)) r))
    )
   (send-super :init)
   (setq _moving_foot __moving_foot)
   (setq _data __data)
   self)
  (:moving_foot
   (&optional __moving_foot)
   (if __moving_foot (setq _moving_foot __moving_foot)) _moving_foot)
  (:data
   (&rest __data)
   (if (keywordp (car __data))
       (send* _data __data)
     (progn
       (if __data (setq _data (car __data)))
       _data)))
  (:serialization-length
   ()
   (+
    ;; int32[] _moving_foot
    (* 4    (length _moving_foot)) 4
    ;; geometry_msgs/Pose2D[] _data
    (apply #'+ (send-all _data :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32[] _moving_foot
     (write-long (length _moving_foot) s)
     (dotimes (i (length _moving_foot))
       (write-long (elt _moving_foot i) s)
       )
     ;; geometry_msgs/Pose2D[] _data
     (write-long (length _data) s)
     (dolist (elem _data)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32[] _moving_foot
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _moving_foot (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _moving_foot i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; geometry_msgs/Pose2D[] _data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _data (let (r) (dotimes (i n) (push (instance geometry_msgs::Pose2D :init) r)) r))
     (dolist (elem- _data)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get op3_online_walking_module_msgs::FootStepArray :md5sum-) "d024e75ed0a26f72c71cd63c2720fb63")
(setf (get op3_online_walking_module_msgs::FootStepArray :datatype-) "op3_online_walking_module_msgs/FootStepArray")
(setf (get op3_online_walking_module_msgs::FootStepArray :definition-)
      "
int32 LEFT_FOOT  = 0 # Left foot constant
int32 RIGHT_FOOT = 1 # Right foot constant

int32[]                moving_foot
geometry_msgs/Pose2D[] data

================================================================================
MSG: geometry_msgs/Pose2D
# This expresses a position and orientation on a 2D manifold.

float64 x
float64 y
float64 theta
")



(provide :op3_online_walking_module_msgs/FootStepArray "d024e75ed0a26f72c71cd63c2720fb63")


