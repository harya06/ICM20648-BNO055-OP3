;; Auto-generated. Do not edit!


(when (boundp 'op3_online_walking_module_msgs::KinematicsPose)
  (if (not (find-package "OP3_ONLINE_WALKING_MODULE_MSGS"))
    (make-package "OP3_ONLINE_WALKING_MODULE_MSGS"))
  (shadow 'KinematicsPose (find-package "OP3_ONLINE_WALKING_MODULE_MSGS")))
(unless (find-package "OP3_ONLINE_WALKING_MODULE_MSGS::KINEMATICSPOSE")
  (make-package "OP3_ONLINE_WALKING_MODULE_MSGS::KINEMATICSPOSE"))

(in-package "ROS")
;;//! \htmlinclude KinematicsPose.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass op3_online_walking_module_msgs::KinematicsPose
  :super ros::object
  :slots (_name _mov_time _pose ))

(defmethod op3_online_walking_module_msgs::KinematicsPose
  (:init
   (&key
    ((:name __name) "")
    ((:mov_time __mov_time) 0.0)
    ((:pose __pose) (instance geometry_msgs::Pose :init))
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _mov_time (float __mov_time))
   (setq _pose __pose)
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:mov_time
   (&optional __mov_time)
   (if __mov_time (setq _mov_time __mov_time)) _mov_time)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; float64 _mov_time
    8
    ;; geometry_msgs/Pose _pose
    (send _pose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; float64 _mov_time
       (sys::poke _mov_time (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; geometry_msgs/Pose _pose
       (send _pose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _mov_time
     (setq _mov_time (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; geometry_msgs/Pose _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;;
   self)
  )

(setf (get op3_online_walking_module_msgs::KinematicsPose :md5sum-) "bad2ca99be8aa9d056c70915989b5560")
(setf (get op3_online_walking_module_msgs::KinematicsPose :datatype-) "op3_online_walking_module_msgs/KinematicsPose")
(setf (get op3_online_walking_module_msgs::KinematicsPose :definition-)
      "string  name
float64  mov_time
geometry_msgs/Pose pose


================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :op3_online_walking_module_msgs/KinematicsPose "bad2ca99be8aa9d056c70915989b5560")


