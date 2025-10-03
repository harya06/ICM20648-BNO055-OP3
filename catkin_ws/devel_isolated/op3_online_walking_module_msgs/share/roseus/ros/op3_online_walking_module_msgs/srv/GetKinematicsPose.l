;; Auto-generated. Do not edit!


(when (boundp 'op3_online_walking_module_msgs::GetKinematicsPose)
  (if (not (find-package "OP3_ONLINE_WALKING_MODULE_MSGS"))
    (make-package "OP3_ONLINE_WALKING_MODULE_MSGS"))
  (shadow 'GetKinematicsPose (find-package "OP3_ONLINE_WALKING_MODULE_MSGS")))
(unless (find-package "OP3_ONLINE_WALKING_MODULE_MSGS::GETKINEMATICSPOSE")
  (make-package "OP3_ONLINE_WALKING_MODULE_MSGS::GETKINEMATICSPOSE"))
(unless (find-package "OP3_ONLINE_WALKING_MODULE_MSGS::GETKINEMATICSPOSEREQUEST")
  (make-package "OP3_ONLINE_WALKING_MODULE_MSGS::GETKINEMATICSPOSEREQUEST"))
(unless (find-package "OP3_ONLINE_WALKING_MODULE_MSGS::GETKINEMATICSPOSERESPONSE")
  (make-package "OP3_ONLINE_WALKING_MODULE_MSGS::GETKINEMATICSPOSERESPONSE"))

(in-package "ROS")





(defclass op3_online_walking_module_msgs::GetKinematicsPoseRequest
  :super ros::object
  :slots (_name ))

(defmethod op3_online_walking_module_msgs::GetKinematicsPoseRequest
  (:init
   (&key
    ((:name __name) "")
    )
   (send-super :init)
   (setq _name (string __name))
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass op3_online_walking_module_msgs::GetKinematicsPoseResponse
  :super ros::object
  :slots (_pose ))

(defmethod op3_online_walking_module_msgs::GetKinematicsPoseResponse
  (:init
   (&key
    ((:pose __pose) (instance op3_online_walking_module_msgs::KinematicsPose :init))
    )
   (send-super :init)
   (setq _pose __pose)
   self)
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
    ;; op3_online_walking_module_msgs/KinematicsPose _pose
    (send _pose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; op3_online_walking_module_msgs/KinematicsPose _pose
       (send _pose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; op3_online_walking_module_msgs/KinematicsPose _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;;
   self)
  )

(defclass op3_online_walking_module_msgs::GetKinematicsPose
  :super ros::object
  :slots ())

(setf (get op3_online_walking_module_msgs::GetKinematicsPose :md5sum-) "aad71eecbad83fbcb051be59f9fbb914")
(setf (get op3_online_walking_module_msgs::GetKinematicsPose :datatype-) "op3_online_walking_module_msgs/GetKinematicsPose")
(setf (get op3_online_walking_module_msgs::GetKinematicsPose :request) op3_online_walking_module_msgs::GetKinematicsPoseRequest)
(setf (get op3_online_walking_module_msgs::GetKinematicsPose :response) op3_online_walking_module_msgs::GetKinematicsPoseResponse)

(defmethod op3_online_walking_module_msgs::GetKinematicsPoseRequest
  (:response () (instance op3_online_walking_module_msgs::GetKinematicsPoseResponse :init)))

(setf (get op3_online_walking_module_msgs::GetKinematicsPoseRequest :md5sum-) "aad71eecbad83fbcb051be59f9fbb914")
(setf (get op3_online_walking_module_msgs::GetKinematicsPoseRequest :datatype-) "op3_online_walking_module_msgs/GetKinematicsPoseRequest")
(setf (get op3_online_walking_module_msgs::GetKinematicsPoseRequest :definition-)
      "string    name
---
op3_online_walking_module_msgs/KinematicsPose pose


================================================================================
MSG: op3_online_walking_module_msgs/KinematicsPose
string  name
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

(setf (get op3_online_walking_module_msgs::GetKinematicsPoseResponse :md5sum-) "aad71eecbad83fbcb051be59f9fbb914")
(setf (get op3_online_walking_module_msgs::GetKinematicsPoseResponse :datatype-) "op3_online_walking_module_msgs/GetKinematicsPoseResponse")
(setf (get op3_online_walking_module_msgs::GetKinematicsPoseResponse :definition-)
      "string    name
---
op3_online_walking_module_msgs/KinematicsPose pose


================================================================================
MSG: op3_online_walking_module_msgs/KinematicsPose
string  name
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



(provide :op3_online_walking_module_msgs/GetKinematicsPose "aad71eecbad83fbcb051be59f9fbb914")


