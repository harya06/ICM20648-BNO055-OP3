;; Auto-generated. Do not edit!


(when (boundp 'op3_online_walking_module_msgs::GetJointPose)
  (if (not (find-package "OP3_ONLINE_WALKING_MODULE_MSGS"))
    (make-package "OP3_ONLINE_WALKING_MODULE_MSGS"))
  (shadow 'GetJointPose (find-package "OP3_ONLINE_WALKING_MODULE_MSGS")))
(unless (find-package "OP3_ONLINE_WALKING_MODULE_MSGS::GETJOINTPOSE")
  (make-package "OP3_ONLINE_WALKING_MODULE_MSGS::GETJOINTPOSE"))
(unless (find-package "OP3_ONLINE_WALKING_MODULE_MSGS::GETJOINTPOSEREQUEST")
  (make-package "OP3_ONLINE_WALKING_MODULE_MSGS::GETJOINTPOSEREQUEST"))
(unless (find-package "OP3_ONLINE_WALKING_MODULE_MSGS::GETJOINTPOSERESPONSE")
  (make-package "OP3_ONLINE_WALKING_MODULE_MSGS::GETJOINTPOSERESPONSE"))

(in-package "ROS")





(defclass op3_online_walking_module_msgs::GetJointPoseRequest
  :super ros::object
  :slots ())

(defmethod op3_online_walking_module_msgs::GetJointPoseRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass op3_online_walking_module_msgs::GetJointPoseResponse
  :super ros::object
  :slots (_pose ))

(defmethod op3_online_walking_module_msgs::GetJointPoseResponse
  (:init
   (&key
    ((:pose __pose) (instance op3_online_walking_module_msgs::JointPose :init))
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
    ;; op3_online_walking_module_msgs/JointPose _pose
    (send _pose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; op3_online_walking_module_msgs/JointPose _pose
       (send _pose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; op3_online_walking_module_msgs/JointPose _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;;
   self)
  )

(defclass op3_online_walking_module_msgs::GetJointPose
  :super ros::object
  :slots ())

(setf (get op3_online_walking_module_msgs::GetJointPose :md5sum-) "c1260c47b7c0dc9a41639ef7730118eb")
(setf (get op3_online_walking_module_msgs::GetJointPose :datatype-) "op3_online_walking_module_msgs/GetJointPose")
(setf (get op3_online_walking_module_msgs::GetJointPose :request) op3_online_walking_module_msgs::GetJointPoseRequest)
(setf (get op3_online_walking_module_msgs::GetJointPose :response) op3_online_walking_module_msgs::GetJointPoseResponse)

(defmethod op3_online_walking_module_msgs::GetJointPoseRequest
  (:response () (instance op3_online_walking_module_msgs::GetJointPoseResponse :init)))

(setf (get op3_online_walking_module_msgs::GetJointPoseRequest :md5sum-) "c1260c47b7c0dc9a41639ef7730118eb")
(setf (get op3_online_walking_module_msgs::GetJointPoseRequest :datatype-) "op3_online_walking_module_msgs/GetJointPoseRequest")
(setf (get op3_online_walking_module_msgs::GetJointPoseRequest :definition-)
      "
---
op3_online_walking_module_msgs/JointPose pose


================================================================================
MSG: op3_online_walking_module_msgs/JointPose
float64  mov_time
sensor_msgs/JointState  pose

================================================================================
MSG: sensor_msgs/JointState
# This is a message that holds data to describe the state of a set of torque controlled joints. 
#
# The state of each joint (revolute or prismatic) is defined by:
#  * the position of the joint (rad or m),
#  * the velocity of the joint (rad/s or m/s) and 
#  * the effort that is applied in the joint (Nm or N).
#
# Each joint is uniquely identified by its name
# The header specifies the time at which the joint states were recorded. All the joint states
# in one message have to be recorded at the same time.
#
# This message consists of a multiple arrays, one for each part of the joint state. 
# The goal is to make each of the fields optional. When e.g. your joints have no
# effort associated with them, you can leave the effort array empty. 
#
# All arrays in this message should have the same size, or be empty.
# This is the only way to uniquely associate the joint name with the correct
# states.


Header header

string[] name
float64[] position
float64[] velocity
float64[] effort

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id
")

(setf (get op3_online_walking_module_msgs::GetJointPoseResponse :md5sum-) "c1260c47b7c0dc9a41639ef7730118eb")
(setf (get op3_online_walking_module_msgs::GetJointPoseResponse :datatype-) "op3_online_walking_module_msgs/GetJointPoseResponse")
(setf (get op3_online_walking_module_msgs::GetJointPoseResponse :definition-)
      "
---
op3_online_walking_module_msgs/JointPose pose


================================================================================
MSG: op3_online_walking_module_msgs/JointPose
float64  mov_time
sensor_msgs/JointState  pose

================================================================================
MSG: sensor_msgs/JointState
# This is a message that holds data to describe the state of a set of torque controlled joints. 
#
# The state of each joint (revolute or prismatic) is defined by:
#  * the position of the joint (rad or m),
#  * the velocity of the joint (rad/s or m/s) and 
#  * the effort that is applied in the joint (Nm or N).
#
# Each joint is uniquely identified by its name
# The header specifies the time at which the joint states were recorded. All the joint states
# in one message have to be recorded at the same time.
#
# This message consists of a multiple arrays, one for each part of the joint state. 
# The goal is to make each of the fields optional. When e.g. your joints have no
# effort associated with them, you can leave the effort array empty. 
#
# All arrays in this message should have the same size, or be empty.
# This is the only way to uniquely associate the joint name with the correct
# states.


Header header

string[] name
float64[] position
float64[] velocity
float64[] effort

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id
")



(provide :op3_online_walking_module_msgs/GetJointPose "c1260c47b7c0dc9a41639ef7730118eb")


