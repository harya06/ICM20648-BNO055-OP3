;; Auto-generated. Do not edit!


(when (boundp 'op3_online_walking_module_msgs::JointPose)
  (if (not (find-package "OP3_ONLINE_WALKING_MODULE_MSGS"))
    (make-package "OP3_ONLINE_WALKING_MODULE_MSGS"))
  (shadow 'JointPose (find-package "OP3_ONLINE_WALKING_MODULE_MSGS")))
(unless (find-package "OP3_ONLINE_WALKING_MODULE_MSGS::JOINTPOSE")
  (make-package "OP3_ONLINE_WALKING_MODULE_MSGS::JOINTPOSE"))

(in-package "ROS")
;;//! \htmlinclude JointPose.msg.html
(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))


(defclass op3_online_walking_module_msgs::JointPose
  :super ros::object
  :slots (_mov_time _pose ))

(defmethod op3_online_walking_module_msgs::JointPose
  (:init
   (&key
    ((:mov_time __mov_time) 0.0)
    ((:pose __pose) (instance sensor_msgs::JointState :init))
    )
   (send-super :init)
   (setq _mov_time (float __mov_time))
   (setq _pose __pose)
   self)
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
    ;; float64 _mov_time
    8
    ;; sensor_msgs/JointState _pose
    (send _pose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _mov_time
       (sys::poke _mov_time (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; sensor_msgs/JointState _pose
       (send _pose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _mov_time
     (setq _mov_time (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; sensor_msgs/JointState _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;;
   self)
  )

(setf (get op3_online_walking_module_msgs::JointPose :md5sum-) "93e783e02761adffe02faae8c7feff29")
(setf (get op3_online_walking_module_msgs::JointPose :datatype-) "op3_online_walking_module_msgs/JointPose")
(setf (get op3_online_walking_module_msgs::JointPose :definition-)
      "float64  mov_time
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



(provide :op3_online_walking_module_msgs/JointPose "93e783e02761adffe02faae8c7feff29")


