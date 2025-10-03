;; Auto-generated. Do not edit!


(when (boundp 'robotis_controller_msgs::StatusMsg)
  (if (not (find-package "ROBOTIS_CONTROLLER_MSGS"))
    (make-package "ROBOTIS_CONTROLLER_MSGS"))
  (shadow 'StatusMsg (find-package "ROBOTIS_CONTROLLER_MSGS")))
(unless (find-package "ROBOTIS_CONTROLLER_MSGS::STATUSMSG")
  (make-package "ROBOTIS_CONTROLLER_MSGS::STATUSMSG"))

(in-package "ROS")
;;//! \htmlinclude StatusMsg.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*STATUS_UNKNOWN*" (find-package "ROBOTIS_CONTROLLER_MSGS::STATUSMSG"))
(shadow '*STATUS_UNKNOWN* (find-package "ROBOTIS_CONTROLLER_MSGS::STATUSMSG"))
(defconstant robotis_controller_msgs::StatusMsg::*STATUS_UNKNOWN* 0)
(intern "*STATUS_INFO*" (find-package "ROBOTIS_CONTROLLER_MSGS::STATUSMSG"))
(shadow '*STATUS_INFO* (find-package "ROBOTIS_CONTROLLER_MSGS::STATUSMSG"))
(defconstant robotis_controller_msgs::StatusMsg::*STATUS_INFO* 1)
(intern "*STATUS_WARN*" (find-package "ROBOTIS_CONTROLLER_MSGS::STATUSMSG"))
(shadow '*STATUS_WARN* (find-package "ROBOTIS_CONTROLLER_MSGS::STATUSMSG"))
(defconstant robotis_controller_msgs::StatusMsg::*STATUS_WARN* 2)
(intern "*STATUS_ERROR*" (find-package "ROBOTIS_CONTROLLER_MSGS::STATUSMSG"))
(shadow '*STATUS_ERROR* (find-package "ROBOTIS_CONTROLLER_MSGS::STATUSMSG"))
(defconstant robotis_controller_msgs::StatusMsg::*STATUS_ERROR* 3)
(defclass robotis_controller_msgs::StatusMsg
  :super ros::object
  :slots (_header _type _module_name _status_msg ))

(defmethod robotis_controller_msgs::StatusMsg
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:type __type) 0)
    ((:module_name __module_name) "")
    ((:status_msg __status_msg) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _type (round __type))
   (setq _module_name (string __module_name))
   (setq _status_msg (string __status_msg))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:module_name
   (&optional __module_name)
   (if __module_name (setq _module_name __module_name)) _module_name)
  (:status_msg
   (&optional __status_msg)
   (if __status_msg (setq _status_msg __status_msg)) _status_msg)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _type
    1
    ;; string _module_name
    4 (length _module_name)
    ;; string _status_msg
    4 (length _status_msg)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _type
       (write-byte _type s)
     ;; string _module_name
       (write-long (length _module_name) s) (princ _module_name s)
     ;; string _status_msg
       (write-long (length _status_msg) s) (princ _status_msg s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _type
     (setq _type (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; string _module_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _module_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _status_msg
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _status_msg (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get robotis_controller_msgs::StatusMsg :md5sum-) "8d546af394a35cb47516d4d064603220")
(setf (get robotis_controller_msgs::StatusMsg :datatype-) "robotis_controller_msgs/StatusMsg")
(setf (get robotis_controller_msgs::StatusMsg :definition-)
      "# Status Constants
uint8 STATUS_UNKNOWN = 0
uint8 STATUS_INFO = 1
uint8 STATUS_WARN = 2
uint8 STATUS_ERROR = 3

std_msgs/Header header
uint8 type
string module_name
string status_msg
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



(provide :robotis_controller_msgs/StatusMsg "8d546af394a35cb47516d4d064603220")


