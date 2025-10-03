;; Auto-generated. Do not edit!


(when (boundp 'op3_tuning_module_msgs::JointTorqueOnOffArray)
  (if (not (find-package "OP3_TUNING_MODULE_MSGS"))
    (make-package "OP3_TUNING_MODULE_MSGS"))
  (shadow 'JointTorqueOnOffArray (find-package "OP3_TUNING_MODULE_MSGS")))
(unless (find-package "OP3_TUNING_MODULE_MSGS::JOINTTORQUEONOFFARRAY")
  (make-package "OP3_TUNING_MODULE_MSGS::JOINTTORQUEONOFFARRAY"))

(in-package "ROS")
;;//! \htmlinclude JointTorqueOnOffArray.msg.html


(defclass op3_tuning_module_msgs::JointTorqueOnOffArray
  :super ros::object
  :slots (_torque_enable_data ))

(defmethod op3_tuning_module_msgs::JointTorqueOnOffArray
  (:init
   (&key
    ((:torque_enable_data __torque_enable_data) (let (r) (dotimes (i 0) (push (instance op3_tuning_module_msgs::JointTorqueOnOff :init) r)) r))
    )
   (send-super :init)
   (setq _torque_enable_data __torque_enable_data)
   self)
  (:torque_enable_data
   (&rest __torque_enable_data)
   (if (keywordp (car __torque_enable_data))
       (send* _torque_enable_data __torque_enable_data)
     (progn
       (if __torque_enable_data (setq _torque_enable_data (car __torque_enable_data)))
       _torque_enable_data)))
  (:serialization-length
   ()
   (+
    ;; op3_tuning_module_msgs/JointTorqueOnOff[] _torque_enable_data
    (apply #'+ (send-all _torque_enable_data :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; op3_tuning_module_msgs/JointTorqueOnOff[] _torque_enable_data
     (write-long (length _torque_enable_data) s)
     (dolist (elem _torque_enable_data)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; op3_tuning_module_msgs/JointTorqueOnOff[] _torque_enable_data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _torque_enable_data (let (r) (dotimes (i n) (push (instance op3_tuning_module_msgs::JointTorqueOnOff :init) r)) r))
     (dolist (elem- _torque_enable_data)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get op3_tuning_module_msgs::JointTorqueOnOffArray :md5sum-) "1ca4db772b4d802ac00aebf4469fc8bf")
(setf (get op3_tuning_module_msgs::JointTorqueOnOffArray :datatype-) "op3_tuning_module_msgs/JointTorqueOnOffArray")
(setf (get op3_tuning_module_msgs::JointTorqueOnOffArray :definition-)
      "JointTorqueOnOff[] torque_enable_data
================================================================================
MSG: op3_tuning_module_msgs/JointTorqueOnOff
string  joint_name
bool    torque_enable
")



(provide :op3_tuning_module_msgs/JointTorqueOnOffArray "1ca4db772b4d802ac00aebf4469fc8bf")


