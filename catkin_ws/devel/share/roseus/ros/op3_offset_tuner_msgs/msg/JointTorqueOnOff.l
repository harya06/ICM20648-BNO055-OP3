;; Auto-generated. Do not edit!


(when (boundp 'op3_offset_tuner_msgs::JointTorqueOnOff)
  (if (not (find-package "OP3_OFFSET_TUNER_MSGS"))
    (make-package "OP3_OFFSET_TUNER_MSGS"))
  (shadow 'JointTorqueOnOff (find-package "OP3_OFFSET_TUNER_MSGS")))
(unless (find-package "OP3_OFFSET_TUNER_MSGS::JOINTTORQUEONOFF")
  (make-package "OP3_OFFSET_TUNER_MSGS::JOINTTORQUEONOFF"))

(in-package "ROS")
;;//! \htmlinclude JointTorqueOnOff.msg.html


(defclass op3_offset_tuner_msgs::JointTorqueOnOff
  :super ros::object
  :slots (_joint_name _torque_enable ))

(defmethod op3_offset_tuner_msgs::JointTorqueOnOff
  (:init
   (&key
    ((:joint_name __joint_name) "")
    ((:torque_enable __torque_enable) nil)
    )
   (send-super :init)
   (setq _joint_name (string __joint_name))
   (setq _torque_enable __torque_enable)
   self)
  (:joint_name
   (&optional __joint_name)
   (if __joint_name (setq _joint_name __joint_name)) _joint_name)
  (:torque_enable
   (&optional __torque_enable)
   (if __torque_enable (setq _torque_enable __torque_enable)) _torque_enable)
  (:serialization-length
   ()
   (+
    ;; string _joint_name
    4 (length _joint_name)
    ;; bool _torque_enable
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _joint_name
       (write-long (length _joint_name) s) (princ _joint_name s)
     ;; bool _torque_enable
       (if _torque_enable (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _joint_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _joint_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _torque_enable
     (setq _torque_enable (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get op3_offset_tuner_msgs::JointTorqueOnOff :md5sum-) "b5a5bf39f4a0958f049fc5cc9d8fbd8b")
(setf (get op3_offset_tuner_msgs::JointTorqueOnOff :datatype-) "op3_offset_tuner_msgs/JointTorqueOnOff")
(setf (get op3_offset_tuner_msgs::JointTorqueOnOff :definition-)
      "string  joint_name
bool    torque_enable
")



(provide :op3_offset_tuner_msgs/JointTorqueOnOff "b5a5bf39f4a0958f049fc5cc9d8fbd8b")


