;; Auto-generated. Do not edit!


(when (boundp 'op3_action_module_msgs::IsRunning)
  (if (not (find-package "OP3_ACTION_MODULE_MSGS"))
    (make-package "OP3_ACTION_MODULE_MSGS"))
  (shadow 'IsRunning (find-package "OP3_ACTION_MODULE_MSGS")))
(unless (find-package "OP3_ACTION_MODULE_MSGS::ISRUNNING")
  (make-package "OP3_ACTION_MODULE_MSGS::ISRUNNING"))
(unless (find-package "OP3_ACTION_MODULE_MSGS::ISRUNNINGREQUEST")
  (make-package "OP3_ACTION_MODULE_MSGS::ISRUNNINGREQUEST"))
(unless (find-package "OP3_ACTION_MODULE_MSGS::ISRUNNINGRESPONSE")
  (make-package "OP3_ACTION_MODULE_MSGS::ISRUNNINGRESPONSE"))

(in-package "ROS")





(defclass op3_action_module_msgs::IsRunningRequest
  :super ros::object
  :slots ())

(defmethod op3_action_module_msgs::IsRunningRequest
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

(defclass op3_action_module_msgs::IsRunningResponse
  :super ros::object
  :slots (_is_running ))

(defmethod op3_action_module_msgs::IsRunningResponse
  (:init
   (&key
    ((:is_running __is_running) nil)
    )
   (send-super :init)
   (setq _is_running __is_running)
   self)
  (:is_running
   (&optional __is_running)
   (if __is_running (setq _is_running __is_running)) _is_running)
  (:serialization-length
   ()
   (+
    ;; bool _is_running
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _is_running
       (if _is_running (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _is_running
     (setq _is_running (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass op3_action_module_msgs::IsRunning
  :super ros::object
  :slots ())

(setf (get op3_action_module_msgs::IsRunning :md5sum-) "ae3468a1af93d845e943210e7cef5a54")
(setf (get op3_action_module_msgs::IsRunning :datatype-) "op3_action_module_msgs/IsRunning")
(setf (get op3_action_module_msgs::IsRunning :request) op3_action_module_msgs::IsRunningRequest)
(setf (get op3_action_module_msgs::IsRunning :response) op3_action_module_msgs::IsRunningResponse)

(defmethod op3_action_module_msgs::IsRunningRequest
  (:response () (instance op3_action_module_msgs::IsRunningResponse :init)))

(setf (get op3_action_module_msgs::IsRunningRequest :md5sum-) "ae3468a1af93d845e943210e7cef5a54")
(setf (get op3_action_module_msgs::IsRunningRequest :datatype-) "op3_action_module_msgs/IsRunningRequest")
(setf (get op3_action_module_msgs::IsRunningRequest :definition-)
      "
---
bool is_running
")

(setf (get op3_action_module_msgs::IsRunningResponse :md5sum-) "ae3468a1af93d845e943210e7cef5a54")
(setf (get op3_action_module_msgs::IsRunningResponse :datatype-) "op3_action_module_msgs/IsRunningResponse")
(setf (get op3_action_module_msgs::IsRunningResponse :definition-)
      "
---
bool is_running
")



(provide :op3_action_module_msgs/IsRunning "ae3468a1af93d845e943210e7cef5a54")


