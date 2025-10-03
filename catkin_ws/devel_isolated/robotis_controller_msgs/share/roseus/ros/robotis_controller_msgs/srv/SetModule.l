;; Auto-generated. Do not edit!


(when (boundp 'robotis_controller_msgs::SetModule)
  (if (not (find-package "ROBOTIS_CONTROLLER_MSGS"))
    (make-package "ROBOTIS_CONTROLLER_MSGS"))
  (shadow 'SetModule (find-package "ROBOTIS_CONTROLLER_MSGS")))
(unless (find-package "ROBOTIS_CONTROLLER_MSGS::SETMODULE")
  (make-package "ROBOTIS_CONTROLLER_MSGS::SETMODULE"))
(unless (find-package "ROBOTIS_CONTROLLER_MSGS::SETMODULEREQUEST")
  (make-package "ROBOTIS_CONTROLLER_MSGS::SETMODULEREQUEST"))
(unless (find-package "ROBOTIS_CONTROLLER_MSGS::SETMODULERESPONSE")
  (make-package "ROBOTIS_CONTROLLER_MSGS::SETMODULERESPONSE"))

(in-package "ROS")





(defclass robotis_controller_msgs::SetModuleRequest
  :super ros::object
  :slots (_module_name ))

(defmethod robotis_controller_msgs::SetModuleRequest
  (:init
   (&key
    ((:module_name __module_name) "")
    )
   (send-super :init)
   (setq _module_name (string __module_name))
   self)
  (:module_name
   (&optional __module_name)
   (if __module_name (setq _module_name __module_name)) _module_name)
  (:serialization-length
   ()
   (+
    ;; string _module_name
    4 (length _module_name)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _module_name
       (write-long (length _module_name) s) (princ _module_name s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _module_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _module_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass robotis_controller_msgs::SetModuleResponse
  :super ros::object
  :slots (_result ))

(defmethod robotis_controller_msgs::SetModuleResponse
  (:init
   (&key
    ((:result __result) nil)
    )
   (send-super :init)
   (setq _result __result)
   self)
  (:result
   (&optional __result)
   (if __result (setq _result __result)) _result)
  (:serialization-length
   ()
   (+
    ;; bool _result
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _result
       (if _result (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _result
     (setq _result (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass robotis_controller_msgs::SetModule
  :super ros::object
  :slots ())

(setf (get robotis_controller_msgs::SetModule :md5sum-) "fe41ec40765aa76b66cdabcf74584706")
(setf (get robotis_controller_msgs::SetModule :datatype-) "robotis_controller_msgs/SetModule")
(setf (get robotis_controller_msgs::SetModule :request) robotis_controller_msgs::SetModuleRequest)
(setf (get robotis_controller_msgs::SetModule :response) robotis_controller_msgs::SetModuleResponse)

(defmethod robotis_controller_msgs::SetModuleRequest
  (:response () (instance robotis_controller_msgs::SetModuleResponse :init)))

(setf (get robotis_controller_msgs::SetModuleRequest :md5sum-) "fe41ec40765aa76b66cdabcf74584706")
(setf (get robotis_controller_msgs::SetModuleRequest :datatype-) "robotis_controller_msgs/SetModuleRequest")
(setf (get robotis_controller_msgs::SetModuleRequest :definition-)
      "string module_name
---
bool result
")

(setf (get robotis_controller_msgs::SetModuleResponse :md5sum-) "fe41ec40765aa76b66cdabcf74584706")
(setf (get robotis_controller_msgs::SetModuleResponse :datatype-) "robotis_controller_msgs/SetModuleResponse")
(setf (get robotis_controller_msgs::SetModuleResponse :definition-)
      "string module_name
---
bool result
")



(provide :robotis_controller_msgs/SetModule "fe41ec40765aa76b66cdabcf74584706")


