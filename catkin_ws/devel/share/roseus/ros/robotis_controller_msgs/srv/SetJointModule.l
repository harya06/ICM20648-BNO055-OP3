;; Auto-generated. Do not edit!


(when (boundp 'robotis_controller_msgs::SetJointModule)
  (if (not (find-package "ROBOTIS_CONTROLLER_MSGS"))
    (make-package "ROBOTIS_CONTROLLER_MSGS"))
  (shadow 'SetJointModule (find-package "ROBOTIS_CONTROLLER_MSGS")))
(unless (find-package "ROBOTIS_CONTROLLER_MSGS::SETJOINTMODULE")
  (make-package "ROBOTIS_CONTROLLER_MSGS::SETJOINTMODULE"))
(unless (find-package "ROBOTIS_CONTROLLER_MSGS::SETJOINTMODULEREQUEST")
  (make-package "ROBOTIS_CONTROLLER_MSGS::SETJOINTMODULEREQUEST"))
(unless (find-package "ROBOTIS_CONTROLLER_MSGS::SETJOINTMODULERESPONSE")
  (make-package "ROBOTIS_CONTROLLER_MSGS::SETJOINTMODULERESPONSE"))

(in-package "ROS")





(defclass robotis_controller_msgs::SetJointModuleRequest
  :super ros::object
  :slots (_joint_name _module_name ))

(defmethod robotis_controller_msgs::SetJointModuleRequest
  (:init
   (&key
    ((:joint_name __joint_name) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:module_name __module_name) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _joint_name __joint_name)
   (setq _module_name __module_name)
   self)
  (:joint_name
   (&optional __joint_name)
   (if __joint_name (setq _joint_name __joint_name)) _joint_name)
  (:module_name
   (&optional __module_name)
   (if __module_name (setq _module_name __module_name)) _module_name)
  (:serialization-length
   ()
   (+
    ;; string[] _joint_name
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _joint_name)) 4
    ;; string[] _module_name
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _module_name)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string[] _joint_name
     (write-long (length _joint_name) s)
     (dolist (elem _joint_name)
       (write-long (length elem) s) (princ elem s)
       )
     ;; string[] _module_name
     (write-long (length _module_name) s)
     (dolist (elem _module_name)
       (write-long (length elem) s) (princ elem s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string[] _joint_name
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_name (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _joint_name i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; string[] _module_name
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _module_name (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _module_name i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;;
   self)
  )

(defclass robotis_controller_msgs::SetJointModuleResponse
  :super ros::object
  :slots (_result ))

(defmethod robotis_controller_msgs::SetJointModuleResponse
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

(defclass robotis_controller_msgs::SetJointModule
  :super ros::object
  :slots ())

(setf (get robotis_controller_msgs::SetJointModule :md5sum-) "018bd00fbf52c888e80425821afd0a3d")
(setf (get robotis_controller_msgs::SetJointModule :datatype-) "robotis_controller_msgs/SetJointModule")
(setf (get robotis_controller_msgs::SetJointModule :request) robotis_controller_msgs::SetJointModuleRequest)
(setf (get robotis_controller_msgs::SetJointModule :response) robotis_controller_msgs::SetJointModuleResponse)

(defmethod robotis_controller_msgs::SetJointModuleRequest
  (:response () (instance robotis_controller_msgs::SetJointModuleResponse :init)))

(setf (get robotis_controller_msgs::SetJointModuleRequest :md5sum-) "018bd00fbf52c888e80425821afd0a3d")
(setf (get robotis_controller_msgs::SetJointModuleRequest :datatype-) "robotis_controller_msgs/SetJointModuleRequest")
(setf (get robotis_controller_msgs::SetJointModuleRequest :definition-)
      "string[] joint_name
string[] module_name
---
bool result
")

(setf (get robotis_controller_msgs::SetJointModuleResponse :md5sum-) "018bd00fbf52c888e80425821afd0a3d")
(setf (get robotis_controller_msgs::SetJointModuleResponse :datatype-) "robotis_controller_msgs/SetJointModuleResponse")
(setf (get robotis_controller_msgs::SetJointModuleResponse :definition-)
      "string[] joint_name
string[] module_name
---
bool result
")



(provide :robotis_controller_msgs/SetJointModule "018bd00fbf52c888e80425821afd0a3d")


