;; Auto-generated. Do not edit!


(when (boundp 'robotis_controller_msgs::GetJointModule)
  (if (not (find-package "ROBOTIS_CONTROLLER_MSGS"))
    (make-package "ROBOTIS_CONTROLLER_MSGS"))
  (shadow 'GetJointModule (find-package "ROBOTIS_CONTROLLER_MSGS")))
(unless (find-package "ROBOTIS_CONTROLLER_MSGS::GETJOINTMODULE")
  (make-package "ROBOTIS_CONTROLLER_MSGS::GETJOINTMODULE"))
(unless (find-package "ROBOTIS_CONTROLLER_MSGS::GETJOINTMODULEREQUEST")
  (make-package "ROBOTIS_CONTROLLER_MSGS::GETJOINTMODULEREQUEST"))
(unless (find-package "ROBOTIS_CONTROLLER_MSGS::GETJOINTMODULERESPONSE")
  (make-package "ROBOTIS_CONTROLLER_MSGS::GETJOINTMODULERESPONSE"))

(in-package "ROS")





(defclass robotis_controller_msgs::GetJointModuleRequest
  :super ros::object
  :slots (_joint_name ))

(defmethod robotis_controller_msgs::GetJointModuleRequest
  (:init
   (&key
    ((:joint_name __joint_name) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _joint_name __joint_name)
   self)
  (:joint_name
   (&optional __joint_name)
   (if __joint_name (setq _joint_name __joint_name)) _joint_name)
  (:serialization-length
   ()
   (+
    ;; string[] _joint_name
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _joint_name)) 4
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
   ;;
   self)
  )

(defclass robotis_controller_msgs::GetJointModuleResponse
  :super ros::object
  :slots (_joint_name _module_name ))

(defmethod robotis_controller_msgs::GetJointModuleResponse
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

(defclass robotis_controller_msgs::GetJointModule
  :super ros::object
  :slots ())

(setf (get robotis_controller_msgs::GetJointModule :md5sum-) "55513368a0fa6f6cbd22bf1bfb152faa")
(setf (get robotis_controller_msgs::GetJointModule :datatype-) "robotis_controller_msgs/GetJointModule")
(setf (get robotis_controller_msgs::GetJointModule :request) robotis_controller_msgs::GetJointModuleRequest)
(setf (get robotis_controller_msgs::GetJointModule :response) robotis_controller_msgs::GetJointModuleResponse)

(defmethod robotis_controller_msgs::GetJointModuleRequest
  (:response () (instance robotis_controller_msgs::GetJointModuleResponse :init)))

(setf (get robotis_controller_msgs::GetJointModuleRequest :md5sum-) "55513368a0fa6f6cbd22bf1bfb152faa")
(setf (get robotis_controller_msgs::GetJointModuleRequest :datatype-) "robotis_controller_msgs/GetJointModuleRequest")
(setf (get robotis_controller_msgs::GetJointModuleRequest :definition-)
      "string[] joint_name
---
string[] joint_name
string[] module_name
")

(setf (get robotis_controller_msgs::GetJointModuleResponse :md5sum-) "55513368a0fa6f6cbd22bf1bfb152faa")
(setf (get robotis_controller_msgs::GetJointModuleResponse :datatype-) "robotis_controller_msgs/GetJointModuleResponse")
(setf (get robotis_controller_msgs::GetJointModuleResponse :definition-)
      "string[] joint_name
---
string[] joint_name
string[] module_name
")



(provide :robotis_controller_msgs/GetJointModule "55513368a0fa6f6cbd22bf1bfb152faa")


