;; Auto-generated. Do not edit!


(when (boundp 'robotis_controller_msgs::LoadOffset)
  (if (not (find-package "ROBOTIS_CONTROLLER_MSGS"))
    (make-package "ROBOTIS_CONTROLLER_MSGS"))
  (shadow 'LoadOffset (find-package "ROBOTIS_CONTROLLER_MSGS")))
(unless (find-package "ROBOTIS_CONTROLLER_MSGS::LOADOFFSET")
  (make-package "ROBOTIS_CONTROLLER_MSGS::LOADOFFSET"))
(unless (find-package "ROBOTIS_CONTROLLER_MSGS::LOADOFFSETREQUEST")
  (make-package "ROBOTIS_CONTROLLER_MSGS::LOADOFFSETREQUEST"))
(unless (find-package "ROBOTIS_CONTROLLER_MSGS::LOADOFFSETRESPONSE")
  (make-package "ROBOTIS_CONTROLLER_MSGS::LOADOFFSETRESPONSE"))

(in-package "ROS")





(defclass robotis_controller_msgs::LoadOffsetRequest
  :super ros::object
  :slots (_file_path ))

(defmethod robotis_controller_msgs::LoadOffsetRequest
  (:init
   (&key
    ((:file_path __file_path) "")
    )
   (send-super :init)
   (setq _file_path (string __file_path))
   self)
  (:file_path
   (&optional __file_path)
   (if __file_path (setq _file_path __file_path)) _file_path)
  (:serialization-length
   ()
   (+
    ;; string _file_path
    4 (length _file_path)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _file_path
       (write-long (length _file_path) s) (princ _file_path s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _file_path
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _file_path (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass robotis_controller_msgs::LoadOffsetResponse
  :super ros::object
  :slots (_result ))

(defmethod robotis_controller_msgs::LoadOffsetResponse
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

(defclass robotis_controller_msgs::LoadOffset
  :super ros::object
  :slots ())

(setf (get robotis_controller_msgs::LoadOffset :md5sum-) "b223f245a1d13c9e179ae29717131752")
(setf (get robotis_controller_msgs::LoadOffset :datatype-) "robotis_controller_msgs/LoadOffset")
(setf (get robotis_controller_msgs::LoadOffset :request) robotis_controller_msgs::LoadOffsetRequest)
(setf (get robotis_controller_msgs::LoadOffset :response) robotis_controller_msgs::LoadOffsetResponse)

(defmethod robotis_controller_msgs::LoadOffsetRequest
  (:response () (instance robotis_controller_msgs::LoadOffsetResponse :init)))

(setf (get robotis_controller_msgs::LoadOffsetRequest :md5sum-) "b223f245a1d13c9e179ae29717131752")
(setf (get robotis_controller_msgs::LoadOffsetRequest :datatype-) "robotis_controller_msgs/LoadOffsetRequest")
(setf (get robotis_controller_msgs::LoadOffsetRequest :definition-)
      "string file_path
---
bool result
")

(setf (get robotis_controller_msgs::LoadOffsetResponse :md5sum-) "b223f245a1d13c9e179ae29717131752")
(setf (get robotis_controller_msgs::LoadOffsetResponse :datatype-) "robotis_controller_msgs/LoadOffsetResponse")
(setf (get robotis_controller_msgs::LoadOffsetResponse :definition-)
      "string file_path
---
bool result
")



(provide :robotis_controller_msgs/LoadOffset "b223f245a1d13c9e179ae29717131752")


