;; Auto-generated. Do not edit!


(when (boundp 'op3_offset_tuner_msgs::GetPresentJointOffsetData)
  (if (not (find-package "OP3_OFFSET_TUNER_MSGS"))
    (make-package "OP3_OFFSET_TUNER_MSGS"))
  (shadow 'GetPresentJointOffsetData (find-package "OP3_OFFSET_TUNER_MSGS")))
(unless (find-package "OP3_OFFSET_TUNER_MSGS::GETPRESENTJOINTOFFSETDATA")
  (make-package "OP3_OFFSET_TUNER_MSGS::GETPRESENTJOINTOFFSETDATA"))
(unless (find-package "OP3_OFFSET_TUNER_MSGS::GETPRESENTJOINTOFFSETDATAREQUEST")
  (make-package "OP3_OFFSET_TUNER_MSGS::GETPRESENTJOINTOFFSETDATAREQUEST"))
(unless (find-package "OP3_OFFSET_TUNER_MSGS::GETPRESENTJOINTOFFSETDATARESPONSE")
  (make-package "OP3_OFFSET_TUNER_MSGS::GETPRESENTJOINTOFFSETDATARESPONSE"))

(in-package "ROS")





(defclass op3_offset_tuner_msgs::GetPresentJointOffsetDataRequest
  :super ros::object
  :slots ())

(defmethod op3_offset_tuner_msgs::GetPresentJointOffsetDataRequest
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

(defclass op3_offset_tuner_msgs::GetPresentJointOffsetDataResponse
  :super ros::object
  :slots (_present_data_array ))

(defmethod op3_offset_tuner_msgs::GetPresentJointOffsetDataResponse
  (:init
   (&key
    ((:present_data_array __present_data_array) (let (r) (dotimes (i 0) (push (instance op3_offset_tuner_msgs::JointOffsetPositionData :init) r)) r))
    )
   (send-super :init)
   (setq _present_data_array __present_data_array)
   self)
  (:present_data_array
   (&rest __present_data_array)
   (if (keywordp (car __present_data_array))
       (send* _present_data_array __present_data_array)
     (progn
       (if __present_data_array (setq _present_data_array (car __present_data_array)))
       _present_data_array)))
  (:serialization-length
   ()
   (+
    ;; op3_offset_tuner_msgs/JointOffsetPositionData[] _present_data_array
    (apply #'+ (send-all _present_data_array :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; op3_offset_tuner_msgs/JointOffsetPositionData[] _present_data_array
     (write-long (length _present_data_array) s)
     (dolist (elem _present_data_array)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; op3_offset_tuner_msgs/JointOffsetPositionData[] _present_data_array
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _present_data_array (let (r) (dotimes (i n) (push (instance op3_offset_tuner_msgs::JointOffsetPositionData :init) r)) r))
     (dolist (elem- _present_data_array)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(defclass op3_offset_tuner_msgs::GetPresentJointOffsetData
  :super ros::object
  :slots ())

(setf (get op3_offset_tuner_msgs::GetPresentJointOffsetData :md5sum-) "fe3cab7abbd49468a5d502064853404e")
(setf (get op3_offset_tuner_msgs::GetPresentJointOffsetData :datatype-) "op3_offset_tuner_msgs/GetPresentJointOffsetData")
(setf (get op3_offset_tuner_msgs::GetPresentJointOffsetData :request) op3_offset_tuner_msgs::GetPresentJointOffsetDataRequest)
(setf (get op3_offset_tuner_msgs::GetPresentJointOffsetData :response) op3_offset_tuner_msgs::GetPresentJointOffsetDataResponse)

(defmethod op3_offset_tuner_msgs::GetPresentJointOffsetDataRequest
  (:response () (instance op3_offset_tuner_msgs::GetPresentJointOffsetDataResponse :init)))

(setf (get op3_offset_tuner_msgs::GetPresentJointOffsetDataRequest :md5sum-) "fe3cab7abbd49468a5d502064853404e")
(setf (get op3_offset_tuner_msgs::GetPresentJointOffsetDataRequest :datatype-) "op3_offset_tuner_msgs/GetPresentJointOffsetDataRequest")
(setf (get op3_offset_tuner_msgs::GetPresentJointOffsetDataRequest :definition-)
      "
---
JointOffsetPositionData[] present_data_array

================================================================================
MSG: op3_offset_tuner_msgs/JointOffsetPositionData
string  joint_name
float64 goal_value
float64 offset_value
float64 present_value
int32   p_gain
int32   i_gain
")

(setf (get op3_offset_tuner_msgs::GetPresentJointOffsetDataResponse :md5sum-) "fe3cab7abbd49468a5d502064853404e")
(setf (get op3_offset_tuner_msgs::GetPresentJointOffsetDataResponse :datatype-) "op3_offset_tuner_msgs/GetPresentJointOffsetDataResponse")
(setf (get op3_offset_tuner_msgs::GetPresentJointOffsetDataResponse :definition-)
      "
---
JointOffsetPositionData[] present_data_array

================================================================================
MSG: op3_offset_tuner_msgs/JointOffsetPositionData
string  joint_name
float64 goal_value
float64 offset_value
float64 present_value
int32   p_gain
int32   i_gain
")



(provide :op3_offset_tuner_msgs/GetPresentJointOffsetData "fe3cab7abbd49468a5d502064853404e")


