;; Auto-generated. Do not edit!


(when (boundp 'robotis_controller_msgs::SyncWriteItem)
  (if (not (find-package "ROBOTIS_CONTROLLER_MSGS"))
    (make-package "ROBOTIS_CONTROLLER_MSGS"))
  (shadow 'SyncWriteItem (find-package "ROBOTIS_CONTROLLER_MSGS")))
(unless (find-package "ROBOTIS_CONTROLLER_MSGS::SYNCWRITEITEM")
  (make-package "ROBOTIS_CONTROLLER_MSGS::SYNCWRITEITEM"))

(in-package "ROS")
;;//! \htmlinclude SyncWriteItem.msg.html


(defclass robotis_controller_msgs::SyncWriteItem
  :super ros::object
  :slots (_item_name _joint_name _value ))

(defmethod robotis_controller_msgs::SyncWriteItem
  (:init
   (&key
    ((:item_name __item_name) "")
    ((:joint_name __joint_name) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:value __value) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _item_name (string __item_name))
   (setq _joint_name __joint_name)
   (setq _value __value)
   self)
  (:item_name
   (&optional __item_name)
   (if __item_name (setq _item_name __item_name)) _item_name)
  (:joint_name
   (&optional __joint_name)
   (if __joint_name (setq _joint_name __joint_name)) _joint_name)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:serialization-length
   ()
   (+
    ;; string _item_name
    4 (length _item_name)
    ;; string[] _joint_name
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _joint_name)) 4
    ;; uint32[] _value
    (* 4    (length _value)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _item_name
       (write-long (length _item_name) s) (princ _item_name s)
     ;; string[] _joint_name
     (write-long (length _joint_name) s)
     (dolist (elem _joint_name)
       (write-long (length elem) s) (princ elem s)
       )
     ;; uint32[] _value
     (write-long (length _value) s)
     (dotimes (i (length _value))
       (write-long (elt _value i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _item_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _item_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string[] _joint_name
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_name (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _joint_name i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; uint32[] _value
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _value (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _value i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get robotis_controller_msgs::SyncWriteItem :md5sum-) "f558e04f04dbcc25ce64aa1f45f3dbdd")
(setf (get robotis_controller_msgs::SyncWriteItem :datatype-) "robotis_controller_msgs/SyncWriteItem")
(setf (get robotis_controller_msgs::SyncWriteItem :definition-)
      "string item_name
string[] joint_name
uint32[] value
")



(provide :robotis_controller_msgs/SyncWriteItem "f558e04f04dbcc25ce64aa1f45f3dbdd")


