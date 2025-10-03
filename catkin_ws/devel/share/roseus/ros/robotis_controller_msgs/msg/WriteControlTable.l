;; Auto-generated. Do not edit!


(when (boundp 'robotis_controller_msgs::WriteControlTable)
  (if (not (find-package "ROBOTIS_CONTROLLER_MSGS"))
    (make-package "ROBOTIS_CONTROLLER_MSGS"))
  (shadow 'WriteControlTable (find-package "ROBOTIS_CONTROLLER_MSGS")))
(unless (find-package "ROBOTIS_CONTROLLER_MSGS::WRITECONTROLTABLE")
  (make-package "ROBOTIS_CONTROLLER_MSGS::WRITECONTROLTABLE"))

(in-package "ROS")
;;//! \htmlinclude WriteControlTable.msg.html


(defclass robotis_controller_msgs::WriteControlTable
  :super ros::object
  :slots (_joint_name _start_item_name _data_length _data ))

(defmethod robotis_controller_msgs::WriteControlTable
  (:init
   (&key
    ((:joint_name __joint_name) "")
    ((:start_item_name __start_item_name) "")
    ((:data_length __data_length) 0)
    ((:data __data) (make-array 0 :initial-element 0 :element-type :char))
    )
   (send-super :init)
   (setq _joint_name (string __joint_name))
   (setq _start_item_name (string __start_item_name))
   (setq _data_length (round __data_length))
   (setq _data __data)
   self)
  (:joint_name
   (&optional __joint_name)
   (if __joint_name (setq _joint_name __joint_name)) _joint_name)
  (:start_item_name
   (&optional __start_item_name)
   (if __start_item_name (setq _start_item_name __start_item_name)) _start_item_name)
  (:data_length
   (&optional __data_length)
   (if __data_length (setq _data_length __data_length)) _data_length)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; string _joint_name
    4 (length _joint_name)
    ;; string _start_item_name
    4 (length _start_item_name)
    ;; uint16 _data_length
    2
    ;; uint8[] _data
    (* 1    (length _data)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _joint_name
       (write-long (length _joint_name) s) (princ _joint_name s)
     ;; string _start_item_name
       (write-long (length _start_item_name) s) (princ _start_item_name s)
     ;; uint16 _data_length
       (write-word _data_length s)
     ;; uint8[] _data
     (write-long (length _data) s)
     (princ _data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _joint_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _joint_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _start_item_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _start_item_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint16 _data_length
     (setq _data_length (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint8[] _data
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _data (make-array n :element-type :char))
     (replace _data buf :start2 ptr-) (incf ptr- n))
   ;;
   self)
  )

(setf (get robotis_controller_msgs::WriteControlTable :md5sum-) "0d4e0e3e28ede43d877e2fb8f0a1a011")
(setf (get robotis_controller_msgs::WriteControlTable :datatype-) "robotis_controller_msgs/WriteControlTable")
(setf (get robotis_controller_msgs::WriteControlTable :definition-)
      "string joint_name
string start_item_name
uint16 data_length
uint8[] data
")



(provide :robotis_controller_msgs/WriteControlTable "0d4e0e3e28ede43d877e2fb8f0a1a011")


