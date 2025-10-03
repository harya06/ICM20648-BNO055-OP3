;; Auto-generated. Do not edit!


(when (boundp 'op3_action_module_msgs::StartAction)
  (if (not (find-package "OP3_ACTION_MODULE_MSGS"))
    (make-package "OP3_ACTION_MODULE_MSGS"))
  (shadow 'StartAction (find-package "OP3_ACTION_MODULE_MSGS")))
(unless (find-package "OP3_ACTION_MODULE_MSGS::STARTACTION")
  (make-package "OP3_ACTION_MODULE_MSGS::STARTACTION"))

(in-package "ROS")
;;//! \htmlinclude StartAction.msg.html


(defclass op3_action_module_msgs::StartAction
  :super ros::object
  :slots (_page_num _joint_name_array ))

(defmethod op3_action_module_msgs::StartAction
  (:init
   (&key
    ((:page_num __page_num) 0)
    ((:joint_name_array __joint_name_array) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _page_num (round __page_num))
   (setq _joint_name_array __joint_name_array)
   self)
  (:page_num
   (&optional __page_num)
   (if __page_num (setq _page_num __page_num)) _page_num)
  (:joint_name_array
   (&optional __joint_name_array)
   (if __joint_name_array (setq _joint_name_array __joint_name_array)) _joint_name_array)
  (:serialization-length
   ()
   (+
    ;; int32 _page_num
    4
    ;; string[] _joint_name_array
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _joint_name_array)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _page_num
       (write-long _page_num s)
     ;; string[] _joint_name_array
     (write-long (length _joint_name_array) s)
     (dolist (elem _joint_name_array)
       (write-long (length elem) s) (princ elem s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _page_num
     (setq _page_num (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string[] _joint_name_array
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_name_array (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _joint_name_array i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;;
   self)
  )

(setf (get op3_action_module_msgs::StartAction :md5sum-) "089f02f04489a5eddf9886b2ae161539")
(setf (get op3_action_module_msgs::StartAction :datatype-) "op3_action_module_msgs/StartAction")
(setf (get op3_action_module_msgs::StartAction :definition-)
      "int32     page_num
string[]  joint_name_array
")



(provide :op3_action_module_msgs/StartAction "089f02f04489a5eddf9886b2ae161539")


