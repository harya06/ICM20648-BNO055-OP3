;; Auto-generated. Do not edit!


(when (boundp 'op3_online_walking_module_msgs::PreviewRequest)
  (if (not (find-package "OP3_ONLINE_WALKING_MODULE_MSGS"))
    (make-package "OP3_ONLINE_WALKING_MODULE_MSGS"))
  (shadow 'PreviewRequest (find-package "OP3_ONLINE_WALKING_MODULE_MSGS")))
(unless (find-package "OP3_ONLINE_WALKING_MODULE_MSGS::PREVIEWREQUEST")
  (make-package "OP3_ONLINE_WALKING_MODULE_MSGS::PREVIEWREQUEST"))

(in-package "ROS")
;;//! \htmlinclude PreviewRequest.msg.html


(defclass op3_online_walking_module_msgs::PreviewRequest
  :super ros::object
  :slots (_control_cycle _lipm_height ))

(defmethod op3_online_walking_module_msgs::PreviewRequest
  (:init
   (&key
    ((:control_cycle __control_cycle) 0.0)
    ((:lipm_height __lipm_height) 0.0)
    )
   (send-super :init)
   (setq _control_cycle (float __control_cycle))
   (setq _lipm_height (float __lipm_height))
   self)
  (:control_cycle
   (&optional __control_cycle)
   (if __control_cycle (setq _control_cycle __control_cycle)) _control_cycle)
  (:lipm_height
   (&optional __lipm_height)
   (if __lipm_height (setq _lipm_height __lipm_height)) _lipm_height)
  (:serialization-length
   ()
   (+
    ;; float64 _control_cycle
    8
    ;; float64 _lipm_height
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _control_cycle
       (sys::poke _control_cycle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _lipm_height
       (sys::poke _lipm_height (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _control_cycle
     (setq _control_cycle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _lipm_height
     (setq _lipm_height (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get op3_online_walking_module_msgs::PreviewRequest :md5sum-) "5dd8c29c2c1ccae01b0503604675335f")
(setf (get op3_online_walking_module_msgs::PreviewRequest :datatype-) "op3_online_walking_module_msgs/PreviewRequest")
(setf (get op3_online_walking_module_msgs::PreviewRequest :definition-)
      "float64 control_cycle
float64 lipm_height

")



(provide :op3_online_walking_module_msgs/PreviewRequest "5dd8c29c2c1ccae01b0503604675335f")


