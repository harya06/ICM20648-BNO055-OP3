;; Auto-generated. Do not edit!


(when (boundp 'robotis_controller_msgs::SensorYPR)
  (if (not (find-package "ROBOTIS_CONTROLLER_MSGS"))
    (make-package "ROBOTIS_CONTROLLER_MSGS"))
  (shadow 'SensorYPR (find-package "ROBOTIS_CONTROLLER_MSGS")))
(unless (find-package "ROBOTIS_CONTROLLER_MSGS::SENSORYPR")
  (make-package "ROBOTIS_CONTROLLER_MSGS::SENSORYPR"))

(in-package "ROS")
;;//! \htmlinclude SensorYPR.msg.html


(defclass robotis_controller_msgs::SensorYPR
  :super ros::object
  :slots (_pitch _roll _yaw ))

(defmethod robotis_controller_msgs::SensorYPR
  (:init
   (&key
    ((:pitch __pitch) 0.0)
    ((:roll __roll) 0.0)
    ((:yaw __yaw) 0.0)
    )
   (send-super :init)
   (setq _pitch (float __pitch))
   (setq _roll (float __roll))
   (setq _yaw (float __yaw))
   self)
  (:pitch
   (&optional __pitch)
   (if __pitch (setq _pitch __pitch)) _pitch)
  (:roll
   (&optional __roll)
   (if __roll (setq _roll __roll)) _roll)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:serialization-length
   ()
   (+
    ;; float64 _pitch
    8
    ;; float64 _roll
    8
    ;; float64 _yaw
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _pitch
       (sys::poke _pitch (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _roll
       (sys::poke _roll (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _pitch
     (setq _pitch (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _roll
     (setq _roll (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _yaw
     (setq _yaw (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get robotis_controller_msgs::SensorYPR :md5sum-) "da3691d617cf6ed55be8bea5c4931fd8")
(setf (get robotis_controller_msgs::SensorYPR :datatype-) "robotis_controller_msgs/SensorYPR")
(setf (get robotis_controller_msgs::SensorYPR :definition-)
      "float64 pitch
float64 roll
float64 yaw
")



(provide :robotis_controller_msgs/SensorYPR "da3691d617cf6ed55be8bea5c4931fd8")


