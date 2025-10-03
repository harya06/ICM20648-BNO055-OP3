;; Auto-generated. Do not edit!


(when (boundp 'op3_online_walking_module_msgs::WalkingParam)
  (if (not (find-package "OP3_ONLINE_WALKING_MODULE_MSGS"))
    (make-package "OP3_ONLINE_WALKING_MODULE_MSGS"))
  (shadow 'WalkingParam (find-package "OP3_ONLINE_WALKING_MODULE_MSGS")))
(unless (find-package "OP3_ONLINE_WALKING_MODULE_MSGS::WALKINGPARAM")
  (make-package "OP3_ONLINE_WALKING_MODULE_MSGS::WALKINGPARAM"))

(in-package "ROS")
;;//! \htmlinclude WalkingParam.msg.html


(defclass op3_online_walking_module_msgs::WalkingParam
  :super ros::object
  :slots (_dsp_ratio _lipm_height _foot_height_max _zmp_offset_x _zmp_offset_y ))

(defmethod op3_online_walking_module_msgs::WalkingParam
  (:init
   (&key
    ((:dsp_ratio __dsp_ratio) 0.0)
    ((:lipm_height __lipm_height) 0.0)
    ((:foot_height_max __foot_height_max) 0.0)
    ((:zmp_offset_x __zmp_offset_x) 0.0)
    ((:zmp_offset_y __zmp_offset_y) 0.0)
    )
   (send-super :init)
   (setq _dsp_ratio (float __dsp_ratio))
   (setq _lipm_height (float __lipm_height))
   (setq _foot_height_max (float __foot_height_max))
   (setq _zmp_offset_x (float __zmp_offset_x))
   (setq _zmp_offset_y (float __zmp_offset_y))
   self)
  (:dsp_ratio
   (&optional __dsp_ratio)
   (if __dsp_ratio (setq _dsp_ratio __dsp_ratio)) _dsp_ratio)
  (:lipm_height
   (&optional __lipm_height)
   (if __lipm_height (setq _lipm_height __lipm_height)) _lipm_height)
  (:foot_height_max
   (&optional __foot_height_max)
   (if __foot_height_max (setq _foot_height_max __foot_height_max)) _foot_height_max)
  (:zmp_offset_x
   (&optional __zmp_offset_x)
   (if __zmp_offset_x (setq _zmp_offset_x __zmp_offset_x)) _zmp_offset_x)
  (:zmp_offset_y
   (&optional __zmp_offset_y)
   (if __zmp_offset_y (setq _zmp_offset_y __zmp_offset_y)) _zmp_offset_y)
  (:serialization-length
   ()
   (+
    ;; float64 _dsp_ratio
    8
    ;; float64 _lipm_height
    8
    ;; float64 _foot_height_max
    8
    ;; float64 _zmp_offset_x
    8
    ;; float64 _zmp_offset_y
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _dsp_ratio
       (sys::poke _dsp_ratio (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _lipm_height
       (sys::poke _lipm_height (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _foot_height_max
       (sys::poke _foot_height_max (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _zmp_offset_x
       (sys::poke _zmp_offset_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _zmp_offset_y
       (sys::poke _zmp_offset_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _dsp_ratio
     (setq _dsp_ratio (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _lipm_height
     (setq _lipm_height (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _foot_height_max
     (setq _foot_height_max (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _zmp_offset_x
     (setq _zmp_offset_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _zmp_offset_y
     (setq _zmp_offset_y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get op3_online_walking_module_msgs::WalkingParam :md5sum-) "2197a6ee210540728363ab6b547935bd")
(setf (get op3_online_walking_module_msgs::WalkingParam :datatype-) "op3_online_walking_module_msgs/WalkingParam")
(setf (get op3_online_walking_module_msgs::WalkingParam :definition-)
      "float64 dsp_ratio
float64 lipm_height
float64 foot_height_max
float64 zmp_offset_x
float64 zmp_offset_y

")



(provide :op3_online_walking_module_msgs/WalkingParam "2197a6ee210540728363ab6b547935bd")


