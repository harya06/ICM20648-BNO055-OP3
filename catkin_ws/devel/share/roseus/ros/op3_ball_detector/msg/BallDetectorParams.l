;; Auto-generated. Do not edit!


(when (boundp 'op3_ball_detector::BallDetectorParams)
  (if (not (find-package "OP3_BALL_DETECTOR"))
    (make-package "OP3_BALL_DETECTOR"))
  (shadow 'BallDetectorParams (find-package "OP3_BALL_DETECTOR")))
(unless (find-package "OP3_BALL_DETECTOR::BALLDETECTORPARAMS")
  (make-package "OP3_BALL_DETECTOR::BALLDETECTORPARAMS"))

(in-package "ROS")
;;//! \htmlinclude BallDetectorParams.msg.html


(defclass op3_ball_detector::BallDetectorParams
  :super ros::object
  :slots (_gaussian_blur_size _gaussian_blur_sigma _canny_edge_th _hough_accum_resolution _hough_accum_th _min_circle_dist _min_radius _max_radius _filter_h_min _filter_h_max _filter_s_min _filter_s_max _filter_v_min _filter_v_max _ellipse_size ))

(defmethod op3_ball_detector::BallDetectorParams
  (:init
   (&key
    ((:gaussian_blur_size __gaussian_blur_size) 0)
    ((:gaussian_blur_sigma __gaussian_blur_sigma) 0.0)
    ((:canny_edge_th __canny_edge_th) 0.0)
    ((:hough_accum_resolution __hough_accum_resolution) 0.0)
    ((:hough_accum_th __hough_accum_th) 0.0)
    ((:min_circle_dist __min_circle_dist) 0.0)
    ((:min_radius __min_radius) 0)
    ((:max_radius __max_radius) 0)
    ((:filter_h_min __filter_h_min) 0)
    ((:filter_h_max __filter_h_max) 0)
    ((:filter_s_min __filter_s_min) 0)
    ((:filter_s_max __filter_s_max) 0)
    ((:filter_v_min __filter_v_min) 0)
    ((:filter_v_max __filter_v_max) 0)
    ((:ellipse_size __ellipse_size) 0)
    )
   (send-super :init)
   (setq _gaussian_blur_size (round __gaussian_blur_size))
   (setq _gaussian_blur_sigma (float __gaussian_blur_sigma))
   (setq _canny_edge_th (float __canny_edge_th))
   (setq _hough_accum_resolution (float __hough_accum_resolution))
   (setq _hough_accum_th (float __hough_accum_th))
   (setq _min_circle_dist (float __min_circle_dist))
   (setq _min_radius (round __min_radius))
   (setq _max_radius (round __max_radius))
   (setq _filter_h_min (round __filter_h_min))
   (setq _filter_h_max (round __filter_h_max))
   (setq _filter_s_min (round __filter_s_min))
   (setq _filter_s_max (round __filter_s_max))
   (setq _filter_v_min (round __filter_v_min))
   (setq _filter_v_max (round __filter_v_max))
   (setq _ellipse_size (round __ellipse_size))
   self)
  (:gaussian_blur_size
   (&optional __gaussian_blur_size)
   (if __gaussian_blur_size (setq _gaussian_blur_size __gaussian_blur_size)) _gaussian_blur_size)
  (:gaussian_blur_sigma
   (&optional __gaussian_blur_sigma)
   (if __gaussian_blur_sigma (setq _gaussian_blur_sigma __gaussian_blur_sigma)) _gaussian_blur_sigma)
  (:canny_edge_th
   (&optional __canny_edge_th)
   (if __canny_edge_th (setq _canny_edge_th __canny_edge_th)) _canny_edge_th)
  (:hough_accum_resolution
   (&optional __hough_accum_resolution)
   (if __hough_accum_resolution (setq _hough_accum_resolution __hough_accum_resolution)) _hough_accum_resolution)
  (:hough_accum_th
   (&optional __hough_accum_th)
   (if __hough_accum_th (setq _hough_accum_th __hough_accum_th)) _hough_accum_th)
  (:min_circle_dist
   (&optional __min_circle_dist)
   (if __min_circle_dist (setq _min_circle_dist __min_circle_dist)) _min_circle_dist)
  (:min_radius
   (&optional __min_radius)
   (if __min_radius (setq _min_radius __min_radius)) _min_radius)
  (:max_radius
   (&optional __max_radius)
   (if __max_radius (setq _max_radius __max_radius)) _max_radius)
  (:filter_h_min
   (&optional __filter_h_min)
   (if __filter_h_min (setq _filter_h_min __filter_h_min)) _filter_h_min)
  (:filter_h_max
   (&optional __filter_h_max)
   (if __filter_h_max (setq _filter_h_max __filter_h_max)) _filter_h_max)
  (:filter_s_min
   (&optional __filter_s_min)
   (if __filter_s_min (setq _filter_s_min __filter_s_min)) _filter_s_min)
  (:filter_s_max
   (&optional __filter_s_max)
   (if __filter_s_max (setq _filter_s_max __filter_s_max)) _filter_s_max)
  (:filter_v_min
   (&optional __filter_v_min)
   (if __filter_v_min (setq _filter_v_min __filter_v_min)) _filter_v_min)
  (:filter_v_max
   (&optional __filter_v_max)
   (if __filter_v_max (setq _filter_v_max __filter_v_max)) _filter_v_max)
  (:ellipse_size
   (&optional __ellipse_size)
   (if __ellipse_size (setq _ellipse_size __ellipse_size)) _ellipse_size)
  (:serialization-length
   ()
   (+
    ;; uint32 _gaussian_blur_size
    4
    ;; float32 _gaussian_blur_sigma
    4
    ;; float32 _canny_edge_th
    4
    ;; float32 _hough_accum_resolution
    4
    ;; float32 _hough_accum_th
    4
    ;; float32 _min_circle_dist
    4
    ;; uint32 _min_radius
    4
    ;; uint32 _max_radius
    4
    ;; uint32 _filter_h_min
    4
    ;; uint32 _filter_h_max
    4
    ;; uint32 _filter_s_min
    4
    ;; uint32 _filter_s_max
    4
    ;; uint32 _filter_v_min
    4
    ;; uint32 _filter_v_max
    4
    ;; uint32 _ellipse_size
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _gaussian_blur_size
       (write-long _gaussian_blur_size s)
     ;; float32 _gaussian_blur_sigma
       (sys::poke _gaussian_blur_sigma (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _canny_edge_th
       (sys::poke _canny_edge_th (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _hough_accum_resolution
       (sys::poke _hough_accum_resolution (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _hough_accum_th
       (sys::poke _hough_accum_th (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _min_circle_dist
       (sys::poke _min_circle_dist (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint32 _min_radius
       (write-long _min_radius s)
     ;; uint32 _max_radius
       (write-long _max_radius s)
     ;; uint32 _filter_h_min
       (write-long _filter_h_min s)
     ;; uint32 _filter_h_max
       (write-long _filter_h_max s)
     ;; uint32 _filter_s_min
       (write-long _filter_s_min s)
     ;; uint32 _filter_s_max
       (write-long _filter_s_max s)
     ;; uint32 _filter_v_min
       (write-long _filter_v_min s)
     ;; uint32 _filter_v_max
       (write-long _filter_v_max s)
     ;; uint32 _ellipse_size
       (write-long _ellipse_size s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _gaussian_blur_size
     (setq _gaussian_blur_size (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _gaussian_blur_sigma
     (setq _gaussian_blur_sigma (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _canny_edge_th
     (setq _canny_edge_th (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _hough_accum_resolution
     (setq _hough_accum_resolution (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _hough_accum_th
     (setq _hough_accum_th (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _min_circle_dist
     (setq _min_circle_dist (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint32 _min_radius
     (setq _min_radius (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _max_radius
     (setq _max_radius (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _filter_h_min
     (setq _filter_h_min (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _filter_h_max
     (setq _filter_h_max (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _filter_s_min
     (setq _filter_s_min (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _filter_s_max
     (setq _filter_s_max (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _filter_v_min
     (setq _filter_v_min (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _filter_v_max
     (setq _filter_v_max (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _ellipse_size
     (setq _ellipse_size (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get op3_ball_detector::BallDetectorParams :md5sum-) "23a4c471d4f161f32a80066aac78a96c")
(setf (get op3_ball_detector::BallDetectorParams :datatype-) "op3_ball_detector/BallDetectorParams")
(setf (get op3_ball_detector::BallDetectorParams :definition-)
      "# This represents the parameters of ball_detector

uint32  gaussian_blur_size      # only odd number, 1 - 11
float32 gaussian_blur_sigma     # 1 - 5
float32 canny_edge_th           # 50 - 200
float32 hough_accum_resolution  # 1 - 8
float32 hough_accum_th          # 10 - 200
float32 min_circle_dist         # 10 - 200
uint32  min_radius              # 10 - 200
uint32  max_radius              # 100 - 600
uint32  filter_h_min            # 0 - 359
uint32  filter_h_max
uint32  filter_s_min            # 0 - 255
uint32  filter_s_max
uint32  filter_v_min            # 0 - 255
uint32  filter_v_max
uint32  ellipse_size            # 1 - 9

")



(provide :op3_ball_detector/BallDetectorParams "23a4c471d4f161f32a80066aac78a96c")


