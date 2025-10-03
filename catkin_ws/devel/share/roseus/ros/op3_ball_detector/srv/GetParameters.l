;; Auto-generated. Do not edit!


(when (boundp 'op3_ball_detector::GetParameters)
  (if (not (find-package "OP3_BALL_DETECTOR"))
    (make-package "OP3_BALL_DETECTOR"))
  (shadow 'GetParameters (find-package "OP3_BALL_DETECTOR")))
(unless (find-package "OP3_BALL_DETECTOR::GETPARAMETERS")
  (make-package "OP3_BALL_DETECTOR::GETPARAMETERS"))
(unless (find-package "OP3_BALL_DETECTOR::GETPARAMETERSREQUEST")
  (make-package "OP3_BALL_DETECTOR::GETPARAMETERSREQUEST"))
(unless (find-package "OP3_BALL_DETECTOR::GETPARAMETERSRESPONSE")
  (make-package "OP3_BALL_DETECTOR::GETPARAMETERSRESPONSE"))

(in-package "ROS")





(defclass op3_ball_detector::GetParametersRequest
  :super ros::object
  :slots ())

(defmethod op3_ball_detector::GetParametersRequest
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

(defclass op3_ball_detector::GetParametersResponse
  :super ros::object
  :slots (_returns ))

(defmethod op3_ball_detector::GetParametersResponse
  (:init
   (&key
    ((:returns __returns) (instance op3_ball_detector::BallDetectorParams :init))
    )
   (send-super :init)
   (setq _returns __returns)
   self)
  (:returns
   (&rest __returns)
   (if (keywordp (car __returns))
       (send* _returns __returns)
     (progn
       (if __returns (setq _returns (car __returns)))
       _returns)))
  (:serialization-length
   ()
   (+
    ;; op3_ball_detector/BallDetectorParams _returns
    (send _returns :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; op3_ball_detector/BallDetectorParams _returns
       (send _returns :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; op3_ball_detector/BallDetectorParams _returns
     (send _returns :deserialize buf ptr-) (incf ptr- (send _returns :serialization-length))
   ;;
   self)
  )

(defclass op3_ball_detector::GetParameters
  :super ros::object
  :slots ())

(setf (get op3_ball_detector::GetParameters :md5sum-) "17229f06a68a0d87fd525e4d8bdaee90")
(setf (get op3_ball_detector::GetParameters :datatype-) "op3_ball_detector/GetParameters")
(setf (get op3_ball_detector::GetParameters :request) op3_ball_detector::GetParametersRequest)
(setf (get op3_ball_detector::GetParameters :response) op3_ball_detector::GetParametersResponse)

(defmethod op3_ball_detector::GetParametersRequest
  (:response () (instance op3_ball_detector::GetParametersResponse :init)))

(setf (get op3_ball_detector::GetParametersRequest :md5sum-) "17229f06a68a0d87fd525e4d8bdaee90")
(setf (get op3_ball_detector::GetParametersRequest :datatype-) "op3_ball_detector/GetParametersRequest")
(setf (get op3_ball_detector::GetParametersRequest :definition-)
      "
---
BallDetectorParams  returns


================================================================================
MSG: op3_ball_detector/BallDetectorParams
# This represents the parameters of ball_detector

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

(setf (get op3_ball_detector::GetParametersResponse :md5sum-) "17229f06a68a0d87fd525e4d8bdaee90")
(setf (get op3_ball_detector::GetParametersResponse :datatype-) "op3_ball_detector/GetParametersResponse")
(setf (get op3_ball_detector::GetParametersResponse :definition-)
      "
---
BallDetectorParams  returns


================================================================================
MSG: op3_ball_detector/BallDetectorParams
# This represents the parameters of ball_detector

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



(provide :op3_ball_detector/GetParameters "17229f06a68a0d87fd525e4d8bdaee90")


