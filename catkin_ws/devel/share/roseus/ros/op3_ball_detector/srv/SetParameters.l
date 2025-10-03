;; Auto-generated. Do not edit!


(when (boundp 'op3_ball_detector::SetParameters)
  (if (not (find-package "OP3_BALL_DETECTOR"))
    (make-package "OP3_BALL_DETECTOR"))
  (shadow 'SetParameters (find-package "OP3_BALL_DETECTOR")))
(unless (find-package "OP3_BALL_DETECTOR::SETPARAMETERS")
  (make-package "OP3_BALL_DETECTOR::SETPARAMETERS"))
(unless (find-package "OP3_BALL_DETECTOR::SETPARAMETERSREQUEST")
  (make-package "OP3_BALL_DETECTOR::SETPARAMETERSREQUEST"))
(unless (find-package "OP3_BALL_DETECTOR::SETPARAMETERSRESPONSE")
  (make-package "OP3_BALL_DETECTOR::SETPARAMETERSRESPONSE"))

(in-package "ROS")





(defclass op3_ball_detector::SetParametersRequest
  :super ros::object
  :slots (_params ))

(defmethod op3_ball_detector::SetParametersRequest
  (:init
   (&key
    ((:params __params) (instance op3_ball_detector::BallDetectorParams :init))
    )
   (send-super :init)
   (setq _params __params)
   self)
  (:params
   (&rest __params)
   (if (keywordp (car __params))
       (send* _params __params)
     (progn
       (if __params (setq _params (car __params)))
       _params)))
  (:serialization-length
   ()
   (+
    ;; op3_ball_detector/BallDetectorParams _params
    (send _params :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; op3_ball_detector/BallDetectorParams _params
       (send _params :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; op3_ball_detector/BallDetectorParams _params
     (send _params :deserialize buf ptr-) (incf ptr- (send _params :serialization-length))
   ;;
   self)
  )

(defclass op3_ball_detector::SetParametersResponse
  :super ros::object
  :slots (_returns ))

(defmethod op3_ball_detector::SetParametersResponse
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

(defclass op3_ball_detector::SetParameters
  :super ros::object
  :slots ())

(setf (get op3_ball_detector::SetParameters :md5sum-) "b144be2bfcda348236c05dcc43378583")
(setf (get op3_ball_detector::SetParameters :datatype-) "op3_ball_detector/SetParameters")
(setf (get op3_ball_detector::SetParameters :request) op3_ball_detector::SetParametersRequest)
(setf (get op3_ball_detector::SetParameters :response) op3_ball_detector::SetParametersResponse)

(defmethod op3_ball_detector::SetParametersRequest
  (:response () (instance op3_ball_detector::SetParametersResponse :init)))

(setf (get op3_ball_detector::SetParametersRequest :md5sum-) "b144be2bfcda348236c05dcc43378583")
(setf (get op3_ball_detector::SetParametersRequest :datatype-) "op3_ball_detector/SetParametersRequest")
(setf (get op3_ball_detector::SetParametersRequest :definition-)
      "BallDetectorParams  params

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

(setf (get op3_ball_detector::SetParametersResponse :md5sum-) "b144be2bfcda348236c05dcc43378583")
(setf (get op3_ball_detector::SetParametersResponse :datatype-) "op3_ball_detector/SetParametersResponse")
(setf (get op3_ball_detector::SetParametersResponse :definition-)
      "BallDetectorParams  params

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



(provide :op3_ball_detector/SetParameters "b144be2bfcda348236c05dcc43378583")


