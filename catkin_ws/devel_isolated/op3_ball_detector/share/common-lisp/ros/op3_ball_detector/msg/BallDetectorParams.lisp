; Auto-generated. Do not edit!


(cl:in-package op3_ball_detector-msg)


;//! \htmlinclude BallDetectorParams.msg.html

(cl:defclass <BallDetectorParams> (roslisp-msg-protocol:ros-message)
  ((gaussian_blur_size
    :reader gaussian_blur_size
    :initarg :gaussian_blur_size
    :type cl:integer
    :initform 0)
   (gaussian_blur_sigma
    :reader gaussian_blur_sigma
    :initarg :gaussian_blur_sigma
    :type cl:float
    :initform 0.0)
   (canny_edge_th
    :reader canny_edge_th
    :initarg :canny_edge_th
    :type cl:float
    :initform 0.0)
   (hough_accum_resolution
    :reader hough_accum_resolution
    :initarg :hough_accum_resolution
    :type cl:float
    :initform 0.0)
   (hough_accum_th
    :reader hough_accum_th
    :initarg :hough_accum_th
    :type cl:float
    :initform 0.0)
   (min_circle_dist
    :reader min_circle_dist
    :initarg :min_circle_dist
    :type cl:float
    :initform 0.0)
   (min_radius
    :reader min_radius
    :initarg :min_radius
    :type cl:integer
    :initform 0)
   (max_radius
    :reader max_radius
    :initarg :max_radius
    :type cl:integer
    :initform 0)
   (filter_h_min
    :reader filter_h_min
    :initarg :filter_h_min
    :type cl:integer
    :initform 0)
   (filter_h_max
    :reader filter_h_max
    :initarg :filter_h_max
    :type cl:integer
    :initform 0)
   (filter_s_min
    :reader filter_s_min
    :initarg :filter_s_min
    :type cl:integer
    :initform 0)
   (filter_s_max
    :reader filter_s_max
    :initarg :filter_s_max
    :type cl:integer
    :initform 0)
   (filter_v_min
    :reader filter_v_min
    :initarg :filter_v_min
    :type cl:integer
    :initform 0)
   (filter_v_max
    :reader filter_v_max
    :initarg :filter_v_max
    :type cl:integer
    :initform 0)
   (ellipse_size
    :reader ellipse_size
    :initarg :ellipse_size
    :type cl:integer
    :initform 0))
)

(cl:defclass BallDetectorParams (<BallDetectorParams>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BallDetectorParams>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BallDetectorParams)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name op3_ball_detector-msg:<BallDetectorParams> is deprecated: use op3_ball_detector-msg:BallDetectorParams instead.")))

(cl:ensure-generic-function 'gaussian_blur_size-val :lambda-list '(m))
(cl:defmethod gaussian_blur_size-val ((m <BallDetectorParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_ball_detector-msg:gaussian_blur_size-val is deprecated.  Use op3_ball_detector-msg:gaussian_blur_size instead.")
  (gaussian_blur_size m))

(cl:ensure-generic-function 'gaussian_blur_sigma-val :lambda-list '(m))
(cl:defmethod gaussian_blur_sigma-val ((m <BallDetectorParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_ball_detector-msg:gaussian_blur_sigma-val is deprecated.  Use op3_ball_detector-msg:gaussian_blur_sigma instead.")
  (gaussian_blur_sigma m))

(cl:ensure-generic-function 'canny_edge_th-val :lambda-list '(m))
(cl:defmethod canny_edge_th-val ((m <BallDetectorParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_ball_detector-msg:canny_edge_th-val is deprecated.  Use op3_ball_detector-msg:canny_edge_th instead.")
  (canny_edge_th m))

(cl:ensure-generic-function 'hough_accum_resolution-val :lambda-list '(m))
(cl:defmethod hough_accum_resolution-val ((m <BallDetectorParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_ball_detector-msg:hough_accum_resolution-val is deprecated.  Use op3_ball_detector-msg:hough_accum_resolution instead.")
  (hough_accum_resolution m))

(cl:ensure-generic-function 'hough_accum_th-val :lambda-list '(m))
(cl:defmethod hough_accum_th-val ((m <BallDetectorParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_ball_detector-msg:hough_accum_th-val is deprecated.  Use op3_ball_detector-msg:hough_accum_th instead.")
  (hough_accum_th m))

(cl:ensure-generic-function 'min_circle_dist-val :lambda-list '(m))
(cl:defmethod min_circle_dist-val ((m <BallDetectorParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_ball_detector-msg:min_circle_dist-val is deprecated.  Use op3_ball_detector-msg:min_circle_dist instead.")
  (min_circle_dist m))

(cl:ensure-generic-function 'min_radius-val :lambda-list '(m))
(cl:defmethod min_radius-val ((m <BallDetectorParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_ball_detector-msg:min_radius-val is deprecated.  Use op3_ball_detector-msg:min_radius instead.")
  (min_radius m))

(cl:ensure-generic-function 'max_radius-val :lambda-list '(m))
(cl:defmethod max_radius-val ((m <BallDetectorParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_ball_detector-msg:max_radius-val is deprecated.  Use op3_ball_detector-msg:max_radius instead.")
  (max_radius m))

(cl:ensure-generic-function 'filter_h_min-val :lambda-list '(m))
(cl:defmethod filter_h_min-val ((m <BallDetectorParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_ball_detector-msg:filter_h_min-val is deprecated.  Use op3_ball_detector-msg:filter_h_min instead.")
  (filter_h_min m))

(cl:ensure-generic-function 'filter_h_max-val :lambda-list '(m))
(cl:defmethod filter_h_max-val ((m <BallDetectorParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_ball_detector-msg:filter_h_max-val is deprecated.  Use op3_ball_detector-msg:filter_h_max instead.")
  (filter_h_max m))

(cl:ensure-generic-function 'filter_s_min-val :lambda-list '(m))
(cl:defmethod filter_s_min-val ((m <BallDetectorParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_ball_detector-msg:filter_s_min-val is deprecated.  Use op3_ball_detector-msg:filter_s_min instead.")
  (filter_s_min m))

(cl:ensure-generic-function 'filter_s_max-val :lambda-list '(m))
(cl:defmethod filter_s_max-val ((m <BallDetectorParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_ball_detector-msg:filter_s_max-val is deprecated.  Use op3_ball_detector-msg:filter_s_max instead.")
  (filter_s_max m))

(cl:ensure-generic-function 'filter_v_min-val :lambda-list '(m))
(cl:defmethod filter_v_min-val ((m <BallDetectorParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_ball_detector-msg:filter_v_min-val is deprecated.  Use op3_ball_detector-msg:filter_v_min instead.")
  (filter_v_min m))

(cl:ensure-generic-function 'filter_v_max-val :lambda-list '(m))
(cl:defmethod filter_v_max-val ((m <BallDetectorParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_ball_detector-msg:filter_v_max-val is deprecated.  Use op3_ball_detector-msg:filter_v_max instead.")
  (filter_v_max m))

(cl:ensure-generic-function 'ellipse_size-val :lambda-list '(m))
(cl:defmethod ellipse_size-val ((m <BallDetectorParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_ball_detector-msg:ellipse_size-val is deprecated.  Use op3_ball_detector-msg:ellipse_size instead.")
  (ellipse_size m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BallDetectorParams>) ostream)
  "Serializes a message object of type '<BallDetectorParams>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gaussian_blur_size)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'gaussian_blur_size)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'gaussian_blur_size)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'gaussian_blur_size)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'gaussian_blur_sigma))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'canny_edge_th))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'hough_accum_resolution))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'hough_accum_th))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'min_circle_dist))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'min_radius)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'min_radius)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'min_radius)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'min_radius)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'max_radius)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'max_radius)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'max_radius)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'max_radius)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'filter_h_min)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'filter_h_min)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'filter_h_min)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'filter_h_min)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'filter_h_max)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'filter_h_max)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'filter_h_max)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'filter_h_max)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'filter_s_min)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'filter_s_min)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'filter_s_min)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'filter_s_min)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'filter_s_max)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'filter_s_max)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'filter_s_max)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'filter_s_max)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'filter_v_min)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'filter_v_min)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'filter_v_min)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'filter_v_min)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'filter_v_max)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'filter_v_max)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'filter_v_max)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'filter_v_max)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ellipse_size)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ellipse_size)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'ellipse_size)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'ellipse_size)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BallDetectorParams>) istream)
  "Deserializes a message object of type '<BallDetectorParams>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gaussian_blur_size)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'gaussian_blur_size)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'gaussian_blur_size)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'gaussian_blur_size)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'gaussian_blur_sigma) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'canny_edge_th) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'hough_accum_resolution) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'hough_accum_th) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'min_circle_dist) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'min_radius)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'min_radius)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'min_radius)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'min_radius)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'max_radius)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'max_radius)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'max_radius)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'max_radius)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'filter_h_min)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'filter_h_min)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'filter_h_min)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'filter_h_min)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'filter_h_max)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'filter_h_max)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'filter_h_max)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'filter_h_max)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'filter_s_min)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'filter_s_min)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'filter_s_min)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'filter_s_min)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'filter_s_max)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'filter_s_max)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'filter_s_max)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'filter_s_max)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'filter_v_min)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'filter_v_min)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'filter_v_min)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'filter_v_min)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'filter_v_max)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'filter_v_max)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'filter_v_max)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'filter_v_max)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ellipse_size)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ellipse_size)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'ellipse_size)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'ellipse_size)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BallDetectorParams>)))
  "Returns string type for a message object of type '<BallDetectorParams>"
  "op3_ball_detector/BallDetectorParams")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BallDetectorParams)))
  "Returns string type for a message object of type 'BallDetectorParams"
  "op3_ball_detector/BallDetectorParams")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BallDetectorParams>)))
  "Returns md5sum for a message object of type '<BallDetectorParams>"
  "23a4c471d4f161f32a80066aac78a96c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BallDetectorParams)))
  "Returns md5sum for a message object of type 'BallDetectorParams"
  "23a4c471d4f161f32a80066aac78a96c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BallDetectorParams>)))
  "Returns full string definition for message of type '<BallDetectorParams>"
  (cl:format cl:nil "# This represents the parameters of ball_detector~%~%uint32  gaussian_blur_size      # only odd number, 1 - 11~%float32 gaussian_blur_sigma     # 1 - 5~%float32 canny_edge_th           # 50 - 200~%float32 hough_accum_resolution  # 1 - 8~%float32 hough_accum_th          # 10 - 200~%float32 min_circle_dist         # 10 - 200~%uint32  min_radius              # 10 - 200~%uint32  max_radius              # 100 - 600~%uint32  filter_h_min            # 0 - 359~%uint32  filter_h_max~%uint32  filter_s_min            # 0 - 255~%uint32  filter_s_max~%uint32  filter_v_min            # 0 - 255~%uint32  filter_v_max~%uint32  ellipse_size            # 1 - 9~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BallDetectorParams)))
  "Returns full string definition for message of type 'BallDetectorParams"
  (cl:format cl:nil "# This represents the parameters of ball_detector~%~%uint32  gaussian_blur_size      # only odd number, 1 - 11~%float32 gaussian_blur_sigma     # 1 - 5~%float32 canny_edge_th           # 50 - 200~%float32 hough_accum_resolution  # 1 - 8~%float32 hough_accum_th          # 10 - 200~%float32 min_circle_dist         # 10 - 200~%uint32  min_radius              # 10 - 200~%uint32  max_radius              # 100 - 600~%uint32  filter_h_min            # 0 - 359~%uint32  filter_h_max~%uint32  filter_s_min            # 0 - 255~%uint32  filter_s_max~%uint32  filter_v_min            # 0 - 255~%uint32  filter_v_max~%uint32  ellipse_size            # 1 - 9~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BallDetectorParams>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BallDetectorParams>))
  "Converts a ROS message object to a list"
  (cl:list 'BallDetectorParams
    (cl:cons ':gaussian_blur_size (gaussian_blur_size msg))
    (cl:cons ':gaussian_blur_sigma (gaussian_blur_sigma msg))
    (cl:cons ':canny_edge_th (canny_edge_th msg))
    (cl:cons ':hough_accum_resolution (hough_accum_resolution msg))
    (cl:cons ':hough_accum_th (hough_accum_th msg))
    (cl:cons ':min_circle_dist (min_circle_dist msg))
    (cl:cons ':min_radius (min_radius msg))
    (cl:cons ':max_radius (max_radius msg))
    (cl:cons ':filter_h_min (filter_h_min msg))
    (cl:cons ':filter_h_max (filter_h_max msg))
    (cl:cons ':filter_s_min (filter_s_min msg))
    (cl:cons ':filter_s_max (filter_s_max msg))
    (cl:cons ':filter_v_min (filter_v_min msg))
    (cl:cons ':filter_v_max (filter_v_max msg))
    (cl:cons ':ellipse_size (ellipse_size msg))
))
