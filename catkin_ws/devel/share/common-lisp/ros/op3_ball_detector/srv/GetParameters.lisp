; Auto-generated. Do not edit!


(cl:in-package op3_ball_detector-srv)


;//! \htmlinclude GetParameters-request.msg.html

(cl:defclass <GetParameters-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetParameters-request (<GetParameters-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetParameters-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetParameters-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name op3_ball_detector-srv:<GetParameters-request> is deprecated: use op3_ball_detector-srv:GetParameters-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetParameters-request>) ostream)
  "Serializes a message object of type '<GetParameters-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetParameters-request>) istream)
  "Deserializes a message object of type '<GetParameters-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetParameters-request>)))
  "Returns string type for a service object of type '<GetParameters-request>"
  "op3_ball_detector/GetParametersRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetParameters-request)))
  "Returns string type for a service object of type 'GetParameters-request"
  "op3_ball_detector/GetParametersRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetParameters-request>)))
  "Returns md5sum for a message object of type '<GetParameters-request>"
  "17229f06a68a0d87fd525e4d8bdaee90")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetParameters-request)))
  "Returns md5sum for a message object of type 'GetParameters-request"
  "17229f06a68a0d87fd525e4d8bdaee90")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetParameters-request>)))
  "Returns full string definition for message of type '<GetParameters-request>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetParameters-request)))
  "Returns full string definition for message of type 'GetParameters-request"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetParameters-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetParameters-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetParameters-request
))
;//! \htmlinclude GetParameters-response.msg.html

(cl:defclass <GetParameters-response> (roslisp-msg-protocol:ros-message)
  ((returns
    :reader returns
    :initarg :returns
    :type op3_ball_detector-msg:BallDetectorParams
    :initform (cl:make-instance 'op3_ball_detector-msg:BallDetectorParams)))
)

(cl:defclass GetParameters-response (<GetParameters-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetParameters-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetParameters-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name op3_ball_detector-srv:<GetParameters-response> is deprecated: use op3_ball_detector-srv:GetParameters-response instead.")))

(cl:ensure-generic-function 'returns-val :lambda-list '(m))
(cl:defmethod returns-val ((m <GetParameters-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_ball_detector-srv:returns-val is deprecated.  Use op3_ball_detector-srv:returns instead.")
  (returns m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetParameters-response>) ostream)
  "Serializes a message object of type '<GetParameters-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'returns) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetParameters-response>) istream)
  "Deserializes a message object of type '<GetParameters-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'returns) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetParameters-response>)))
  "Returns string type for a service object of type '<GetParameters-response>"
  "op3_ball_detector/GetParametersResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetParameters-response)))
  "Returns string type for a service object of type 'GetParameters-response"
  "op3_ball_detector/GetParametersResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetParameters-response>)))
  "Returns md5sum for a message object of type '<GetParameters-response>"
  "17229f06a68a0d87fd525e4d8bdaee90")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetParameters-response)))
  "Returns md5sum for a message object of type 'GetParameters-response"
  "17229f06a68a0d87fd525e4d8bdaee90")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetParameters-response>)))
  "Returns full string definition for message of type '<GetParameters-response>"
  (cl:format cl:nil "BallDetectorParams  returns~%~%~%================================================================================~%MSG: op3_ball_detector/BallDetectorParams~%# This represents the parameters of ball_detector~%~%uint32  gaussian_blur_size      # only odd number, 1 - 11~%float32 gaussian_blur_sigma     # 1 - 5~%float32 canny_edge_th           # 50 - 200~%float32 hough_accum_resolution  # 1 - 8~%float32 hough_accum_th          # 10 - 200~%float32 min_circle_dist         # 10 - 200~%uint32  min_radius              # 10 - 200~%uint32  max_radius              # 100 - 600~%uint32  filter_h_min            # 0 - 359~%uint32  filter_h_max~%uint32  filter_s_min            # 0 - 255~%uint32  filter_s_max~%uint32  filter_v_min            # 0 - 255~%uint32  filter_v_max~%uint32  ellipse_size            # 1 - 9~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetParameters-response)))
  "Returns full string definition for message of type 'GetParameters-response"
  (cl:format cl:nil "BallDetectorParams  returns~%~%~%================================================================================~%MSG: op3_ball_detector/BallDetectorParams~%# This represents the parameters of ball_detector~%~%uint32  gaussian_blur_size      # only odd number, 1 - 11~%float32 gaussian_blur_sigma     # 1 - 5~%float32 canny_edge_th           # 50 - 200~%float32 hough_accum_resolution  # 1 - 8~%float32 hough_accum_th          # 10 - 200~%float32 min_circle_dist         # 10 - 200~%uint32  min_radius              # 10 - 200~%uint32  max_radius              # 100 - 600~%uint32  filter_h_min            # 0 - 359~%uint32  filter_h_max~%uint32  filter_s_min            # 0 - 255~%uint32  filter_s_max~%uint32  filter_v_min            # 0 - 255~%uint32  filter_v_max~%uint32  ellipse_size            # 1 - 9~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetParameters-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'returns))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetParameters-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetParameters-response
    (cl:cons ':returns (returns msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetParameters)))
  'GetParameters-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetParameters)))
  'GetParameters-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetParameters)))
  "Returns string type for a service object of type '<GetParameters>"
  "op3_ball_detector/GetParameters")