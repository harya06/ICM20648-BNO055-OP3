; Auto-generated. Do not edit!


(cl:in-package op3_online_walking_module_msgs-srv)


;//! \htmlinclude GetPreviewMatrix-request.msg.html

(cl:defclass <GetPreviewMatrix-request> (roslisp-msg-protocol:ros-message)
  ((req
    :reader req
    :initarg :req
    :type op3_online_walking_module_msgs-msg:PreviewRequest
    :initform (cl:make-instance 'op3_online_walking_module_msgs-msg:PreviewRequest)))
)

(cl:defclass GetPreviewMatrix-request (<GetPreviewMatrix-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetPreviewMatrix-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetPreviewMatrix-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name op3_online_walking_module_msgs-srv:<GetPreviewMatrix-request> is deprecated: use op3_online_walking_module_msgs-srv:GetPreviewMatrix-request instead.")))

(cl:ensure-generic-function 'req-val :lambda-list '(m))
(cl:defmethod req-val ((m <GetPreviewMatrix-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_online_walking_module_msgs-srv:req-val is deprecated.  Use op3_online_walking_module_msgs-srv:req instead.")
  (req m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetPreviewMatrix-request>) ostream)
  "Serializes a message object of type '<GetPreviewMatrix-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'req) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetPreviewMatrix-request>) istream)
  "Deserializes a message object of type '<GetPreviewMatrix-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'req) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetPreviewMatrix-request>)))
  "Returns string type for a service object of type '<GetPreviewMatrix-request>"
  "op3_online_walking_module_msgs/GetPreviewMatrixRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPreviewMatrix-request)))
  "Returns string type for a service object of type 'GetPreviewMatrix-request"
  "op3_online_walking_module_msgs/GetPreviewMatrixRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetPreviewMatrix-request>)))
  "Returns md5sum for a message object of type '<GetPreviewMatrix-request>"
  "353f820108e54e41d7780433bc38ff53")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetPreviewMatrix-request)))
  "Returns md5sum for a message object of type 'GetPreviewMatrix-request"
  "353f820108e54e41d7780433bc38ff53")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetPreviewMatrix-request>)))
  "Returns full string definition for message of type '<GetPreviewMatrix-request>"
  (cl:format cl:nil "op3_online_walking_module_msgs/PreviewRequest req~%~%================================================================================~%MSG: op3_online_walking_module_msgs/PreviewRequest~%float64 control_cycle~%float64 lipm_height~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetPreviewMatrix-request)))
  "Returns full string definition for message of type 'GetPreviewMatrix-request"
  (cl:format cl:nil "op3_online_walking_module_msgs/PreviewRequest req~%~%================================================================================~%MSG: op3_online_walking_module_msgs/PreviewRequest~%float64 control_cycle~%float64 lipm_height~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetPreviewMatrix-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'req))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetPreviewMatrix-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetPreviewMatrix-request
    (cl:cons ':req (req msg))
))
;//! \htmlinclude GetPreviewMatrix-response.msg.html

(cl:defclass <GetPreviewMatrix-response> (roslisp-msg-protocol:ros-message)
  ((res
    :reader res
    :initarg :res
    :type op3_online_walking_module_msgs-msg:PreviewResponse
    :initform (cl:make-instance 'op3_online_walking_module_msgs-msg:PreviewResponse)))
)

(cl:defclass GetPreviewMatrix-response (<GetPreviewMatrix-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetPreviewMatrix-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetPreviewMatrix-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name op3_online_walking_module_msgs-srv:<GetPreviewMatrix-response> is deprecated: use op3_online_walking_module_msgs-srv:GetPreviewMatrix-response instead.")))

(cl:ensure-generic-function 'res-val :lambda-list '(m))
(cl:defmethod res-val ((m <GetPreviewMatrix-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_online_walking_module_msgs-srv:res-val is deprecated.  Use op3_online_walking_module_msgs-srv:res instead.")
  (res m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetPreviewMatrix-response>) ostream)
  "Serializes a message object of type '<GetPreviewMatrix-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'res) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetPreviewMatrix-response>) istream)
  "Deserializes a message object of type '<GetPreviewMatrix-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'res) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetPreviewMatrix-response>)))
  "Returns string type for a service object of type '<GetPreviewMatrix-response>"
  "op3_online_walking_module_msgs/GetPreviewMatrixResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPreviewMatrix-response)))
  "Returns string type for a service object of type 'GetPreviewMatrix-response"
  "op3_online_walking_module_msgs/GetPreviewMatrixResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetPreviewMatrix-response>)))
  "Returns md5sum for a message object of type '<GetPreviewMatrix-response>"
  "353f820108e54e41d7780433bc38ff53")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetPreviewMatrix-response)))
  "Returns md5sum for a message object of type 'GetPreviewMatrix-response"
  "353f820108e54e41d7780433bc38ff53")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetPreviewMatrix-response>)))
  "Returns full string definition for message of type '<GetPreviewMatrix-response>"
  (cl:format cl:nil "op3_online_walking_module_msgs/PreviewResponse res~%~%~%================================================================================~%MSG: op3_online_walking_module_msgs/PreviewResponse~%int32 K_row~%int32 K_col~%float64[] K~%int32 P_row~%int32 P_col~%float64[] P~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetPreviewMatrix-response)))
  "Returns full string definition for message of type 'GetPreviewMatrix-response"
  (cl:format cl:nil "op3_online_walking_module_msgs/PreviewResponse res~%~%~%================================================================================~%MSG: op3_online_walking_module_msgs/PreviewResponse~%int32 K_row~%int32 K_col~%float64[] K~%int32 P_row~%int32 P_col~%float64[] P~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetPreviewMatrix-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'res))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetPreviewMatrix-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetPreviewMatrix-response
    (cl:cons ':res (res msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetPreviewMatrix)))
  'GetPreviewMatrix-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetPreviewMatrix)))
  'GetPreviewMatrix-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPreviewMatrix)))
  "Returns string type for a service object of type '<GetPreviewMatrix>"
  "op3_online_walking_module_msgs/GetPreviewMatrix")