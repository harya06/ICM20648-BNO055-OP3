; Auto-generated. Do not edit!


(cl:in-package op3_action_module_msgs-msg)


;//! \htmlinclude StartAction.msg.html

(cl:defclass <StartAction> (roslisp-msg-protocol:ros-message)
  ((page_num
    :reader page_num
    :initarg :page_num
    :type cl:integer
    :initform 0)
   (joint_name_array
    :reader joint_name_array
    :initarg :joint_name_array
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass StartAction (<StartAction>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StartAction>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StartAction)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name op3_action_module_msgs-msg:<StartAction> is deprecated: use op3_action_module_msgs-msg:StartAction instead.")))

(cl:ensure-generic-function 'page_num-val :lambda-list '(m))
(cl:defmethod page_num-val ((m <StartAction>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_action_module_msgs-msg:page_num-val is deprecated.  Use op3_action_module_msgs-msg:page_num instead.")
  (page_num m))

(cl:ensure-generic-function 'joint_name_array-val :lambda-list '(m))
(cl:defmethod joint_name_array-val ((m <StartAction>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_action_module_msgs-msg:joint_name_array-val is deprecated.  Use op3_action_module_msgs-msg:joint_name_array instead.")
  (joint_name_array m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StartAction>) ostream)
  "Serializes a message object of type '<StartAction>"
  (cl:let* ((signed (cl:slot-value msg 'page_num)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'joint_name_array))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'joint_name_array))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StartAction>) istream)
  "Deserializes a message object of type '<StartAction>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'page_num) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'joint_name_array) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'joint_name_array)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StartAction>)))
  "Returns string type for a message object of type '<StartAction>"
  "op3_action_module_msgs/StartAction")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartAction)))
  "Returns string type for a message object of type 'StartAction"
  "op3_action_module_msgs/StartAction")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StartAction>)))
  "Returns md5sum for a message object of type '<StartAction>"
  "089f02f04489a5eddf9886b2ae161539")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StartAction)))
  "Returns md5sum for a message object of type 'StartAction"
  "089f02f04489a5eddf9886b2ae161539")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StartAction>)))
  "Returns full string definition for message of type '<StartAction>"
  (cl:format cl:nil "int32     page_num~%string[]  joint_name_array~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StartAction)))
  "Returns full string definition for message of type 'StartAction"
  (cl:format cl:nil "int32     page_num~%string[]  joint_name_array~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StartAction>))
  (cl:+ 0
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'joint_name_array) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StartAction>))
  "Converts a ROS message object to a list"
  (cl:list 'StartAction
    (cl:cons ':page_num (page_num msg))
    (cl:cons ':joint_name_array (joint_name_array msg))
))
