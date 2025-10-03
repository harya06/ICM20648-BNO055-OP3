; Auto-generated. Do not edit!


(cl:in-package robotis_controller_msgs-msg)


;//! \htmlinclude WriteControlTable.msg.html

(cl:defclass <WriteControlTable> (roslisp-msg-protocol:ros-message)
  ((joint_name
    :reader joint_name
    :initarg :joint_name
    :type cl:string
    :initform "")
   (start_item_name
    :reader start_item_name
    :initarg :start_item_name
    :type cl:string
    :initform "")
   (data_length
    :reader data_length
    :initarg :data_length
    :type cl:fixnum
    :initform 0)
   (data
    :reader data
    :initarg :data
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass WriteControlTable (<WriteControlTable>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WriteControlTable>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WriteControlTable)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robotis_controller_msgs-msg:<WriteControlTable> is deprecated: use robotis_controller_msgs-msg:WriteControlTable instead.")))

(cl:ensure-generic-function 'joint_name-val :lambda-list '(m))
(cl:defmethod joint_name-val ((m <WriteControlTable>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotis_controller_msgs-msg:joint_name-val is deprecated.  Use robotis_controller_msgs-msg:joint_name instead.")
  (joint_name m))

(cl:ensure-generic-function 'start_item_name-val :lambda-list '(m))
(cl:defmethod start_item_name-val ((m <WriteControlTable>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotis_controller_msgs-msg:start_item_name-val is deprecated.  Use robotis_controller_msgs-msg:start_item_name instead.")
  (start_item_name m))

(cl:ensure-generic-function 'data_length-val :lambda-list '(m))
(cl:defmethod data_length-val ((m <WriteControlTable>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotis_controller_msgs-msg:data_length-val is deprecated.  Use robotis_controller_msgs-msg:data_length instead.")
  (data_length m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <WriteControlTable>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotis_controller_msgs-msg:data-val is deprecated.  Use robotis_controller_msgs-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WriteControlTable>) ostream)
  "Serializes a message object of type '<WriteControlTable>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'joint_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'joint_name))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'start_item_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'start_item_name))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'data_length)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'data_length)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WriteControlTable>) istream)
  "Deserializes a message object of type '<WriteControlTable>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'joint_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'joint_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'start_item_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'start_item_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'data_length)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'data_length)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WriteControlTable>)))
  "Returns string type for a message object of type '<WriteControlTable>"
  "robotis_controller_msgs/WriteControlTable")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WriteControlTable)))
  "Returns string type for a message object of type 'WriteControlTable"
  "robotis_controller_msgs/WriteControlTable")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WriteControlTable>)))
  "Returns md5sum for a message object of type '<WriteControlTable>"
  "0d4e0e3e28ede43d877e2fb8f0a1a011")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WriteControlTable)))
  "Returns md5sum for a message object of type 'WriteControlTable"
  "0d4e0e3e28ede43d877e2fb8f0a1a011")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WriteControlTable>)))
  "Returns full string definition for message of type '<WriteControlTable>"
  (cl:format cl:nil "string joint_name~%string start_item_name~%uint16 data_length~%uint8[] data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WriteControlTable)))
  "Returns full string definition for message of type 'WriteControlTable"
  (cl:format cl:nil "string joint_name~%string start_item_name~%uint16 data_length~%uint8[] data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WriteControlTable>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'joint_name))
     4 (cl:length (cl:slot-value msg 'start_item_name))
     2
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WriteControlTable>))
  "Converts a ROS message object to a list"
  (cl:list 'WriteControlTable
    (cl:cons ':joint_name (joint_name msg))
    (cl:cons ':start_item_name (start_item_name msg))
    (cl:cons ':data_length (data_length msg))
    (cl:cons ':data (data msg))
))
