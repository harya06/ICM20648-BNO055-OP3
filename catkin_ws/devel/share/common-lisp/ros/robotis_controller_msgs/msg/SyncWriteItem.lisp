; Auto-generated. Do not edit!


(cl:in-package robotis_controller_msgs-msg)


;//! \htmlinclude SyncWriteItem.msg.html

(cl:defclass <SyncWriteItem> (roslisp-msg-protocol:ros-message)
  ((item_name
    :reader item_name
    :initarg :item_name
    :type cl:string
    :initform "")
   (joint_name
    :reader joint_name
    :initarg :joint_name
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (value
    :reader value
    :initarg :value
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass SyncWriteItem (<SyncWriteItem>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SyncWriteItem>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SyncWriteItem)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robotis_controller_msgs-msg:<SyncWriteItem> is deprecated: use robotis_controller_msgs-msg:SyncWriteItem instead.")))

(cl:ensure-generic-function 'item_name-val :lambda-list '(m))
(cl:defmethod item_name-val ((m <SyncWriteItem>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotis_controller_msgs-msg:item_name-val is deprecated.  Use robotis_controller_msgs-msg:item_name instead.")
  (item_name m))

(cl:ensure-generic-function 'joint_name-val :lambda-list '(m))
(cl:defmethod joint_name-val ((m <SyncWriteItem>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotis_controller_msgs-msg:joint_name-val is deprecated.  Use robotis_controller_msgs-msg:joint_name instead.")
  (joint_name m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <SyncWriteItem>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotis_controller_msgs-msg:value-val is deprecated.  Use robotis_controller_msgs-msg:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SyncWriteItem>) ostream)
  "Serializes a message object of type '<SyncWriteItem>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'item_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'item_name))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'joint_name))))
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
   (cl:slot-value msg 'joint_name))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) ele) ostream))
   (cl:slot-value msg 'value))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SyncWriteItem>) istream)
  "Deserializes a message object of type '<SyncWriteItem>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'item_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'item_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'joint_name) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'joint_name)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'value) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'value)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SyncWriteItem>)))
  "Returns string type for a message object of type '<SyncWriteItem>"
  "robotis_controller_msgs/SyncWriteItem")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SyncWriteItem)))
  "Returns string type for a message object of type 'SyncWriteItem"
  "robotis_controller_msgs/SyncWriteItem")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SyncWriteItem>)))
  "Returns md5sum for a message object of type '<SyncWriteItem>"
  "f558e04f04dbcc25ce64aa1f45f3dbdd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SyncWriteItem)))
  "Returns md5sum for a message object of type 'SyncWriteItem"
  "f558e04f04dbcc25ce64aa1f45f3dbdd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SyncWriteItem>)))
  "Returns full string definition for message of type '<SyncWriteItem>"
  (cl:format cl:nil "string item_name~%string[] joint_name~%uint32[] value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SyncWriteItem)))
  "Returns full string definition for message of type 'SyncWriteItem"
  (cl:format cl:nil "string item_name~%string[] joint_name~%uint32[] value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SyncWriteItem>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'item_name))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'joint_name) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'value) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SyncWriteItem>))
  "Converts a ROS message object to a list"
  (cl:list 'SyncWriteItem
    (cl:cons ':item_name (item_name msg))
    (cl:cons ':joint_name (joint_name msg))
    (cl:cons ':value (value msg))
))
