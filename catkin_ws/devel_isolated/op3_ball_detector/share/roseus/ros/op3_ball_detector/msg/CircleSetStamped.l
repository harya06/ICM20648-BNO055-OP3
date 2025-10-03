;; Auto-generated. Do not edit!


(when (boundp 'op3_ball_detector::CircleSetStamped)
  (if (not (find-package "OP3_BALL_DETECTOR"))
    (make-package "OP3_BALL_DETECTOR"))
  (shadow 'CircleSetStamped (find-package "OP3_BALL_DETECTOR")))
(unless (find-package "OP3_BALL_DETECTOR::CIRCLESETSTAMPED")
  (make-package "OP3_BALL_DETECTOR::CIRCLESETSTAMPED"))

(in-package "ROS")
;;//! \htmlinclude CircleSetStamped.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass op3_ball_detector::CircleSetStamped
  :super ros::object
  :slots (_header _circles ))

(defmethod op3_ball_detector::CircleSetStamped
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:circles __circles) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Point :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _circles __circles)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:circles
   (&rest __circles)
   (if (keywordp (car __circles))
       (send* _circles __circles)
     (progn
       (if __circles (setq _circles (car __circles)))
       _circles)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Point[] _circles
    (apply #'+ (send-all _circles :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Point[] _circles
     (write-long (length _circles) s)
     (dolist (elem _circles)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Point[] _circles
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _circles (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _circles)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get op3_ball_detector::CircleSetStamped :md5sum-) "a9ed251dbca477265ff458e1e16f7437")
(setf (get op3_ball_detector::CircleSetStamped :datatype-) "op3_ball_detector/CircleSetStamped")
(setf (get op3_ball_detector::CircleSetStamped :definition-)
      "# This represents the set of detected circles

#timestamp and frame id of the image frame
std_msgs/Header header

#set of detected circles: 
#      (circles[i].x, circles[i].y) is the center point in image coordinates
#      circles[i].z is the circle radius
geometry_msgs/Point[] circles

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :op3_ball_detector/CircleSetStamped "a9ed251dbca477265ff458e1e16f7437")


