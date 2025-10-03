;; Auto-generated. Do not edit!


(when (boundp 'op3_camera_setting_tool::V4lParameters)
  (if (not (find-package "OP3_CAMERA_SETTING_TOOL"))
    (make-package "OP3_CAMERA_SETTING_TOOL"))
  (shadow 'V4lParameters (find-package "OP3_CAMERA_SETTING_TOOL")))
(unless (find-package "OP3_CAMERA_SETTING_TOOL::V4LPARAMETERS")
  (make-package "OP3_CAMERA_SETTING_TOOL::V4LPARAMETERS"))

(in-package "ROS")
;;//! \htmlinclude V4lParameters.msg.html


(defclass op3_camera_setting_tool::V4lParameters
  :super ros::object
  :slots (_video_parameter ))

(defmethod op3_camera_setting_tool::V4lParameters
  (:init
   (&key
    ((:video_parameter __video_parameter) (let (r) (dotimes (i 0) (push (instance op3_camera_setting_tool::V4lParameter :init) r)) r))
    )
   (send-super :init)
   (setq _video_parameter __video_parameter)
   self)
  (:video_parameter
   (&rest __video_parameter)
   (if (keywordp (car __video_parameter))
       (send* _video_parameter __video_parameter)
     (progn
       (if __video_parameter (setq _video_parameter (car __video_parameter)))
       _video_parameter)))
  (:serialization-length
   ()
   (+
    ;; op3_camera_setting_tool/V4lParameter[] _video_parameter
    (apply #'+ (send-all _video_parameter :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; op3_camera_setting_tool/V4lParameter[] _video_parameter
     (write-long (length _video_parameter) s)
     (dolist (elem _video_parameter)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; op3_camera_setting_tool/V4lParameter[] _video_parameter
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _video_parameter (let (r) (dotimes (i n) (push (instance op3_camera_setting_tool::V4lParameter :init) r)) r))
     (dolist (elem- _video_parameter)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get op3_camera_setting_tool::V4lParameters :md5sum-) "becf7b1c441fba0b43309667fa8477b4")
(setf (get op3_camera_setting_tool::V4lParameters :datatype-) "op3_camera_setting_tool/V4lParameters")
(setf (get op3_camera_setting_tool::V4lParameters :definition-)
      "V4lParameter[]    video_parameter
================================================================================
MSG: op3_camera_setting_tool/V4lParameter
string  name
int32   value
")



(provide :op3_camera_setting_tool/V4lParameters "becf7b1c441fba0b43309667fa8477b4")


