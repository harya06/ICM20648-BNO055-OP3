;; Auto-generated. Do not edit!


(when (boundp 'op3_online_walking_module_msgs::GetPreviewMatrix)
  (if (not (find-package "OP3_ONLINE_WALKING_MODULE_MSGS"))
    (make-package "OP3_ONLINE_WALKING_MODULE_MSGS"))
  (shadow 'GetPreviewMatrix (find-package "OP3_ONLINE_WALKING_MODULE_MSGS")))
(unless (find-package "OP3_ONLINE_WALKING_MODULE_MSGS::GETPREVIEWMATRIX")
  (make-package "OP3_ONLINE_WALKING_MODULE_MSGS::GETPREVIEWMATRIX"))
(unless (find-package "OP3_ONLINE_WALKING_MODULE_MSGS::GETPREVIEWMATRIXREQUEST")
  (make-package "OP3_ONLINE_WALKING_MODULE_MSGS::GETPREVIEWMATRIXREQUEST"))
(unless (find-package "OP3_ONLINE_WALKING_MODULE_MSGS::GETPREVIEWMATRIXRESPONSE")
  (make-package "OP3_ONLINE_WALKING_MODULE_MSGS::GETPREVIEWMATRIXRESPONSE"))

(in-package "ROS")





(defclass op3_online_walking_module_msgs::GetPreviewMatrixRequest
  :super ros::object
  :slots (_req ))

(defmethod op3_online_walking_module_msgs::GetPreviewMatrixRequest
  (:init
   (&key
    ((:req __req) (instance op3_online_walking_module_msgs::PreviewRequest :init))
    )
   (send-super :init)
   (setq _req __req)
   self)
  (:req
   (&rest __req)
   (if (keywordp (car __req))
       (send* _req __req)
     (progn
       (if __req (setq _req (car __req)))
       _req)))
  (:serialization-length
   ()
   (+
    ;; op3_online_walking_module_msgs/PreviewRequest _req
    (send _req :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; op3_online_walking_module_msgs/PreviewRequest _req
       (send _req :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; op3_online_walking_module_msgs/PreviewRequest _req
     (send _req :deserialize buf ptr-) (incf ptr- (send _req :serialization-length))
   ;;
   self)
  )

(defclass op3_online_walking_module_msgs::GetPreviewMatrixResponse
  :super ros::object
  :slots (_res ))

(defmethod op3_online_walking_module_msgs::GetPreviewMatrixResponse
  (:init
   (&key
    ((:res __res) (instance op3_online_walking_module_msgs::PreviewResponse :init))
    )
   (send-super :init)
   (setq _res __res)
   self)
  (:res
   (&rest __res)
   (if (keywordp (car __res))
       (send* _res __res)
     (progn
       (if __res (setq _res (car __res)))
       _res)))
  (:serialization-length
   ()
   (+
    ;; op3_online_walking_module_msgs/PreviewResponse _res
    (send _res :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; op3_online_walking_module_msgs/PreviewResponse _res
       (send _res :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; op3_online_walking_module_msgs/PreviewResponse _res
     (send _res :deserialize buf ptr-) (incf ptr- (send _res :serialization-length))
   ;;
   self)
  )

(defclass op3_online_walking_module_msgs::GetPreviewMatrix
  :super ros::object
  :slots ())

(setf (get op3_online_walking_module_msgs::GetPreviewMatrix :md5sum-) "353f820108e54e41d7780433bc38ff53")
(setf (get op3_online_walking_module_msgs::GetPreviewMatrix :datatype-) "op3_online_walking_module_msgs/GetPreviewMatrix")
(setf (get op3_online_walking_module_msgs::GetPreviewMatrix :request) op3_online_walking_module_msgs::GetPreviewMatrixRequest)
(setf (get op3_online_walking_module_msgs::GetPreviewMatrix :response) op3_online_walking_module_msgs::GetPreviewMatrixResponse)

(defmethod op3_online_walking_module_msgs::GetPreviewMatrixRequest
  (:response () (instance op3_online_walking_module_msgs::GetPreviewMatrixResponse :init)))

(setf (get op3_online_walking_module_msgs::GetPreviewMatrixRequest :md5sum-) "353f820108e54e41d7780433bc38ff53")
(setf (get op3_online_walking_module_msgs::GetPreviewMatrixRequest :datatype-) "op3_online_walking_module_msgs/GetPreviewMatrixRequest")
(setf (get op3_online_walking_module_msgs::GetPreviewMatrixRequest :definition-)
      "op3_online_walking_module_msgs/PreviewRequest req

================================================================================
MSG: op3_online_walking_module_msgs/PreviewRequest
float64 control_cycle
float64 lipm_height
---
op3_online_walking_module_msgs/PreviewResponse res


================================================================================
MSG: op3_online_walking_module_msgs/PreviewResponse
int32 K_row
int32 K_col
float64[] K
int32 P_row
int32 P_col
float64[] P
")

(setf (get op3_online_walking_module_msgs::GetPreviewMatrixResponse :md5sum-) "353f820108e54e41d7780433bc38ff53")
(setf (get op3_online_walking_module_msgs::GetPreviewMatrixResponse :datatype-) "op3_online_walking_module_msgs/GetPreviewMatrixResponse")
(setf (get op3_online_walking_module_msgs::GetPreviewMatrixResponse :definition-)
      "op3_online_walking_module_msgs/PreviewRequest req

================================================================================
MSG: op3_online_walking_module_msgs/PreviewRequest
float64 control_cycle
float64 lipm_height
---
op3_online_walking_module_msgs/PreviewResponse res


================================================================================
MSG: op3_online_walking_module_msgs/PreviewResponse
int32 K_row
int32 K_col
float64[] K
int32 P_row
int32 P_col
float64[] P
")



(provide :op3_online_walking_module_msgs/GetPreviewMatrix "353f820108e54e41d7780433bc38ff53")


