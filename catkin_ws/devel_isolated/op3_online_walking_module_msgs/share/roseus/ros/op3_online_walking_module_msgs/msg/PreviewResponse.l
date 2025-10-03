;; Auto-generated. Do not edit!


(when (boundp 'op3_online_walking_module_msgs::PreviewResponse)
  (if (not (find-package "OP3_ONLINE_WALKING_MODULE_MSGS"))
    (make-package "OP3_ONLINE_WALKING_MODULE_MSGS"))
  (shadow 'PreviewResponse (find-package "OP3_ONLINE_WALKING_MODULE_MSGS")))
(unless (find-package "OP3_ONLINE_WALKING_MODULE_MSGS::PREVIEWRESPONSE")
  (make-package "OP3_ONLINE_WALKING_MODULE_MSGS::PREVIEWRESPONSE"))

(in-package "ROS")
;;//! \htmlinclude PreviewResponse.msg.html


(defclass op3_online_walking_module_msgs::PreviewResponse
  :super ros::object
  :slots (_K_row _K_col _K _P_row _P_col _P ))

(defmethod op3_online_walking_module_msgs::PreviewResponse
  (:init
   (&key
    ((:K_row __K_row) 0)
    ((:K_col __K_col) 0)
    ((:K __K) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:P_row __P_row) 0)
    ((:P_col __P_col) 0)
    ((:P __P) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _K_row (round __K_row))
   (setq _K_col (round __K_col))
   (setq _K __K)
   (setq _P_row (round __P_row))
   (setq _P_col (round __P_col))
   (setq _P __P)
   self)
  (:K_row
   (&optional __K_row)
   (if __K_row (setq _K_row __K_row)) _K_row)
  (:K_col
   (&optional __K_col)
   (if __K_col (setq _K_col __K_col)) _K_col)
  (:K
   (&optional __K)
   (if __K (setq _K __K)) _K)
  (:P_row
   (&optional __P_row)
   (if __P_row (setq _P_row __P_row)) _P_row)
  (:P_col
   (&optional __P_col)
   (if __P_col (setq _P_col __P_col)) _P_col)
  (:P
   (&optional __P)
   (if __P (setq _P __P)) _P)
  (:serialization-length
   ()
   (+
    ;; int32 _K_row
    4
    ;; int32 _K_col
    4
    ;; float64[] _K
    (* 8    (length _K)) 4
    ;; int32 _P_row
    4
    ;; int32 _P_col
    4
    ;; float64[] _P
    (* 8    (length _P)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _K_row
       (write-long _K_row s)
     ;; int32 _K_col
       (write-long _K_col s)
     ;; float64[] _K
     (write-long (length _K) s)
     (dotimes (i (length _K))
       (sys::poke (elt _K i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; int32 _P_row
       (write-long _P_row s)
     ;; int32 _P_col
       (write-long _P_col s)
     ;; float64[] _P
     (write-long (length _P) s)
     (dotimes (i (length _P))
       (sys::poke (elt _P i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _K_row
     (setq _K_row (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _K_col
     (setq _K_col (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64[] _K
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _K (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _K i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; int32 _P_row
     (setq _P_row (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _P_col
     (setq _P_col (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64[] _P
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _P (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _P i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get op3_online_walking_module_msgs::PreviewResponse :md5sum-) "485c4c72e8d24c7f7a770f8a88709eb6")
(setf (get op3_online_walking_module_msgs::PreviewResponse :datatype-) "op3_online_walking_module_msgs/PreviewResponse")
(setf (get op3_online_walking_module_msgs::PreviewResponse :definition-)
      "int32 K_row
int32 K_col
float64[] K
int32 P_row
int32 P_col
float64[] P

")



(provide :op3_online_walking_module_msgs/PreviewResponse "485c4c72e8d24c7f7a770f8a88709eb6")


