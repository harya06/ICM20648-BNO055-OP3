;; Auto-generated. Do not edit!


(when (boundp 'utils::GetVisionDev)
  (if (not (find-package "UTILS"))
    (make-package "UTILS"))
  (shadow 'GetVisionDev (find-package "UTILS")))
(unless (find-package "UTILS::GETVISIONDEV")
  (make-package "UTILS::GETVISIONDEV"))
(unless (find-package "UTILS::GETVISIONDEVREQUEST")
  (make-package "UTILS::GETVISIONDEVREQUEST"))
(unless (find-package "UTILS::GETVISIONDEVRESPONSE")
  (make-package "UTILS::GETVISIONDEVRESPONSE"))

(in-package "ROS")





(defclass utils::GetVisionDevRequest
  :super ros::object
  :slots ())

(defmethod utils::GetVisionDevRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass utils::GetVisionDevResponse
  :super ros::object
  :slots (_dev ))

(defmethod utils::GetVisionDevResponse
  (:init
   (&key
    ((:dev __dev) 0)
    )
   (send-super :init)
   (setq _dev (round __dev))
   self)
  (:dev
   (&optional __dev)
   (if __dev (setq _dev __dev)) _dev)
  (:serialization-length
   ()
   (+
    ;; int64 _dev
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _dev
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _dev (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _dev) (= (length (_dev . bv)) 2)) ;; bignum
              (write-long (ash (elt (_dev . bv) 0) 0) s)
              (write-long (ash (elt (_dev . bv) 1) -1) s))
             ((and (class _dev) (= (length (_dev . bv)) 1)) ;; big1
              (write-long (elt (_dev . bv) 0) s)
              (write-long (if (>= _dev 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _dev s)(write-long (if (>= _dev 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _dev
#+(or :alpha :irix6 :x86_64)
      (setf _dev (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _dev (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(defclass utils::GetVisionDev
  :super ros::object
  :slots ())

(setf (get utils::GetVisionDev :md5sum-) "28d20ec34b723ab37e9c2029519bbce4")
(setf (get utils::GetVisionDev :datatype-) "utils/GetVisionDev")
(setf (get utils::GetVisionDev :request) utils::GetVisionDevRequest)
(setf (get utils::GetVisionDev :response) utils::GetVisionDevResponse)

(defmethod utils::GetVisionDevRequest
  (:response () (instance utils::GetVisionDevResponse :init)))

(setf (get utils::GetVisionDevRequest :md5sum-) "28d20ec34b723ab37e9c2029519bbce4")
(setf (get utils::GetVisionDevRequest :datatype-) "utils/GetVisionDevRequest")
(setf (get utils::GetVisionDevRequest :definition-)
      "
---
int64 dev
")

(setf (get utils::GetVisionDevResponse :md5sum-) "28d20ec34b723ab37e9c2029519bbce4")
(setf (get utils::GetVisionDevResponse :datatype-) "utils/GetVisionDevResponse")
(setf (get utils::GetVisionDevResponse :definition-)
      "
---
int64 dev
")



(provide :utils/GetVisionDev "28d20ec34b723ab37e9c2029519bbce4")


