
(cl:in-package :asdf)

(defsystem "op3_ball_detector-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "BallDetectorParams" :depends-on ("_package_BallDetectorParams"))
    (:file "_package_BallDetectorParams" :depends-on ("_package"))
    (:file "CircleSetStamped" :depends-on ("_package_CircleSetStamped"))
    (:file "_package_CircleSetStamped" :depends-on ("_package"))
  ))