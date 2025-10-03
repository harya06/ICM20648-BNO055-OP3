
(cl:in-package :asdf)

(defsystem "op3_ball_detector-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :op3_ball_detector-msg
)
  :components ((:file "_package")
    (:file "GetParameters" :depends-on ("_package_GetParameters"))
    (:file "_package_GetParameters" :depends-on ("_package"))
    (:file "SetParameters" :depends-on ("_package_SetParameters"))
    (:file "_package_SetParameters" :depends-on ("_package"))
  ))