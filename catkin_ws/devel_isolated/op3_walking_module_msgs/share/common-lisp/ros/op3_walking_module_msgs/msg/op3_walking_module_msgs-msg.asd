
(cl:in-package :asdf)

(defsystem "op3_walking_module_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "WalkingParam" :depends-on ("_package_WalkingParam"))
    (:file "_package_WalkingParam" :depends-on ("_package"))
  ))