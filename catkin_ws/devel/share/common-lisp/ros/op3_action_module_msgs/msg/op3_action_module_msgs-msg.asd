
(cl:in-package :asdf)

(defsystem "op3_action_module_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "StartAction" :depends-on ("_package_StartAction"))
    (:file "_package_StartAction" :depends-on ("_package"))
  ))