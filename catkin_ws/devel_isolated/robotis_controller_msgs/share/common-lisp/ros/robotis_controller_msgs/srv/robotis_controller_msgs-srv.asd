
(cl:in-package :asdf)

(defsystem "robotis_controller_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "GetJointModule" :depends-on ("_package_GetJointModule"))
    (:file "_package_GetJointModule" :depends-on ("_package"))
    (:file "LoadOffset" :depends-on ("_package_LoadOffset"))
    (:file "_package_LoadOffset" :depends-on ("_package"))
    (:file "SetJointModule" :depends-on ("_package_SetJointModule"))
    (:file "_package_SetJointModule" :depends-on ("_package"))
    (:file "SetModule" :depends-on ("_package_SetModule"))
    (:file "_package_SetModule" :depends-on ("_package"))
  ))