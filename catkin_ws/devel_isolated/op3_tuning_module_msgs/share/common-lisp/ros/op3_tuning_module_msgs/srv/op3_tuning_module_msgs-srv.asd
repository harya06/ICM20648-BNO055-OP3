
(cl:in-package :asdf)

(defsystem "op3_tuning_module_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :op3_tuning_module_msgs-msg
)
  :components ((:file "_package")
    (:file "GetPresentJointOffsetData" :depends-on ("_package_GetPresentJointOffsetData"))
    (:file "_package_GetPresentJointOffsetData" :depends-on ("_package"))
  ))