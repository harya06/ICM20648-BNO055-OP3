
(cl:in-package :asdf)

(defsystem "op3_tuning_module_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "JointOffsetData" :depends-on ("_package_JointOffsetData"))
    (:file "_package_JointOffsetData" :depends-on ("_package"))
    (:file "JointOffsetPositionData" :depends-on ("_package_JointOffsetPositionData"))
    (:file "_package_JointOffsetPositionData" :depends-on ("_package"))
    (:file "JointTorqueOnOff" :depends-on ("_package_JointTorqueOnOff"))
    (:file "_package_JointTorqueOnOff" :depends-on ("_package"))
    (:file "JointTorqueOnOffArray" :depends-on ("_package_JointTorqueOnOffArray"))
    (:file "_package_JointTorqueOnOffArray" :depends-on ("_package"))
  ))