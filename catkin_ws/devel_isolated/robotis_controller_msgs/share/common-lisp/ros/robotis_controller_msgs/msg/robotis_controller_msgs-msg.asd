
(cl:in-package :asdf)

(defsystem "robotis_controller_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "JointCtrlModule" :depends-on ("_package_JointCtrlModule"))
    (:file "_package_JointCtrlModule" :depends-on ("_package"))
    (:file "SensorYPR" :depends-on ("_package_SensorYPR"))
    (:file "_package_SensorYPR" :depends-on ("_package"))
    (:file "StatusMsg" :depends-on ("_package_StatusMsg"))
    (:file "_package_StatusMsg" :depends-on ("_package"))
    (:file "SyncWriteItem" :depends-on ("_package_SyncWriteItem"))
    (:file "_package_SyncWriteItem" :depends-on ("_package"))
    (:file "WriteControlTable" :depends-on ("_package_WriteControlTable"))
    (:file "_package_WriteControlTable" :depends-on ("_package"))
  ))