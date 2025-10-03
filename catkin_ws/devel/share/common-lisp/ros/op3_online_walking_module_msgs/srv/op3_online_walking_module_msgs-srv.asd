
(cl:in-package :asdf)

(defsystem "op3_online_walking_module_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :op3_online_walking_module_msgs-msg
)
  :components ((:file "_package")
    (:file "GetJointPose" :depends-on ("_package_GetJointPose"))
    (:file "_package_GetJointPose" :depends-on ("_package"))
    (:file "GetKinematicsPose" :depends-on ("_package_GetKinematicsPose"))
    (:file "_package_GetKinematicsPose" :depends-on ("_package"))
    (:file "GetPreviewMatrix" :depends-on ("_package_GetPreviewMatrix"))
    (:file "_package_GetPreviewMatrix" :depends-on ("_package"))
  ))