
(cl:in-package :asdf)

(defsystem "op3_online_walking_module_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "FootStepArray" :depends-on ("_package_FootStepArray"))
    (:file "_package_FootStepArray" :depends-on ("_package"))
    (:file "FootStepCommand" :depends-on ("_package_FootStepCommand"))
    (:file "_package_FootStepCommand" :depends-on ("_package"))
    (:file "JointPose" :depends-on ("_package_JointPose"))
    (:file "_package_JointPose" :depends-on ("_package"))
    (:file "KinematicsPose" :depends-on ("_package_KinematicsPose"))
    (:file "_package_KinematicsPose" :depends-on ("_package"))
    (:file "PreviewRequest" :depends-on ("_package_PreviewRequest"))
    (:file "_package_PreviewRequest" :depends-on ("_package"))
    (:file "PreviewResponse" :depends-on ("_package_PreviewResponse"))
    (:file "_package_PreviewResponse" :depends-on ("_package"))
    (:file "Step2D" :depends-on ("_package_Step2D"))
    (:file "_package_Step2D" :depends-on ("_package"))
    (:file "Step2DArray" :depends-on ("_package_Step2DArray"))
    (:file "_package_Step2DArray" :depends-on ("_package"))
    (:file "WalkingParam" :depends-on ("_package_WalkingParam"))
    (:file "_package_WalkingParam" :depends-on ("_package"))
  ))