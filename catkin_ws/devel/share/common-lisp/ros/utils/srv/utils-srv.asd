
(cl:in-package :asdf)

(defsystem "utils-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "GetVisionDev" :depends-on ("_package_GetVisionDev"))
    (:file "_package_GetVisionDev" :depends-on ("_package"))
  ))