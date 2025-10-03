
(cl:in-package :asdf)

(defsystem "utils-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "compass" :depends-on ("_package_compass"))
    (:file "_package_compass" :depends-on ("_package"))
  ))