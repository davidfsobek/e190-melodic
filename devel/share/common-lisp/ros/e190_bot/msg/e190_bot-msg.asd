
(cl:in-package :asdf)

(defsystem "e190_bot-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ir_sensor" :depends-on ("_package_ir_sensor"))
    (:file "_package_ir_sensor" :depends-on ("_package"))
  ))