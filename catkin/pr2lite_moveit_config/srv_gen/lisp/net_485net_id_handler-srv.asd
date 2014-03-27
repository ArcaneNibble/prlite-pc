
(cl:in-package :asdf)

(defsystem "net_485net_id_handler-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ReloadID" :depends-on ("_package_ReloadID"))
    (:file "_package_ReloadID" :depends-on ("_package"))
    (:file "SearchID" :depends-on ("_package_SearchID"))
    (:file "_package_SearchID" :depends-on ("_package"))
  ))