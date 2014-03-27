
(cl:in-package :asdf)

(defsystem "packets_485net-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "packet_485net_stream" :depends-on ("_package_packet_485net_stream"))
    (:file "_package_packet_485net_stream" :depends-on ("_package"))
    (:file "packet_485net_bootloader" :depends-on ("_package_packet_485net_bootloader"))
    (:file "_package_packet_485net_bootloader" :depends-on ("_package"))
    (:file "packet_485net_dgram" :depends-on ("_package_packet_485net_dgram"))
    (:file "_package_packet_485net_dgram" :depends-on ("_package"))
    (:file "packet_485net_raw" :depends-on ("_package_packet_485net_raw"))
    (:file "_package_packet_485net_raw" :depends-on ("_package"))
  ))