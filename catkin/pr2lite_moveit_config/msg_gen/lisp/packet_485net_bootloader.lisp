; Auto-generated. Do not edit!


(cl:in-package packets_485net-msg)


;//! \htmlinclude packet_485net_bootloader.msg.html

(cl:defclass <packet_485net_bootloader> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (source
    :reader source
    :initarg :source
    :type cl:fixnum
    :initform 0)
   (destination
    :reader destination
    :initarg :destination
    :type cl:fixnum
    :initform 0)
   (protocol
    :reader protocol
    :initarg :protocol
    :type cl:fixnum
    :initform 0)
   (data
    :reader data
    :initarg :data
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (checksum
    :reader checksum
    :initarg :checksum
    :type cl:fixnum
    :initform 0))
)

(cl:defclass packet_485net_bootloader (<packet_485net_bootloader>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <packet_485net_bootloader>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'packet_485net_bootloader)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name packets_485net-msg:<packet_485net_bootloader> is deprecated: use packets_485net-msg:packet_485net_bootloader instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <packet_485net_bootloader>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader packets_485net-msg:header-val is deprecated.  Use packets_485net-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'source-val :lambda-list '(m))
(cl:defmethod source-val ((m <packet_485net_bootloader>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader packets_485net-msg:source-val is deprecated.  Use packets_485net-msg:source instead.")
  (source m))

(cl:ensure-generic-function 'destination-val :lambda-list '(m))
(cl:defmethod destination-val ((m <packet_485net_bootloader>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader packets_485net-msg:destination-val is deprecated.  Use packets_485net-msg:destination instead.")
  (destination m))

(cl:ensure-generic-function 'protocol-val :lambda-list '(m))
(cl:defmethod protocol-val ((m <packet_485net_bootloader>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader packets_485net-msg:protocol-val is deprecated.  Use packets_485net-msg:protocol instead.")
  (protocol m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <packet_485net_bootloader>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader packets_485net-msg:data-val is deprecated.  Use packets_485net-msg:data instead.")
  (data m))

(cl:ensure-generic-function 'checksum-val :lambda-list '(m))
(cl:defmethod checksum-val ((m <packet_485net_bootloader>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader packets_485net-msg:checksum-val is deprecated.  Use packets_485net-msg:checksum instead.")
  (checksum m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <packet_485net_bootloader>) ostream)
  "Serializes a message object of type '<packet_485net_bootloader>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'source)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'destination)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'protocol)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'data))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'checksum)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <packet_485net_bootloader>) istream)
  "Deserializes a message object of type '<packet_485net_bootloader>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'source)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'destination)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'protocol)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'checksum)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<packet_485net_bootloader>)))
  "Returns string type for a message object of type '<packet_485net_bootloader>"
  "packets_485net/packet_485net_bootloader")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'packet_485net_bootloader)))
  "Returns string type for a message object of type 'packet_485net_bootloader"
  "packets_485net/packet_485net_bootloader")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<packet_485net_bootloader>)))
  "Returns md5sum for a message object of type '<packet_485net_bootloader>"
  "a521989bb34adf962442e5b092f890fe")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'packet_485net_bootloader)))
  "Returns md5sum for a message object of type 'packet_485net_bootloader"
  "a521989bb34adf962442e5b092f890fe")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<packet_485net_bootloader>)))
  "Returns full string definition for message of type '<packet_485net_bootloader>"
  (cl:format cl:nil "Header header~%uint8 source~%uint8 destination~%uint8 protocol~%uint8[] data~%uint8 checksum~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'packet_485net_bootloader)))
  "Returns full string definition for message of type 'packet_485net_bootloader"
  (cl:format cl:nil "Header header~%uint8 source~%uint8 destination~%uint8 protocol~%uint8[] data~%uint8 checksum~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <packet_485net_bootloader>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <packet_485net_bootloader>))
  "Converts a ROS message object to a list"
  (cl:list 'packet_485net_bootloader
    (cl:cons ':header (header msg))
    (cl:cons ':source (source msg))
    (cl:cons ':destination (destination msg))
    (cl:cons ':protocol (protocol msg))
    (cl:cons ':data (data msg))
    (cl:cons ':checksum (checksum msg))
))
