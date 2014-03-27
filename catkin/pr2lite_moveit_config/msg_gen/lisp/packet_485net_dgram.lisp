; Auto-generated. Do not edit!


(cl:in-package packets_485net-msg)


;//! \htmlinclude packet_485net_dgram.msg.html

(cl:defclass <packet_485net_dgram> (roslisp-msg-protocol:ros-message)
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
   (sport
    :reader sport
    :initarg :sport
    :type cl:fixnum
    :initform 0)
   (dport
    :reader dport
    :initarg :dport
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

(cl:defclass packet_485net_dgram (<packet_485net_dgram>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <packet_485net_dgram>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'packet_485net_dgram)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name packets_485net-msg:<packet_485net_dgram> is deprecated: use packets_485net-msg:packet_485net_dgram instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <packet_485net_dgram>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader packets_485net-msg:header-val is deprecated.  Use packets_485net-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'source-val :lambda-list '(m))
(cl:defmethod source-val ((m <packet_485net_dgram>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader packets_485net-msg:source-val is deprecated.  Use packets_485net-msg:source instead.")
  (source m))

(cl:ensure-generic-function 'destination-val :lambda-list '(m))
(cl:defmethod destination-val ((m <packet_485net_dgram>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader packets_485net-msg:destination-val is deprecated.  Use packets_485net-msg:destination instead.")
  (destination m))

(cl:ensure-generic-function 'sport-val :lambda-list '(m))
(cl:defmethod sport-val ((m <packet_485net_dgram>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader packets_485net-msg:sport-val is deprecated.  Use packets_485net-msg:sport instead.")
  (sport m))

(cl:ensure-generic-function 'dport-val :lambda-list '(m))
(cl:defmethod dport-val ((m <packet_485net_dgram>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader packets_485net-msg:dport-val is deprecated.  Use packets_485net-msg:dport instead.")
  (dport m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <packet_485net_dgram>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader packets_485net-msg:data-val is deprecated.  Use packets_485net-msg:data instead.")
  (data m))

(cl:ensure-generic-function 'checksum-val :lambda-list '(m))
(cl:defmethod checksum-val ((m <packet_485net_dgram>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader packets_485net-msg:checksum-val is deprecated.  Use packets_485net-msg:checksum instead.")
  (checksum m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <packet_485net_dgram>) ostream)
  "Serializes a message object of type '<packet_485net_dgram>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'source)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'destination)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sport)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dport)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'data))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'checksum)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <packet_485net_dgram>) istream)
  "Deserializes a message object of type '<packet_485net_dgram>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'source)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'destination)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sport)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dport)) (cl:read-byte istream))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<packet_485net_dgram>)))
  "Returns string type for a message object of type '<packet_485net_dgram>"
  "packets_485net/packet_485net_dgram")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'packet_485net_dgram)))
  "Returns string type for a message object of type 'packet_485net_dgram"
  "packets_485net/packet_485net_dgram")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<packet_485net_dgram>)))
  "Returns md5sum for a message object of type '<packet_485net_dgram>"
  "a884c574e8693a21d85f5d9bd13095bb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'packet_485net_dgram)))
  "Returns md5sum for a message object of type 'packet_485net_dgram"
  "a884c574e8693a21d85f5d9bd13095bb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<packet_485net_dgram>)))
  "Returns full string definition for message of type '<packet_485net_dgram>"
  (cl:format cl:nil "Header header~%uint8 source~%uint8 destination~%uint8 sport~%uint8 dport~%uint8[] data~%uint8 checksum~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'packet_485net_dgram)))
  "Returns full string definition for message of type 'packet_485net_dgram"
  (cl:format cl:nil "Header header~%uint8 source~%uint8 destination~%uint8 sport~%uint8 dport~%uint8[] data~%uint8 checksum~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <packet_485net_dgram>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     1
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <packet_485net_dgram>))
  "Converts a ROS message object to a list"
  (cl:list 'packet_485net_dgram
    (cl:cons ':header (header msg))
    (cl:cons ':source (source msg))
    (cl:cons ':destination (destination msg))
    (cl:cons ':sport (sport msg))
    (cl:cons ':dport (dport msg))
    (cl:cons ':data (data msg))
    (cl:cons ':checksum (checksum msg))
))
