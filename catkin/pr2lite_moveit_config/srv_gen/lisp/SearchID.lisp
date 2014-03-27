; Auto-generated. Do not edit!


(cl:in-package net_485net_id_handler-srv)


;//! \htmlinclude SearchID-request.msg.html

(cl:defclass <SearchID-request> (roslisp-msg-protocol:ros-message)
  ((type
    :reader type
    :initarg :type
    :type cl:string
    :initform "")
   (desc
    :reader desc
    :initarg :desc
    :type cl:string
    :initform ""))
)

(cl:defclass SearchID-request (<SearchID-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SearchID-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SearchID-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name net_485net_id_handler-srv:<SearchID-request> is deprecated: use net_485net_id_handler-srv:SearchID-request instead.")))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <SearchID-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader net_485net_id_handler-srv:type-val is deprecated.  Use net_485net_id_handler-srv:type instead.")
  (type m))

(cl:ensure-generic-function 'desc-val :lambda-list '(m))
(cl:defmethod desc-val ((m <SearchID-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader net_485net_id_handler-srv:desc-val is deprecated.  Use net_485net_id_handler-srv:desc instead.")
  (desc m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SearchID-request>) ostream)
  "Serializes a message object of type '<SearchID-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'type))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'type))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'desc))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'desc))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SearchID-request>) istream)
  "Deserializes a message object of type '<SearchID-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'type) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'type) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'desc) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'desc) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SearchID-request>)))
  "Returns string type for a service object of type '<SearchID-request>"
  "net_485net_id_handler/SearchIDRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SearchID-request)))
  "Returns string type for a service object of type 'SearchID-request"
  "net_485net_id_handler/SearchIDRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SearchID-request>)))
  "Returns md5sum for a message object of type '<SearchID-request>"
  "78e222123899a19f1660debb8b9dde08")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SearchID-request)))
  "Returns md5sum for a message object of type 'SearchID-request"
  "78e222123899a19f1660debb8b9dde08")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SearchID-request>)))
  "Returns full string definition for message of type '<SearchID-request>"
  (cl:format cl:nil "string type~%string desc~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SearchID-request)))
  "Returns full string definition for message of type 'SearchID-request"
  (cl:format cl:nil "string type~%string desc~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SearchID-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'type))
     4 (cl:length (cl:slot-value msg 'desc))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SearchID-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SearchID-request
    (cl:cons ':type (type msg))
    (cl:cons ':desc (desc msg))
))
;//! \htmlinclude SearchID-response.msg.html

(cl:defclass <SearchID-response> (roslisp-msg-protocol:ros-message)
  ((res
    :reader res
    :initarg :res
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SearchID-response (<SearchID-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SearchID-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SearchID-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name net_485net_id_handler-srv:<SearchID-response> is deprecated: use net_485net_id_handler-srv:SearchID-response instead.")))

(cl:ensure-generic-function 'res-val :lambda-list '(m))
(cl:defmethod res-val ((m <SearchID-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader net_485net_id_handler-srv:res-val is deprecated.  Use net_485net_id_handler-srv:res instead.")
  (res m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SearchID-response>) ostream)
  "Serializes a message object of type '<SearchID-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'res)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SearchID-response>) istream)
  "Deserializes a message object of type '<SearchID-response>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'res)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SearchID-response>)))
  "Returns string type for a service object of type '<SearchID-response>"
  "net_485net_id_handler/SearchIDResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SearchID-response)))
  "Returns string type for a service object of type 'SearchID-response"
  "net_485net_id_handler/SearchIDResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SearchID-response>)))
  "Returns md5sum for a message object of type '<SearchID-response>"
  "78e222123899a19f1660debb8b9dde08")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SearchID-response)))
  "Returns md5sum for a message object of type 'SearchID-response"
  "78e222123899a19f1660debb8b9dde08")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SearchID-response>)))
  "Returns full string definition for message of type '<SearchID-response>"
  (cl:format cl:nil "uint8 res~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SearchID-response)))
  "Returns full string definition for message of type 'SearchID-response"
  (cl:format cl:nil "uint8 res~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SearchID-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SearchID-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SearchID-response
    (cl:cons ':res (res msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SearchID)))
  'SearchID-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SearchID)))
  'SearchID-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SearchID)))
  "Returns string type for a service object of type '<SearchID>"
  "net_485net_id_handler/SearchID")