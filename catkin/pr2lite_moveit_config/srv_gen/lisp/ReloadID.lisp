; Auto-generated. Do not edit!


(cl:in-package net_485net_id_handler-srv)


;//! \htmlinclude ReloadID-request.msg.html

(cl:defclass <ReloadID-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ReloadID-request (<ReloadID-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReloadID-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReloadID-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name net_485net_id_handler-srv:<ReloadID-request> is deprecated: use net_485net_id_handler-srv:ReloadID-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReloadID-request>) ostream)
  "Serializes a message object of type '<ReloadID-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReloadID-request>) istream)
  "Deserializes a message object of type '<ReloadID-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReloadID-request>)))
  "Returns string type for a service object of type '<ReloadID-request>"
  "net_485net_id_handler/ReloadIDRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReloadID-request)))
  "Returns string type for a service object of type 'ReloadID-request"
  "net_485net_id_handler/ReloadIDRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReloadID-request>)))
  "Returns md5sum for a message object of type '<ReloadID-request>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReloadID-request)))
  "Returns md5sum for a message object of type 'ReloadID-request"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReloadID-request>)))
  "Returns full string definition for message of type '<ReloadID-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReloadID-request)))
  "Returns full string definition for message of type 'ReloadID-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReloadID-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReloadID-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ReloadID-request
))
;//! \htmlinclude ReloadID-response.msg.html

(cl:defclass <ReloadID-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ReloadID-response (<ReloadID-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReloadID-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReloadID-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name net_485net_id_handler-srv:<ReloadID-response> is deprecated: use net_485net_id_handler-srv:ReloadID-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReloadID-response>) ostream)
  "Serializes a message object of type '<ReloadID-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReloadID-response>) istream)
  "Deserializes a message object of type '<ReloadID-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReloadID-response>)))
  "Returns string type for a service object of type '<ReloadID-response>"
  "net_485net_id_handler/ReloadIDResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReloadID-response)))
  "Returns string type for a service object of type 'ReloadID-response"
  "net_485net_id_handler/ReloadIDResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReloadID-response>)))
  "Returns md5sum for a message object of type '<ReloadID-response>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReloadID-response)))
  "Returns md5sum for a message object of type 'ReloadID-response"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReloadID-response>)))
  "Returns full string definition for message of type '<ReloadID-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReloadID-response)))
  "Returns full string definition for message of type 'ReloadID-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReloadID-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReloadID-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ReloadID-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ReloadID)))
  'ReloadID-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ReloadID)))
  'ReloadID-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReloadID)))
  "Returns string type for a service object of type '<ReloadID>"
  "net_485net_id_handler/ReloadID")