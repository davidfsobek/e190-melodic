; Auto-generated. Do not edit!


(cl:in-package e190_bot-msg)


;//! \htmlinclude ir_sensor.msg.html

(cl:defclass <ir_sensor> (roslisp-msg-protocol:ros-message)
  ((distance
    :reader distance
    :initarg :distance
    :type cl:float
    :initform 0.0))
)

(cl:defclass ir_sensor (<ir_sensor>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ir_sensor>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ir_sensor)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name e190_bot-msg:<ir_sensor> is deprecated: use e190_bot-msg:ir_sensor instead.")))

(cl:ensure-generic-function 'distance-val :lambda-list '(m))
(cl:defmethod distance-val ((m <ir_sensor>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader e190_bot-msg:distance-val is deprecated.  Use e190_bot-msg:distance instead.")
  (distance m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ir_sensor>) ostream)
  "Serializes a message object of type '<ir_sensor>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ir_sensor>) istream)
  "Deserializes a message object of type '<ir_sensor>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distance) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ir_sensor>)))
  "Returns string type for a message object of type '<ir_sensor>"
  "e190_bot/ir_sensor")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ir_sensor)))
  "Returns string type for a message object of type 'ir_sensor"
  "e190_bot/ir_sensor")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ir_sensor>)))
  "Returns md5sum for a message object of type '<ir_sensor>"
  "6e77fb10f0c8b4833ec273aa9ac74459")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ir_sensor)))
  "Returns md5sum for a message object of type 'ir_sensor"
  "6e77fb10f0c8b4833ec273aa9ac74459")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ir_sensor>)))
  "Returns full string definition for message of type '<ir_sensor>"
  (cl:format cl:nil "float32 distance~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ir_sensor)))
  "Returns full string definition for message of type 'ir_sensor"
  (cl:format cl:nil "float32 distance~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ir_sensor>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ir_sensor>))
  "Converts a ROS message object to a list"
  (cl:list 'ir_sensor
    (cl:cons ':distance (distance msg))
))
