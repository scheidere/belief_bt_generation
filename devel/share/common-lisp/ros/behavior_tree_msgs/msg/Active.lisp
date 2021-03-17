; Auto-generated. Do not edit!


(cl:in-package behavior_tree_msgs-msg)


;//! \htmlinclude Active.msg.html

(cl:defclass <Active> (roslisp-msg-protocol:ros-message)
  ((active
    :reader active
    :initarg :active
    :type cl:boolean
    :initform cl:nil)
   (id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0))
)

(cl:defclass Active (<Active>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Active>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Active)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name behavior_tree_msgs-msg:<Active> is deprecated: use behavior_tree_msgs-msg:Active instead.")))

(cl:ensure-generic-function 'active-val :lambda-list '(m))
(cl:defmethod active-val ((m <Active>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader behavior_tree_msgs-msg:active-val is deprecated.  Use behavior_tree_msgs-msg:active instead.")
  (active m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <Active>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader behavior_tree_msgs-msg:id-val is deprecated.  Use behavior_tree_msgs-msg:id instead.")
  (id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Active>) ostream)
  "Serializes a message object of type '<Active>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'active) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Active>) istream)
  "Deserializes a message object of type '<Active>"
    (cl:setf (cl:slot-value msg 'active) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Active>)))
  "Returns string type for a message object of type '<Active>"
  "behavior_tree_msgs/Active")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Active)))
  "Returns string type for a message object of type 'Active"
  "behavior_tree_msgs/Active")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Active>)))
  "Returns md5sum for a message object of type '<Active>"
  "c07e551e186876ca3af3ff1a8f330ec9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Active)))
  "Returns md5sum for a message object of type 'Active"
  "c07e551e186876ca3af3ff1a8f330ec9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Active>)))
  "Returns full string definition for message of type '<Active>"
  (cl:format cl:nil "bool active~%int64 id~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Active)))
  "Returns full string definition for message of type 'Active"
  (cl:format cl:nil "bool active~%int64 id~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Active>))
  (cl:+ 0
     1
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Active>))
  "Converts a ROS message object to a list"
  (cl:list 'Active
    (cl:cons ':active (active msg))
    (cl:cons ':id (id msg))
))
