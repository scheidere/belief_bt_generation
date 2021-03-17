; Auto-generated. Do not edit!


(cl:in-package behavior_tree_msgs-msg)


;//! \htmlinclude BehaviorTreeCommand.msg.html

(cl:defclass <BehaviorTreeCommand> (roslisp-msg-protocol:ros-message)
  ((condition_name
    :reader condition_name
    :initarg :condition_name
    :type cl:string
    :initform "")
   (status
    :reader status
    :initarg :status
    :type cl:fixnum
    :initform 0))
)

(cl:defclass BehaviorTreeCommand (<BehaviorTreeCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BehaviorTreeCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BehaviorTreeCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name behavior_tree_msgs-msg:<BehaviorTreeCommand> is deprecated: use behavior_tree_msgs-msg:BehaviorTreeCommand instead.")))

(cl:ensure-generic-function 'condition_name-val :lambda-list '(m))
(cl:defmethod condition_name-val ((m <BehaviorTreeCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader behavior_tree_msgs-msg:condition_name-val is deprecated.  Use behavior_tree_msgs-msg:condition_name instead.")
  (condition_name m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <BehaviorTreeCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader behavior_tree_msgs-msg:status-val is deprecated.  Use behavior_tree_msgs-msg:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BehaviorTreeCommand>) ostream)
  "Serializes a message object of type '<BehaviorTreeCommand>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'condition_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'condition_name))
  (cl:let* ((signed (cl:slot-value msg 'status)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BehaviorTreeCommand>) istream)
  "Deserializes a message object of type '<BehaviorTreeCommand>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'condition_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'condition_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BehaviorTreeCommand>)))
  "Returns string type for a message object of type '<BehaviorTreeCommand>"
  "behavior_tree_msgs/BehaviorTreeCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BehaviorTreeCommand)))
  "Returns string type for a message object of type 'BehaviorTreeCommand"
  "behavior_tree_msgs/BehaviorTreeCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BehaviorTreeCommand>)))
  "Returns md5sum for a message object of type '<BehaviorTreeCommand>"
  "88f8877408328a1537655cc1377c588d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BehaviorTreeCommand)))
  "Returns md5sum for a message object of type 'BehaviorTreeCommand"
  "88f8877408328a1537655cc1377c588d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BehaviorTreeCommand>)))
  "Returns full string definition for message of type '<BehaviorTreeCommand>"
  (cl:format cl:nil "string condition_name~%int8 status~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BehaviorTreeCommand)))
  "Returns full string definition for message of type 'BehaviorTreeCommand"
  (cl:format cl:nil "string condition_name~%int8 status~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BehaviorTreeCommand>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'condition_name))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BehaviorTreeCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'BehaviorTreeCommand
    (cl:cons ':condition_name (condition_name msg))
    (cl:cons ':status (status msg))
))
