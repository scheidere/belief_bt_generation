; Auto-generated. Do not edit!


(cl:in-package behavior_tree_msgs-msg)


;//! \htmlinclude BehaviorTreeCommands.msg.html

(cl:defclass <BehaviorTreeCommands> (roslisp-msg-protocol:ros-message)
  ((commands
    :reader commands
    :initarg :commands
    :type (cl:vector behavior_tree_msgs-msg:BehaviorTreeCommand)
   :initform (cl:make-array 0 :element-type 'behavior_tree_msgs-msg:BehaviorTreeCommand :initial-element (cl:make-instance 'behavior_tree_msgs-msg:BehaviorTreeCommand))))
)

(cl:defclass BehaviorTreeCommands (<BehaviorTreeCommands>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BehaviorTreeCommands>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BehaviorTreeCommands)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name behavior_tree_msgs-msg:<BehaviorTreeCommands> is deprecated: use behavior_tree_msgs-msg:BehaviorTreeCommands instead.")))

(cl:ensure-generic-function 'commands-val :lambda-list '(m))
(cl:defmethod commands-val ((m <BehaviorTreeCommands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader behavior_tree_msgs-msg:commands-val is deprecated.  Use behavior_tree_msgs-msg:commands instead.")
  (commands m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BehaviorTreeCommands>) ostream)
  "Serializes a message object of type '<BehaviorTreeCommands>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'commands))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'commands))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BehaviorTreeCommands>) istream)
  "Deserializes a message object of type '<BehaviorTreeCommands>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'commands) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'commands)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'behavior_tree_msgs-msg:BehaviorTreeCommand))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BehaviorTreeCommands>)))
  "Returns string type for a message object of type '<BehaviorTreeCommands>"
  "behavior_tree_msgs/BehaviorTreeCommands")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BehaviorTreeCommands)))
  "Returns string type for a message object of type 'BehaviorTreeCommands"
  "behavior_tree_msgs/BehaviorTreeCommands")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BehaviorTreeCommands>)))
  "Returns md5sum for a message object of type '<BehaviorTreeCommands>"
  "6602df19ee620e103a9a9693540b7d4a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BehaviorTreeCommands)))
  "Returns md5sum for a message object of type 'BehaviorTreeCommands"
  "6602df19ee620e103a9a9693540b7d4a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BehaviorTreeCommands>)))
  "Returns full string definition for message of type '<BehaviorTreeCommands>"
  (cl:format cl:nil "BehaviorTreeCommand[] commands~%================================================================================~%MSG: behavior_tree_msgs/BehaviorTreeCommand~%string condition_name~%int8 status~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BehaviorTreeCommands)))
  "Returns full string definition for message of type 'BehaviorTreeCommands"
  (cl:format cl:nil "BehaviorTreeCommand[] commands~%================================================================================~%MSG: behavior_tree_msgs/BehaviorTreeCommand~%string condition_name~%int8 status~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BehaviorTreeCommands>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'commands) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BehaviorTreeCommands>))
  "Converts a ROS message object to a list"
  (cl:list 'BehaviorTreeCommands
    (cl:cons ':commands (commands msg))
))
