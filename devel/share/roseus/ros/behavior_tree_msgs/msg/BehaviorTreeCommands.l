;; Auto-generated. Do not edit!


(when (boundp 'behavior_tree_msgs::BehaviorTreeCommands)
  (if (not (find-package "BEHAVIOR_TREE_MSGS"))
    (make-package "BEHAVIOR_TREE_MSGS"))
  (shadow 'BehaviorTreeCommands (find-package "BEHAVIOR_TREE_MSGS")))
(unless (find-package "BEHAVIOR_TREE_MSGS::BEHAVIORTREECOMMANDS")
  (make-package "BEHAVIOR_TREE_MSGS::BEHAVIORTREECOMMANDS"))

(in-package "ROS")
;;//! \htmlinclude BehaviorTreeCommands.msg.html


(defclass behavior_tree_msgs::BehaviorTreeCommands
  :super ros::object
  :slots (_commands ))

(defmethod behavior_tree_msgs::BehaviorTreeCommands
  (:init
   (&key
    ((:commands __commands) (let (r) (dotimes (i 0) (push (instance behavior_tree_msgs::BehaviorTreeCommand :init) r)) r))
    )
   (send-super :init)
   (setq _commands __commands)
   self)
  (:commands
   (&rest __commands)
   (if (keywordp (car __commands))
       (send* _commands __commands)
     (progn
       (if __commands (setq _commands (car __commands)))
       _commands)))
  (:serialization-length
   ()
   (+
    ;; behavior_tree_msgs/BehaviorTreeCommand[] _commands
    (apply #'+ (send-all _commands :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; behavior_tree_msgs/BehaviorTreeCommand[] _commands
     (write-long (length _commands) s)
     (dolist (elem _commands)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; behavior_tree_msgs/BehaviorTreeCommand[] _commands
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _commands (let (r) (dotimes (i n) (push (instance behavior_tree_msgs::BehaviorTreeCommand :init) r)) r))
     (dolist (elem- _commands)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get behavior_tree_msgs::BehaviorTreeCommands :md5sum-) "6602df19ee620e103a9a9693540b7d4a")
(setf (get behavior_tree_msgs::BehaviorTreeCommands :datatype-) "behavior_tree_msgs/BehaviorTreeCommands")
(setf (get behavior_tree_msgs::BehaviorTreeCommands :definition-)
      "BehaviorTreeCommand[] commands
================================================================================
MSG: behavior_tree_msgs/BehaviorTreeCommand
string condition_name
int8 status
")



(provide :behavior_tree_msgs/BehaviorTreeCommands "6602df19ee620e103a9a9693540b7d4a")


