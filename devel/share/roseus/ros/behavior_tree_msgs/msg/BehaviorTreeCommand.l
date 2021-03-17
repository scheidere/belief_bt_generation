;; Auto-generated. Do not edit!


(when (boundp 'behavior_tree_msgs::BehaviorTreeCommand)
  (if (not (find-package "BEHAVIOR_TREE_MSGS"))
    (make-package "BEHAVIOR_TREE_MSGS"))
  (shadow 'BehaviorTreeCommand (find-package "BEHAVIOR_TREE_MSGS")))
(unless (find-package "BEHAVIOR_TREE_MSGS::BEHAVIORTREECOMMAND")
  (make-package "BEHAVIOR_TREE_MSGS::BEHAVIORTREECOMMAND"))

(in-package "ROS")
;;//! \htmlinclude BehaviorTreeCommand.msg.html


(defclass behavior_tree_msgs::BehaviorTreeCommand
  :super ros::object
  :slots (_condition_name _status ))

(defmethod behavior_tree_msgs::BehaviorTreeCommand
  (:init
   (&key
    ((:condition_name __condition_name) "")
    ((:status __status) 0)
    )
   (send-super :init)
   (setq _condition_name (string __condition_name))
   (setq _status (round __status))
   self)
  (:condition_name
   (&optional __condition_name)
   (if __condition_name (setq _condition_name __condition_name)) _condition_name)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:serialization-length
   ()
   (+
    ;; string _condition_name
    4 (length _condition_name)
    ;; int8 _status
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _condition_name
       (write-long (length _condition_name) s) (princ _condition_name s)
     ;; int8 _status
       (write-byte _status s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _condition_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _condition_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int8 _status
     (setq _status (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _status 127) (setq _status (- _status 256)))
   ;;
   self)
  )

(setf (get behavior_tree_msgs::BehaviorTreeCommand :md5sum-) "88f8877408328a1537655cc1377c588d")
(setf (get behavior_tree_msgs::BehaviorTreeCommand :datatype-) "behavior_tree_msgs/BehaviorTreeCommand")
(setf (get behavior_tree_msgs::BehaviorTreeCommand :definition-)
      "string condition_name
int8 status
")



(provide :behavior_tree_msgs/BehaviorTreeCommand "88f8877408328a1537655cc1377c588d")


