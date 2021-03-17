;; Auto-generated. Do not edit!


(when (boundp 'behavior_tree_msgs::Status)
  (if (not (find-package "BEHAVIOR_TREE_MSGS"))
    (make-package "BEHAVIOR_TREE_MSGS"))
  (shadow 'Status (find-package "BEHAVIOR_TREE_MSGS")))
(unless (find-package "BEHAVIOR_TREE_MSGS::STATUS")
  (make-package "BEHAVIOR_TREE_MSGS::STATUS"))

(in-package "ROS")
;;//! \htmlinclude Status.msg.html


(intern "*FAILURE*" (find-package "BEHAVIOR_TREE_MSGS::STATUS"))
(shadow '*FAILURE* (find-package "BEHAVIOR_TREE_MSGS::STATUS"))
(defconstant behavior_tree_msgs::Status::*FAILURE* 0)
(intern "*RUNNING*" (find-package "BEHAVIOR_TREE_MSGS::STATUS"))
(shadow '*RUNNING* (find-package "BEHAVIOR_TREE_MSGS::STATUS"))
(defconstant behavior_tree_msgs::Status::*RUNNING* 1)
(intern "*SUCCESS*" (find-package "BEHAVIOR_TREE_MSGS::STATUS"))
(shadow '*SUCCESS* (find-package "BEHAVIOR_TREE_MSGS::STATUS"))
(defconstant behavior_tree_msgs::Status::*SUCCESS* 2)
(defclass behavior_tree_msgs::Status
  :super ros::object
  :slots (_status _id ))

(defmethod behavior_tree_msgs::Status
  (:init
   (&key
    ((:status __status) 0)
    ((:id __id) 0)
    )
   (send-super :init)
   (setq _status (round __status))
   (setq _id (round __id))
   self)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:serialization-length
   ()
   (+
    ;; int8 _status
    1
    ;; uint64 _id
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _status
       (write-byte _status s)
     ;; uint64 _id
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _id (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _id) (= (length (_id . bv)) 2)) ;; bignum
              (write-long (ash (elt (_id . bv) 0) 0) s)
              (write-long (ash (elt (_id . bv) 1) -1) s))
             ((and (class _id) (= (length (_id . bv)) 1)) ;; big1
              (write-long (elt (_id . bv) 0) s)
              (write-long (if (>= _id 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _id s)(write-long (if (>= _id 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _status
     (setq _status (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _status 127) (setq _status (- _status 256)))
   ;; uint64 _id
     
#+(or :alpha :irix6 :x86_64)
      (setf _id (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _id (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(setf (get behavior_tree_msgs::Status :md5sum-) "ff468a49be3012e35a4261c2990f465a")
(setf (get behavior_tree_msgs::Status :datatype-) "behavior_tree_msgs/Status")
(setf (get behavior_tree_msgs::Status :definition-)
      "int8 FAILURE=0
int8 RUNNING=1
int8 SUCCESS=2

int8 status
uint64 id
")



(provide :behavior_tree_msgs/Status "ff468a49be3012e35a4261c2990f465a")


