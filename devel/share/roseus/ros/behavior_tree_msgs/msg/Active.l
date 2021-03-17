;; Auto-generated. Do not edit!


(when (boundp 'behavior_tree_msgs::Active)
  (if (not (find-package "BEHAVIOR_TREE_MSGS"))
    (make-package "BEHAVIOR_TREE_MSGS"))
  (shadow 'Active (find-package "BEHAVIOR_TREE_MSGS")))
(unless (find-package "BEHAVIOR_TREE_MSGS::ACTIVE")
  (make-package "BEHAVIOR_TREE_MSGS::ACTIVE"))

(in-package "ROS")
;;//! \htmlinclude Active.msg.html


(defclass behavior_tree_msgs::Active
  :super ros::object
  :slots (_active _id ))

(defmethod behavior_tree_msgs::Active
  (:init
   (&key
    ((:active __active) nil)
    ((:id __id) 0)
    )
   (send-super :init)
   (setq _active __active)
   (setq _id (round __id))
   self)
  (:active
   (&optional __active)
   (if __active (setq _active __active)) _active)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:serialization-length
   ()
   (+
    ;; bool _active
    1
    ;; int64 _id
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _active
       (if _active (write-byte -1 s) (write-byte 0 s))
     ;; int64 _id
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
   ;; bool _active
     (setq _active (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int64 _id
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

(setf (get behavior_tree_msgs::Active :md5sum-) "c07e551e186876ca3af3ff1a8f330ec9")
(setf (get behavior_tree_msgs::Active :datatype-) "behavior_tree_msgs/Active")
(setf (get behavior_tree_msgs::Active :definition-)
      "bool active
int64 id
")



(provide :behavior_tree_msgs/Active "c07e551e186876ca3af3ff1a8f330ec9")


