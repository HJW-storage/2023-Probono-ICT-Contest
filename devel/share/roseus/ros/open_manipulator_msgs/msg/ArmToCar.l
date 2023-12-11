;; Auto-generated. Do not edit!


(when (boundp 'open_manipulator_msgs::ArmToCar)
  (if (not (find-package "OPEN_MANIPULATOR_MSGS"))
    (make-package "OPEN_MANIPULATOR_MSGS"))
  (shadow 'ArmToCar (find-package "OPEN_MANIPULATOR_MSGS")))
(unless (find-package "OPEN_MANIPULATOR_MSGS::ARMTOCAR")
  (make-package "OPEN_MANIPULATOR_MSGS::ARMTOCAR"))

(in-package "ROS")
;;//! \htmlinclude ArmToCar.msg.html


(defclass open_manipulator_msgs::ArmToCar
  :super ros::object
  :slots (_start_time _move _forward_state ))

(defmethod open_manipulator_msgs::ArmToCar
  (:init
   (&key
    ((:start_time __start_time) (instance ros::time :init))
    ((:move __move) nil)
    ((:forward_state __forward_state) nil)
    )
   (send-super :init)
   (setq _start_time __start_time)
   (setq _move __move)
   (setq _forward_state __forward_state)
   self)
  (:start_time
   (&optional __start_time)
   (if __start_time (setq _start_time __start_time)) _start_time)
  (:move
   (&optional (__move :null))
   (if (not (eq __move :null)) (setq _move __move)) _move)
  (:forward_state
   (&optional (__forward_state :null))
   (if (not (eq __forward_state :null)) (setq _forward_state __forward_state)) _forward_state)
  (:serialization-length
   ()
   (+
    ;; time _start_time
    8
    ;; bool _move
    1
    ;; bool _forward_state
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _start_time
       (write-long (send _start_time :sec) s) (write-long (send _start_time :nsec) s)
     ;; bool _move
       (if _move (write-byte -1 s) (write-byte 0 s))
     ;; bool _forward_state
       (if _forward_state (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _start_time
     (send _start_time :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _start_time :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _move
     (setq _move (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _forward_state
     (setq _forward_state (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get open_manipulator_msgs::ArmToCar :md5sum-) "4b7661ede9092fffae22ae49adf1666d")
(setf (get open_manipulator_msgs::ArmToCar :datatype-) "open_manipulator_msgs/ArmToCar")
(setf (get open_manipulator_msgs::ArmToCar :definition-)
      "time start_time
bool move
bool forward_state
")



(provide :open_manipulator_msgs/ArmToCar "4b7661ede9092fffae22ae49adf1666d")


