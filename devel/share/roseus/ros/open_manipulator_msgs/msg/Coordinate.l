;; Auto-generated. Do not edit!


(when (boundp 'open_manipulator_msgs::Coordinate)
  (if (not (find-package "OPEN_MANIPULATOR_MSGS"))
    (make-package "OPEN_MANIPULATOR_MSGS"))
  (shadow 'Coordinate (find-package "OPEN_MANIPULATOR_MSGS")))
(unless (find-package "OPEN_MANIPULATOR_MSGS::COORDINATE")
  (make-package "OPEN_MANIPULATOR_MSGS::COORDINATE"))

(in-package "ROS")
;;//! \htmlinclude Coordinate.msg.html


(defclass open_manipulator_msgs::Coordinate
  :super ros::object
  :slots (_start_time _msg_seq _x _y _z _distance ))

(defmethod open_manipulator_msgs::Coordinate
  (:init
   (&key
    ((:start_time __start_time) (instance ros::time :init))
    ((:msg_seq __msg_seq) 0)
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    ((:distance __distance) 0.0)
    )
   (send-super :init)
   (setq _start_time __start_time)
   (setq _msg_seq (round __msg_seq))
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   (setq _distance (float __distance))
   self)
  (:start_time
   (&optional __start_time)
   (if __start_time (setq _start_time __start_time)) _start_time)
  (:msg_seq
   (&optional __msg_seq)
   (if __msg_seq (setq _msg_seq __msg_seq)) _msg_seq)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:distance
   (&optional __distance)
   (if __distance (setq _distance __distance)) _distance)
  (:serialization-length
   ()
   (+
    ;; time _start_time
    8
    ;; uint16 _msg_seq
    2
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _z
    8
    ;; float64 _distance
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _start_time
       (write-long (send _start_time :sec) s) (write-long (send _start_time :nsec) s)
     ;; uint16 _msg_seq
       (write-word _msg_seq s)
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _z
       (sys::poke _z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _distance
       (sys::poke _distance (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _start_time
     (send _start_time :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _start_time :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint16 _msg_seq
     (setq _msg_seq (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _z
     (setq _z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _distance
     (setq _distance (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get open_manipulator_msgs::Coordinate :md5sum-) "61deae5df730e150ae50b08d27dbc08d")
(setf (get open_manipulator_msgs::Coordinate :datatype-) "open_manipulator_msgs/Coordinate")
(setf (get open_manipulator_msgs::Coordinate :definition-)
      "time start_time
uint16 msg_seq
float64 x
float64 y
float64 z
float64 distance

")



(provide :open_manipulator_msgs/Coordinate "61deae5df730e150ae50b08d27dbc08d")


