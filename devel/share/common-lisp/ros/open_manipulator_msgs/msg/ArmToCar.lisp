; Auto-generated. Do not edit!


(cl:in-package open_manipulator_msgs-msg)


;//! \htmlinclude ArmToCar.msg.html

(cl:defclass <ArmToCar> (roslisp-msg-protocol:ros-message)
  ((start_time
    :reader start_time
    :initarg :start_time
    :type cl:real
    :initform 0)
   (move
    :reader move
    :initarg :move
    :type cl:boolean
    :initform cl:nil)
   (forward_state
    :reader forward_state
    :initarg :forward_state
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ArmToCar (<ArmToCar>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ArmToCar>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ArmToCar)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name open_manipulator_msgs-msg:<ArmToCar> is deprecated: use open_manipulator_msgs-msg:ArmToCar instead.")))

(cl:ensure-generic-function 'start_time-val :lambda-list '(m))
(cl:defmethod start_time-val ((m <ArmToCar>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader open_manipulator_msgs-msg:start_time-val is deprecated.  Use open_manipulator_msgs-msg:start_time instead.")
  (start_time m))

(cl:ensure-generic-function 'move-val :lambda-list '(m))
(cl:defmethod move-val ((m <ArmToCar>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader open_manipulator_msgs-msg:move-val is deprecated.  Use open_manipulator_msgs-msg:move instead.")
  (move m))

(cl:ensure-generic-function 'forward_state-val :lambda-list '(m))
(cl:defmethod forward_state-val ((m <ArmToCar>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader open_manipulator_msgs-msg:forward_state-val is deprecated.  Use open_manipulator_msgs-msg:forward_state instead.")
  (forward_state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ArmToCar>) ostream)
  "Serializes a message object of type '<ArmToCar>"
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'start_time)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'start_time) (cl:floor (cl:slot-value msg 'start_time)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'move) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'forward_state) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ArmToCar>) istream)
  "Deserializes a message object of type '<ArmToCar>"
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'start_time) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
    (cl:setf (cl:slot-value msg 'move) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'forward_state) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ArmToCar>)))
  "Returns string type for a message object of type '<ArmToCar>"
  "open_manipulator_msgs/ArmToCar")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ArmToCar)))
  "Returns string type for a message object of type 'ArmToCar"
  "open_manipulator_msgs/ArmToCar")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ArmToCar>)))
  "Returns md5sum for a message object of type '<ArmToCar>"
  "4b7661ede9092fffae22ae49adf1666d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ArmToCar)))
  "Returns md5sum for a message object of type 'ArmToCar"
  "4b7661ede9092fffae22ae49adf1666d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ArmToCar>)))
  "Returns full string definition for message of type '<ArmToCar>"
  (cl:format cl:nil "time start_time~%bool move~%bool forward_state~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ArmToCar)))
  "Returns full string definition for message of type 'ArmToCar"
  (cl:format cl:nil "time start_time~%bool move~%bool forward_state~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ArmToCar>))
  (cl:+ 0
     8
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ArmToCar>))
  "Converts a ROS message object to a list"
  (cl:list 'ArmToCar
    (cl:cons ':start_time (start_time msg))
    (cl:cons ':move (move msg))
    (cl:cons ':forward_state (forward_state msg))
))
