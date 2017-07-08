; Auto-generated. Do not edit!


(cl:in-package planning-msg)


;//! \htmlinclude ObstacleMap.msg.html

(cl:defclass <ObstacleMap> (roslisp-msg-protocol:ros-message)
  ((dynamic_obstacles
    :reader dynamic_obstacles
    :initarg :dynamic_obstacles
    :type (cl:vector planning-msg:DynamicObstacle)
   :initform (cl:make-array 0 :element-type 'planning-msg:DynamicObstacle :initial-element (cl:make-instance 'planning-msg:DynamicObstacle))))
)

(cl:defclass ObstacleMap (<ObstacleMap>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ObstacleMap>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ObstacleMap)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name planning-msg:<ObstacleMap> is deprecated: use planning-msg:ObstacleMap instead.")))

(cl:ensure-generic-function 'dynamic_obstacles-val :lambda-list '(m))
(cl:defmethod dynamic_obstacles-val ((m <ObstacleMap>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader planning-msg:dynamic_obstacles-val is deprecated.  Use planning-msg:dynamic_obstacles instead.")
  (dynamic_obstacles m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ObstacleMap>) ostream)
  "Serializes a message object of type '<ObstacleMap>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'dynamic_obstacles))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'dynamic_obstacles))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ObstacleMap>) istream)
  "Deserializes a message object of type '<ObstacleMap>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'dynamic_obstacles) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'dynamic_obstacles)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'planning-msg:DynamicObstacle))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ObstacleMap>)))
  "Returns string type for a message object of type '<ObstacleMap>"
  "planning/ObstacleMap")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ObstacleMap)))
  "Returns string type for a message object of type 'ObstacleMap"
  "planning/ObstacleMap")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ObstacleMap>)))
  "Returns md5sum for a message object of type '<ObstacleMap>"
  "53002d3079c88d059fd08d0aaa769907")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ObstacleMap)))
  "Returns md5sum for a message object of type 'ObstacleMap"
  "53002d3079c88d059fd08d0aaa769907")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ObstacleMap>)))
  "Returns full string definition for message of type '<ObstacleMap>"
  (cl:format cl:nil "DynamicObstacle[] dynamic_obstacles~%================================================================================~%MSG: planning/DynamicObstacle~%uint64 timestamp~%string id~%float64 x~%float64 y~%float64 theta~%float64 velocity~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ObstacleMap)))
  "Returns full string definition for message of type 'ObstacleMap"
  (cl:format cl:nil "DynamicObstacle[] dynamic_obstacles~%================================================================================~%MSG: planning/DynamicObstacle~%uint64 timestamp~%string id~%float64 x~%float64 y~%float64 theta~%float64 velocity~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ObstacleMap>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'dynamic_obstacles) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ObstacleMap>))
  "Converts a ROS message object to a list"
  (cl:list 'ObstacleMap
    (cl:cons ':dynamic_obstacles (dynamic_obstacles msg))
))
