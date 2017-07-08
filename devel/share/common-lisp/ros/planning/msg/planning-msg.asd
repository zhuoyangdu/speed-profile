
(cl:in-package :asdf)

(defsystem "planning-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ObstacleMap" :depends-on ("_package_ObstacleMap"))
    (:file "_package_ObstacleMap" :depends-on ("_package"))
    (:file "DynamicObstacle" :depends-on ("_package_DynamicObstacle"))
    (:file "_package_DynamicObstacle" :depends-on ("_package"))
    (:file "Pose" :depends-on ("_package_Pose"))
    (:file "_package_Pose" :depends-on ("_package"))
    (:file "Trajectory" :depends-on ("_package_Trajectory"))
    (:file "_package_Trajectory" :depends-on ("_package"))
  ))