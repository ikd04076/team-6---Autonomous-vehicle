
(cl:in-package :asdf)

(defsystem "f1tenth_gym_ros-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "RaceInfo" :depends-on ("_package_RaceInfo"))
    (:file "_package_RaceInfo" :depends-on ("_package"))
  ))