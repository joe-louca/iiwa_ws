;; Auto-generated. Do not edit!


(when (boundp 'iiwa_msgs::MoveAlongSplineFeedback)
  (if (not (find-package "IIWA_MSGS"))
    (make-package "IIWA_MSGS"))
  (shadow 'MoveAlongSplineFeedback (find-package "IIWA_MSGS")))
(unless (find-package "IIWA_MSGS::MOVEALONGSPLINEFEEDBACK")
  (make-package "IIWA_MSGS::MOVEALONGSPLINEFEEDBACK"))

(in-package "ROS")
;;//! \htmlinclude MoveAlongSplineFeedback.msg.html


(defclass iiwa_msgs::MoveAlongSplineFeedback
  :super ros::object
  :slots ())

(defmethod iiwa_msgs::MoveAlongSplineFeedback
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(setf (get iiwa_msgs::MoveAlongSplineFeedback :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get iiwa_msgs::MoveAlongSplineFeedback :datatype-) "iiwa_msgs/MoveAlongSplineFeedback")
(setf (get iiwa_msgs::MoveAlongSplineFeedback :definition-)
      "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
# Feedback


")



(provide :iiwa_msgs/MoveAlongSplineFeedback "d41d8cd98f00b204e9800998ecf8427e")


