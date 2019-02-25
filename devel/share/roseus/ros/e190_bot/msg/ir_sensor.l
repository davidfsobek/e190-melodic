;; Auto-generated. Do not edit!


(when (boundp 'e190_bot::ir_sensor)
  (if (not (find-package "E190_BOT"))
    (make-package "E190_BOT"))
  (shadow 'ir_sensor (find-package "E190_BOT")))
(unless (find-package "E190_BOT::IR_SENSOR")
  (make-package "E190_BOT::IR_SENSOR"))

(in-package "ROS")
;;//! \htmlinclude ir_sensor.msg.html


(defclass e190_bot::ir_sensor
  :super ros::object
  :slots (_distance ))

(defmethod e190_bot::ir_sensor
  (:init
   (&key
    ((:distance __distance) 0.0)
    )
   (send-super :init)
   (setq _distance (float __distance))
   self)
  (:distance
   (&optional __distance)
   (if __distance (setq _distance __distance)) _distance)
  (:serialization-length
   ()
   (+
    ;; float32 _distance
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _distance
       (sys::poke _distance (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _distance
     (setq _distance (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get e190_bot::ir_sensor :md5sum-) "6e77fb10f0c8b4833ec273aa9ac74459")
(setf (get e190_bot::ir_sensor :datatype-) "e190_bot/ir_sensor")
(setf (get e190_bot::ir_sensor :definition-)
      "float32 distance

")



(provide :e190_bot/ir_sensor "6e77fb10f0c8b4833ec273aa9ac74459")


