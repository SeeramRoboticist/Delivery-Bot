alias nav='roslaunch backend bringup.launch'

alias confirm_bool='rostopic pub -1 /confirmation std_msgs/Bool "data: true"'

alias confirm_kitchen='rostopic pub -1 /confirmation std_msgs/String "data: "confirm_kitchen""'

alias confirm_table='rostopic pub -1 /confirmation std_msgs/String "data: "confirm_table""'

alias cancel='rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID "stamp:
  secs: 0
  nsecs: 0
id: ''" '
