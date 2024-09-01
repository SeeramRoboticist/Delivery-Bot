Delivery Bot
-------------------------------------------------------


Setting up Software:
-------------------------------------------------------

- Git clone this repo and rename it as src.
- Build it as a package inside a ros_ws.
- For building, run a catkin_make.
- Once done, source ros and this work space.

- Open a terminal and copy paste the below command
``` 
roscd backend/../resources/
```
- now you will be in the resources folder 

- Open a terminal and copy paste the below command
``` 
cp .bash_aliases ~/
```
- Now close all the terminal and source the ws again else add it in .bashrc

- open terminal
```
nav
```
- This will bringup the turtlebot3 gazebo simulation and navigation with rviz.

- The above mentioned setup will be common for all the tasks
-------------------------------------------------------

**Task 1**

**When an order is received with the table number, the robot should move from its home
position to the kitchen and move to the table for the food delivery. After completion of
that task the robot should return to the home position. (No confirmation is required either
from the kitchen or customer table. Getting input from the table or kitchen is your choice.)**

```
rosrun backend task1.py
```
```
rosrun backend order.py 1
```
- we can see the robot doing the task asked.

-------------------------------------------------------

**Task 2**

**When an order is received with the table number, the robot should move from home to
start its task. If no one attends the robot, it will wait for Confirmation (either in kitchen or
table), the robot should return home after timeout.**

```
rosrun backend task2.py
```
```
rosrun backend order.py 1
```
- The robot will move from home to kitchen, if no one attends it will return to home.

- when the process in the kitchen, run the below command in terminal

```
confirm_bool
```
- after execution of this command robot will move to home.

-------------------------------------------------------

**Task 3**

**When an order is received with the table number, the robot should move from home to
start its task. We need to handle the following scenario**
- **It will reach the kitchen for the food and if no confirmation is given to the robot it
should move to the home position after timeout.**
 - **If the food is received from the kitchen, it reaches the table. No one is giving
confirmation to the robot from the table, then the robot will move to the kitchen
first before going to the home position.**

```
rosrun backend task3.py
```
```
rosrun backend order.py 1
```
- The robot will move from home to kitchen, if no one attends it will return to home.
- when the process is in kitchen, we run the following command
```
confirm_kitchen
```
- which will mark confirmation from kitchen and move to table.

- After reaching table if no one attends it will return from table -> kitchen -> home.

- if someone attends, run the following command

while the current process is in table run the below command, it will take the robot directly to home position.
```
confirm_table
```
-------------------------------------------------------
**Task 4**

**When an order is received with the table number, the robot should move from home to
start its task. If a task is canceled while going to the table, the robot returns to the kitchen
and then to home and if canceled while going to the kitchen, the robot will return to home.**

```
rosrun backend task4.py
```
```
rosrun backend order.py 1
```
- when you run the above command the robot will move to kitchen if you run the below command
```
cancel
```
- the robot will stop and return to home position

- again you can run order.py 1

- when the process is in kitchen, we run the following command
```
confirm_kitchen
```

- this will take the robot to table, again if you run "cancel" command, you can see the robot moving back to kitchen and to home.

-------------------------------------------------------
**Task 5**

**When multiple orders are received with the table numbers, the robot should move from
home position to kitchen to collect the orders and move to multiple tables for the food
delivery. After completion of that task (delivery of all the tables) the robot should return
to the home position.**

```
rosrun backend task5.py
```
```
rosrun backend order.py 1 2 3
```
- when you run the above command the robot will move to kitchen and to all table and deliver food and return to home positon.

-------------------------------------------------------

**Task 6**

**When multiple orders are received with the table numbers (let us assume table1, table2,
table3), the robot should move from home position to kitchen to collect the orders and
move to multiple tables for the food delivery. When no one confirms in table1 the robot
should move to the next tables (table2, table3) for delivery. After finishing the delivery of
the final table, the robot goes to the kitchen before going to the home position.**

```
rosrun backend task6.py
```
```
rosrun backend order.py 3 1 2
```


- when the process is in kitchen, we run the following command, else it will timeout and return to home.
```
confirm_kitchen
```
- once the robot reaches the table there will a yellow log warn, that time you can run the below command it will go to next table, else after timeout it will go to the next table.
- if you run the below command all the time the warning comes, means all the tables got there orders the robot will return to home, else kitchen then home.
```
confirm_table
```
-------------------------------------------------------
-------------------------------------------------------

**Task 7**

**When multiple orders are received with the table numbers (let us assume table1, table2,
table3), the robot should move from home position to kitchen to collect the orders and
move to multiple tables for the food delivery. The order of the particular table (table2) is
canceled, the robot should skip that table (table2) and deliver to the other tables (table1,
table3). After finishing the delivery of the final table, the robot goes to the kitchen before
going to the home position.**

```
rosrun backend task7.py
```
```
rosrun backend order.py 2 1 3
```


- when the process is in kitchen, we run the following command, else it will timeout and return to home.
```
confirm_kitchen
```
- once the robot reaches the table there will a yellow log warn, that time you can run the below command it will go to next table, else after timeout it will go to the next table.
- if you run the below command all the time the warning comes, means all the tables got there orders the robot will return to home, else kitchen then home.
```
confirm_table
```
- if you want to cancell the order while the robot is moving, simply open terminal and the below command
```
cancel
```
- this command will stop the process of deliverying to that table and move to next table.


