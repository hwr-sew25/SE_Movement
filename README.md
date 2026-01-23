# movement_api

Dieses ROS1-Package stellt eine einfache API bereit, über die andere Teams
(Z. B. *Directions* oder *POI*) dem TurtleBot3 Navigationsziele übergeben können.
Die Bewegung erfolgt über `move_base`, während das Package die
Kommunikation und das Handling der Targets kapselt.

Das Package kann sowohl in der Simulation (Gazebo) als auch auf dem echten
Roboter genutzt werden


mit diesen Befeheln im terminal wird die Ordnerstruktur aus Github lokal gespeichert und der workspace neu gebaut. Eventuell muss die ordnerstruktur in Windows/mac heruntergeladen werden wenn keine verbindung zum repository möglich ist. Dann einfach in ubuntu kopieren und den catkin make befehle ausführen

cd ~/catkin_ws/src
git clone https://github.com/Jorjeque/movement_api.git
cd ~/catkin_ws
catkin_make

im anschluss reicht es die launch datei auszuführen. Diese lässt sowhl rviz mit der karte als auch gazebo laufen sowie den mapserver und die navigation

source ~/catkin_ws/devel/setup.bash
roslaunch movement_api full_sim_setup.launch


im anschluss nach der positionierung des roboters kann durch diesen beispielbefehl die navigation geteste werden. das simuliert theoretisch eine ausgabe die entssteht wenn durch eine benutzereingabe daten aus der datenbank gezoen werden weil der roboter zu einem Ziel möchte.

source ~/catkin_ws/devel/setup.bash
rosservice call /movement/go_to_pose "x: 18.3
y: -39.2
yaw_deg: 0
frame_id: 'map'"

