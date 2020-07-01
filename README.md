# RRT* path planning and pure pusuit controller for the F1/10 platform
### This project uses the [2019-skeleton-Code](https://github.com/mlab-upenn/f110-fall2019-skeletons) as a foundation. The build configuration and the simulation was mostly unchanged.

## Anleitung
Um das Projekt auszuführen muss man nur das entsprechende Launch-File starten. Für den Start in der Simulation gibt es das Launch-File *simulator.launch* im Package *racecar\_simulator* und für den Start auf dem Auto gibt es *real.launch* im gleichen Package. Um auch weiterhin unsere Visualisierung der Algorithmen in Rviz zu sehen, kann man ein ROS-Netzwerk über mehrere Computer aufbauen. Dafür müssen auf dem Auto die *$ROS\_MASTER\_URI* und *$ROS\_IP* auf die IP des Autos gesetzt werden. Auf dem anderen Computer im gleichen Netzwerk muss dann entsprechend ebenfalls *$ROS\_MASTER\_URI* auf die IP des Autos gesetzt werden. Dann kann rviz auf diesem Computer mit der Konfigurationsdatei *simulator.rviz* (Package: *racecar\_simulator*) gestartet werden und sollte nun alle Topics und Visualisierungen wie gewohnt anzeigen.

Um eine neue Karte mit Hector SLAM zu generieren, sollte man zunächst *teleop.launch* aus dem 2018-Skeleton ausführen. Das ist notwendig, um die Lidar-Node und die Steuerung über den Controller zu starten. Mit dem 2019-Skeleton hatten wir hier leider Probleme mit den Koordinatentranformationen, die mit dem älteren Skeleton nicht aufgetreten sind. Anschließend kann man *tutorial.launch* aus dem *hector\_slam\_launch* package starten um eine Karte zu generieren. In diesem Launch-File kann auch eingestellt werden, ob die Karte direkt anhand von den aktuellen Lidar-Daten generiert werden soll oder anhand von einer vorher aufgenommenen rosbag.
