# MiAPR_Project
Implementacja algorytmu RRT* do planowania ruchu pojazdu o kinematyce samochodowej, wykorzystując krzywe Dubinsa, na mapie zajętości. 

Aby uruchomić projekt w rvizie należy:
- utworzyć folder projekt_miapr, w którym znajdować się muszą foldery launch, maps, src oraz pliki CMakeLists.txt i package.xml
- folder z projektem (projekt_miapr) umieścić w folderze catkin_ws/src
- w terminalu otworzyć lokalizację catkin_ws (source devel/setup.bash)
- użyć komendy `roslaunch projekt_miapr rrt_star.launch`
- w rvizie dodać odpowiednie topici

Aby zmienić mapę na inną z dostępnych należy w pliku rrt_star.launch w 2 linijce zmienić nazwę pliku launch do uruchamiania odpowiedniej mapy
