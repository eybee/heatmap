# heatmap

ROS package for automated creation of visual WiFi heatmaps.
It uses RViz for visualizing the heatmap at the moment, image file export is planned for future releases.  

For coverage path planning it uses the "Moe the autonomous lawnmower" planner:  
https://github.com/Auburn-Automow/au_automow_common

It also uses the RViz point to polygon user interface from the frontier_exploration package by Paul Bovbel:    https://github.com/paulbovbel/frontier_exploration  
  
## Installation  
The path planner depends on the "shapely" python module.  
Learn how to install it here:  
https://pypi.python.org/pypi/Shapely  

