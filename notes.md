#Notes from Course Development

## OCU search
-[rviz-like control panel with point-click publishing](https://github.com/swri-robotics/mapviz)
  - installed and it mostly worked
  - had to install some plugins separately from initial install using `sudo apt-get install ros-indigo-<plugin_name>` including tile_map and multires_image
  - point-click publishing works (note: also works in rviz)
  - still working on displaying map

## Arduino-Odroid communication
- Serial communication using JSON messages
  - [JSON library](https://github.com/bblanchon/ArduinoJson) for both sides
  - [Serial library](https://github.com/siggiorn/arduino-buffered-serial) for arduino side
