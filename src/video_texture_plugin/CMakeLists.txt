cmake_minimum_required(VERSION 3.10)
project(video_texture_plugin)

# 1. Recherche des dépendances
find_package(ament_cmake REQUIRED) # INDISPENSABLE pour ROS 2
find_package(gz-sim8 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core videoio imgproc)

# 2. Définition du plugin
add_library(ros_video_texture_plugin SHARED
  src/ros_video_texture_plugin.cpp
)

# 3. Inclusion des headers
target_include_directories(ros_video_texture_plugin PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)

# 4. Lien avec les bibliothèques
target_link_libraries(ros_video_texture_plugin
  PRIVATE
    gz-sim8::gz-sim8
    gz-msgs10::gz-msgs10
    gz-transport13::gz-transport13
    gz-plugin2::gz-plugin2
    ${OpenCV_LIBRARIES}
)

# 5. Installation du plugin dans lib/
install(TARGETS ros_video_texture_plugin
  DESTINATION lib/${PROJECT_NAME}
)

# 6. Signalisation à ROS 2 (Crée le local_setup.bash)
ament_package()