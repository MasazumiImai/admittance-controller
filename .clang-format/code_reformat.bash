#!/bin/bash

# Reformat by .clang-format
find ./src/limbed_climbing_robot_admittance_controller -path -o -regex '.*\.\(cpp\|hpp\)'\
  -exec clang-format -style=file:./src/limbed_climbing_robot_admittance_controller/.clang-format/.clang-format -i {} \;

# Reformat by ament_uncrustify
find ./src/limbed_climbing_robot_admittance_controller -path -o -regex '.*\.\(cpp\|hpp\)'\
  -exec ament_uncrustify --reformat {} +
