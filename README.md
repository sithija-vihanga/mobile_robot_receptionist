# Mobile Robot Receptionist 2.0

Mobile Robot Receptionist 2.0 is an advanced semi humanoid robot designed to assist in dynamic environments, such as crowded spaces and multi-floor buildings with conventional elevators without using robot specific hardware for elevator access. The project focuses on enhanced human-robot interaction, autonomous navigation, and intelligent task execution and also conversationl capabilities.

---

## Functionality of multi-floor navigation and elevator interactions

https://github.com/user-attachments/assets/eda4a49b-644b-4f4e-a538-aa1e9452c84d

## Features

- **Dynamic Crowd Navigation**: Navigates through crowded spaces using novel navigation algorithm designed as a research work by our team. Crowd navigation is based on ORCA based human trajectory planning and MPC based robot path planning introducing new components like human footprint and prefered walking velocities of human agents to improve human robot interactions.
- **Human-Robot Interaction**: Features conversational abilities and humanoid gestures for engaging with users.
- **Multi-Floor Navigation**: Supports elevator interaction and autonomous navigation between floors using Nav2 stack. Uses behavior trees for automating multi-floor navigation framework and elevator interaction algorithms.
- **Enhanced Mapping**: Uses SLAM for real-time map creation and localization. Robot is mainly designed to work with predefined map in localization settings.
- **Custom Behavior Trees**: Provides sequential navigation goals for dynamic tasks breaking down multi-floor navigation into simpler sub tasks. Multi-floor navigation behavior tree controls a su behavior tree which automates manipulator controls.
- **Integration with YOLO**: Detects and tracks human characteristics to optimize interaction through dynamic obstacle identification and footprint estimation for better path planning.
- **Social Force Model**: Simulates human-like behavior in crowd scenarios for testing the designed navigation algorithm.
- **Datasets**: Custom created data sets for elevator button detection and human categorization for footprint identification.
- **Self-charging capabilities**: Developed a charging dock and docking sequence by monitoring battery levels through the multi-floor navigation behavior tree. 

---

## Hardware setup

| ![Image 1](https://github.com/user-attachments/assets/3f79285c-877b-4bde-973c-abda8a70525b) | ![Image 2](https://github.com/user-attachments/assets/707bb390-5681-467c-8d5b-cda8b000b364) |
|:--------------------------------------------------------------------------------------------:|:-------------------------------------------------------------------------------------------:|
|                                                                                       **Hardware setup**                                                                                   |

### Prerequisites
- **Operating System**: Ubuntu 22.04, Ubuntu 20.04
- **Framework**: ROS 2 Humble Hawksbill
- **Dependencies**:
  - SLAM Toolbox
  - Navigation2 Stack
  - Moveit 2
  - Python 3.10 or higher
  - OpenCV, Matplotlib
  - YOLOv5 or YOLOv8 (Custom Dataset)
