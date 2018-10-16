INCLUDEPATH += "/home/sstepput/LongTermHRI/ros_ws/devel/include/"
INCLUDEPATH += "/opt/ros/kinetic/include/"

DISTFILES += \
    CMakeLists.txt

SOURCES += \
    src/ur5/main.cpp \
    src/ur5/ur5network.cpp \
    src/ur5/ur5ros.cpp \
    src/ur5/ur5controller.cpp \
    src/r2fg/main.cpp \
    src/r2fg/r2fgripper.cpp \
    src/r3fg/main.cpp \
    src/r3fg/r3fgripper.cpp \
    src/r3fg/r3fgros.cpp \
    src/r3fg/r3fgnetwork.cpp \
    src/ur5_vrep_ik/main.cpp \
    src/ur5_vrep_ik/vrepcontroller.cpp \
    src/ur5_vrep_ik/vrepros.cpp \
    src/ur5_vrep_sim/main.cpp \
    src/ur5_vrep_sim/vrepcontroller.cpp \
    src/ur5_vrep_sim/vrepros.cpp \
    src/ur5_continuous_controller/ur5_continuous_controller.cpp \
    src/ur5_continuous_controller/main.cpp

HEADERS += \
    src/ur5/ur5network.h \
    src/ur5/ur5ros.h \
    src/ur5/ur5controller.h \
    src/r2fg/r2fgripper.h \
    src/r3fg/r3fgripper.h \
    src/r3fg/r3fgros.h \
    src/r3fg/r3fgnetwork.h \
    src/ur5_vrep_ik/vrepcontroller.h \
    src/ur5_vrep_ik/vrepros.h \
    src/ur5_vrep_sim/vrepcontroller.h \
    src/ur5_vrep_sim/vrepros.h \
    src/ur5_continuous_controller/ur5_continuous_controller.h
