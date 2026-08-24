# RPP-Controller.pro
QT -= gui

TEMPLATE = lib
TARGET = RPPController
DEFINES += RPPCONTROLLER_LIBRARY

CONFIG += c++17

QMAKE_CXXFLAGS += -O3

INCLUDEPATH += /usr/include/eigen3
INCLUDEPATH += $$PWD/..

#set package support if disabled
QT_CONFIG -= no-pkg-config
CONFIG += link_pkgconfig
PKGCONFIG += eigen3

HEADERS += \
    rpp_controller.hpp \
    rpp_collision_checker.hpp \

SOURCES += \
    rpp_controller.cpp \
    rpp_collision_checker.cpp

# Default rules for deployment.
unix {
    target.path = /usr/lib
}
!isEmpty(target.path): INSTALLS += target

unix:!macx: LIBS += -L$$OUT_PWD/../nav2_smac_planner/ -lnav2_smac_planner

INCLUDEPATH += $$PWD/../nav2_smac_planner
DEPENDPATH += $$PWD/../nav2_smac_planner
