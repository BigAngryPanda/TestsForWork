QT += gui
QT += widgets
QT += core

win32 {
    QMAKE_CXXFLAGS += /std:c++20
}
unix {
    QMAKE_CXXFLAGS += -std=c++2a
}

HEADERS += src/mainwindow.hpp \
		   src/arch.hpp \
		   src/core.hpp

SOURCES += src/main.cpp \
		   src/mainwindow.cpp \
		   src/core.cpp

TARGET = disk_stats
