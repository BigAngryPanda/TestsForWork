# Controls

Click button to change file tree view

Click on folder name to collect stats

In both cases statistics will be recollected

# Files

`main.cpp`

Entry point

`mainwindow.hpp/.cpp`

Window class

Contains all view logic as well as handle user's input

`core.hpp/.cpp`

Responsible for collecting filesystem stats

`arch.hpp`

OS-specific definitions

`build.pro.user`

Specific file for qt studio

You may delete it

# Build

## Linux
```
mkdir build
cd build
qmake ../build.pro
make
```

## Windows

I used msvc 2019 in qt creator

## Dependencies

## Linux
`qt5core qt5widgets qt5gui`

All runtime dependencies listed below:
```
linux-vdso.so.1
libQt5Widgets.so.5
libQt5Core.so.5
libstdc++.so.6
libgcc_s.so.1
libc.so.6
libQt5Gui.so.5
libm.so.6
libz.so.1
libdouble-conversion.so.3
libicui18n.so.70
libicuuc.so.70
libpcre2-16.so.0
libzstd.so.1
libglib-2.0.so.0
/lib64/ld-linux-x86-64.so.2
libGL.so.1
libpng16.so.16
libharfbuzz.so.0
libmd4c.so.0
libicudata.so.70
libpcre.so.3
libGLdispatch.so.0
libGLX.so.0
libfreetype.so.6
libgraphite2.so.3
libX11.so.6
libbrotlidec.so.1
libxcb.so.1
libbrotlicommon.so.1
libXau.so.6
libXdmcp.so.6
libbsd.so.0
libmd.so.0
```

## Windows

For compiling I installed qt via qt online installer

For runtime dll and plugins are in build folder with exe file

## Note
Programm counts all subdirs (subfolders) within directory (folder) not only first level