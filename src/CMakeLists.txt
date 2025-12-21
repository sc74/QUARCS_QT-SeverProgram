cmake_minimum_required(VERSION 3.14)

project(client LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(QT_COMPONENTS
  Core
  Gui
  Widgets
  Concurrent
  Network
  WebSockets
  LinguistTools
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 仅使用 Qt5（去掉 Qt6）
find_package(Qt6 REQUIRED COMPONENTS ${QT_COMPONENTS} SerialPort)

find_package(OpenCV REQUIRED)
find_package(StellarSolver REQUIRED)

add_executable(client
  main.cpp
  websocketclient.h websocketclient.cpp
  mainwindow.h mainwindow.cpp
  tools.h tools.cpp
  focused_star_detection.cpp
  myclient.h myclient.cpp
  websocketthread.h websocketthread.cpp
  Logger.h Logger.cpp
  autopolaralignment.h autopolaralignment.cpp
  autofocus.h autofocus.cpp
  starsimulator.h starsimulator.cpp
  mountstate.h
  SerialDeviceDetector.h SerialDeviceDetector.cpp
)

target_link_libraries(client PRIVATE
    indiclient ${ZLIB_LIBRARY} ${NOVA_LIBRARIES}
)

IF (UNIX AND NOT APPLE)
    target_link_libraries(client PRIVATE -lpthread)
ENDIF ()

include(GNUInstallDirs)
install(TARGETS client
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

target_include_directories(client PUBLIC
  /usr/include/libindi
  /usr/include/libstellarsolver
)

link_directories(${PROJECT_SOURCE_DIR}/usr/lib/)

target_link_libraries(client PUBLIC
  -lnova
  -lindidriver
  -lindiclient
  -lcfitsio
)

target_include_directories(client PUBLIC
    /usr/include
)

target_link_libraries(client PRIVATE
    qhyccd
    stellarsolver6
)

target_include_directories(client PRIVATE Qt::Core)

# 这里全部固定为 Qt5::
target_link_libraries(client PRIVATE
    ${OpenCV_LIBS}
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
    Qt6::Concurrent
    Qt6::Network
    Qt6::WebSockets
    Qt6::SerialPort
    ${StellarSolver_LIBS}
)

# 复制配置文件到构建目录
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/../qhyccd.ini
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
