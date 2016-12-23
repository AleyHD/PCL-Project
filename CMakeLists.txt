CMAKE_MINIMUM_REQUIRED (VERSION 2.6 FATAL_ERROR)

SET (PROJECT_NAME PCL-Project)                 # Defines the project name

PROJECT (${PROJECT_NAME})

FIND_PACKAGE (Qt4 REQUIRED)
FIND_PACKAGE (VTK REQUIRED)
FIND_PACKAGE (PCL 1.8 REQUIRED)

INCLUDE_DIRECTORIES (${PCL_INCLUDE_DIRS})
LINK_DIRECTORIES    (${PCL_LIBRARY_DIRS})
ADD_DEFINITIONS     (${PCL_DEFINITIONS})

SET (PROJECT_SOURCES  main.cpp                # Add cpp Files
                      mainwindow.cpp
                      controlbar.cpp
                      Libs/pointcloud.cpp
                      Libs/pointcloudvisualizer.cpp
                      Libs/cropbox.cpp
                      )
SET (PROJECT_HEADERS  mainwindow.h            # Add h Files
                      controlbar.h
                      Libs/pointcloud.h
                      Libs/pointcloudvisualizer.h
                      Libs/cropbox.h
                      )
SET (PROJECT_FORMS    mainwindow.ui           # Add ui Files
                      controlbar.ui
                      )

QT4_WRAP_CPP (PROJECT_HEADERS_MOC   ${PROJECT_HEADERS})
QT4_WRAP_UI  (PROJECT_FORMS_HEADERS ${PROJECT_FORMS})

INCLUDE         (${QT_USE_FILE})
ADD_DEFINITIONS (${QT_DEFINITIONS})

INCLUDE_DIRECTORIES (${PROJECT_SOURCE_DIR})
INCLUDE_DIRECTORIES (${PROJECT_BINARY_DIR})

ADD_EXECUTABLE  (${PROJECT_NAME} ${PROJECT_SOURCES}
                                 ${PROJECT_FORMS_HEADERS}
                                 ${PROJECT_HEADERS_MOC})

TARGET_LINK_LIBRARIES (${PROJECT_NAME} ${QT_LIBRARIES} ${PCL_LIBRARIES})

