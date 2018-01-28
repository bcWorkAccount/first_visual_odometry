add_library( fvo_common_libs
        src/Config.cpp
        src/FileState.cpp
        src/Frame.cpp
        src/ImageUtils.cpp
        src/GlobalConfig.cpp
        src/MapManagement.cpp
        src/MapPoint.cpp
        src/System.cpp
        src/Viewer.cpp
)

target_link_libraries( fvo_common_libs ${THIRD_PARTY_LIBS} )