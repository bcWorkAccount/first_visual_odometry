add_library(  fvo_vo_libs
        src/ORBExtractor.cpp
        src/ORBMatcher.cpp
        src/Tracker.cpp
        src/LKFlow.cpp
        src/TrackerLK.cpp
)
target_link_libraries( fvo_vo_libs  fvo_common_libs  ${third_party_libs} )