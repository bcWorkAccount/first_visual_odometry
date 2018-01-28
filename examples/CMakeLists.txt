add_executable( fvo_stereo  fvo_stereo.cpp)

target_link_libraries( fvo_stereo
        ${fvo_all_libs}
        ${third_party_libs}
        )