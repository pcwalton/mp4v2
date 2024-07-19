cmake_minimum_required(VERSION 3.6)
#project(mp4v2_2_0_0)
project(mp4v2_1_0_0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Do not allow in-source build.
if( ${PROJECT_SOURCE_DIR} STREQUAL ${PROJECT_BINARY_DIR} )
    message( FATAL_ERROR "In-source build is not allowed. Please build in a separate directory, such as ${PROJECT_SOURCE_DIR}/build." )
endif()

include_directories("${CMAKE_SOURCE_DIR}" "${CMAKE_SOURCE_DIR}/include")

set(PROJECT_repo_rev 0)

configure_file(${CMAKE_CURRENT_LIST_DIR}/include/mp4v2/project.h.in ${CMAKE_CURRENT_LIST_DIR}/include/mp4v2/project.h)
configure_file(${CMAKE_CURRENT_LIST_DIR}/libplatform/config.h.in ${CMAKE_CURRENT_LIST_DIR}/libplatform/config.h)

set(HEADER_FILES
       ${CMAKE_CURRENT_LIST_DIR}/include/mp4v2/chapter.h
       ${CMAKE_CURRENT_LIST_DIR}/include/mp4v2/file.h
       ${CMAKE_CURRENT_LIST_DIR}/include/mp4v2/file_prop.h
       ${CMAKE_CURRENT_LIST_DIR}/include/mp4v2/general.h
       ${CMAKE_CURRENT_LIST_DIR}/include/mp4v2/isma.h
       ${CMAKE_CURRENT_LIST_DIR}/include/mp4v2/itmf_generic.h
       ${CMAKE_CURRENT_LIST_DIR}/include/mp4v2/itmf_tags.h
       ${CMAKE_CURRENT_LIST_DIR}/include/mp4v2/mp4v2.h
       ${CMAKE_CURRENT_LIST_DIR}/include/mp4v2/platform.h
       ${CMAKE_CURRENT_LIST_DIR}/include/mp4v2/project.h
       ${CMAKE_CURRENT_LIST_DIR}/include/mp4v2/sample.h
       ${CMAKE_CURRENT_LIST_DIR}/include/mp4v2/streaming.h
       ${CMAKE_CURRENT_LIST_DIR}/include/mp4v2/track.h
       ${CMAKE_CURRENT_LIST_DIR}/include/mp4v2/track_prop.h
       ${CMAKE_CURRENT_LIST_DIR}/libplatform/io/File.cpp
       ${CMAKE_CURRENT_LIST_DIR}/libplatform/io/File.h
       ${CMAKE_CURRENT_LIST_DIR}/libplatform/io/FileSystem.h
       ${CMAKE_CURRENT_LIST_DIR}/libplatform/number/random.h
       ${CMAKE_CURRENT_LIST_DIR}/libplatform/process/process.h
       ${CMAKE_CURRENT_LIST_DIR}/libplatform/prog/option.h
       ${CMAKE_CURRENT_LIST_DIR}/libplatform/sys/error.h
       ${CMAKE_CURRENT_LIST_DIR}/libplatform/time/time.h
       ${CMAKE_CURRENT_LIST_DIR}/libplatform/config.h
       ${CMAKE_CURRENT_LIST_DIR}/libplatform/endian.h
       ${CMAKE_CURRENT_LIST_DIR}/libplatform/impl.h
       ${CMAKE_CURRENT_LIST_DIR}/libplatform/platform.h
       ${CMAKE_CURRENT_LIST_DIR}/libplatform/platform_base.h
       ${CMAKE_CURRENT_LIST_DIR}/libplatform/platform_posix.h
       ${CMAKE_CURRENT_LIST_DIR}/libplatform/warning.h
       ${CMAKE_CURRENT_LIST_DIR}/libutil/crc.h
       ${CMAKE_CURRENT_LIST_DIR}/libutil/Database.h
       ${CMAKE_CURRENT_LIST_DIR}/libutil/impl.h
       ${CMAKE_CURRENT_LIST_DIR}/libutil/other.h
       ${CMAKE_CURRENT_LIST_DIR}/libutil/Timecode.h
       ${CMAKE_CURRENT_LIST_DIR}/libutil/TrackModifier.h
       ${CMAKE_CURRENT_LIST_DIR}/libutil/util.h
       ${CMAKE_CURRENT_LIST_DIR}/libutil/Utility.h
       ${CMAKE_CURRENT_LIST_DIR}/src/bmff/bmff.h
       ${CMAKE_CURRENT_LIST_DIR}/src/bmff/impl.h
       ${CMAKE_CURRENT_LIST_DIR}/src/bmff/typebmff.h
       ${CMAKE_CURRENT_LIST_DIR}/src/itmf/CoverArtBox.h
       ${CMAKE_CURRENT_LIST_DIR}/src/itmf/generic.h
       ${CMAKE_CURRENT_LIST_DIR}/src/itmf/impl.h
       ${CMAKE_CURRENT_LIST_DIR}/src/itmf/itmf.h
       ${CMAKE_CURRENT_LIST_DIR}/src/itmf/Tags.h
       ${CMAKE_CURRENT_LIST_DIR}/src/itmf/type.h
       ${CMAKE_CURRENT_LIST_DIR}/src/qtff/coding.h
       ${CMAKE_CURRENT_LIST_DIR}/src/qtff/ColorParameterBox.h
       ${CMAKE_CURRENT_LIST_DIR}/src/qtff/impl.h
       ${CMAKE_CURRENT_LIST_DIR}/src/qtff/PictureAspectRatioBox.h
       ${CMAKE_CURRENT_LIST_DIR}/src/qtff/qtff.h
       ${CMAKE_CURRENT_LIST_DIR}/src/atoms.h
       ${CMAKE_CURRENT_LIST_DIR}/src/descriptors.h
       ${CMAKE_CURRENT_LIST_DIR}/src/enum.h
       ${CMAKE_CURRENT_LIST_DIR}/src/exception.h
       ${CMAKE_CURRENT_LIST_DIR}/src/impl.h
       ${CMAKE_CURRENT_LIST_DIR}/src/log.h
       ${CMAKE_CURRENT_LIST_DIR}/src/mp4array.h
       ${CMAKE_CURRENT_LIST_DIR}/src/mp4atom.h
       ${CMAKE_CURRENT_LIST_DIR}/src/mp4container.h
       ${CMAKE_CURRENT_LIST_DIR}/src/mp4descriptor.h
       ${CMAKE_CURRENT_LIST_DIR}/src/mp4file.h
       ${CMAKE_CURRENT_LIST_DIR}/src/mp4property.h
       ${CMAKE_CURRENT_LIST_DIR}/src/mp4track.h
       ${CMAKE_CURRENT_LIST_DIR}/src/mp4util.h
       ${CMAKE_CURRENT_LIST_DIR}/src/ocidescriptors.h
       ${CMAKE_CURRENT_LIST_DIR}/src/odcommands.h
       ${CMAKE_CURRENT_LIST_DIR}/src/qosqualifiers.h
       ${CMAKE_CURRENT_LIST_DIR}/src/rtphint.h
       ${CMAKE_CURRENT_LIST_DIR}/src/src.h
       ${CMAKE_CURRENT_LIST_DIR}/src/text.h
       ${CMAKE_CURRENT_LIST_DIR}/src/util.h)

set(SOURCE_FILES
       ${CMAKE_CURRENT_LIST_DIR}/libplatform/io/File_posix.cpp
       ${CMAKE_CURRENT_LIST_DIR}/libplatform/io/FileSystem.cpp
       ${CMAKE_CURRENT_LIST_DIR}/libplatform/io/FileSystem_posix.cpp
       ${CMAKE_CURRENT_LIST_DIR}/libplatform/number/random_posix.cpp
       ${CMAKE_CURRENT_LIST_DIR}/libplatform/process/process_posix.cpp
       ${CMAKE_CURRENT_LIST_DIR}/libplatform/prog/option.cpp
       ${CMAKE_CURRENT_LIST_DIR}/libplatform/sys/error.cpp
       ${CMAKE_CURRENT_LIST_DIR}/libplatform/time/time.cpp
       ${CMAKE_CURRENT_LIST_DIR}/libplatform/time/time_posix.cpp
       ${CMAKE_CURRENT_LIST_DIR}/libutil/crc.cpp
       ${CMAKE_CURRENT_LIST_DIR}/libutil/Database.cpp
       ${CMAKE_CURRENT_LIST_DIR}/libutil/other.cpp
       ${CMAKE_CURRENT_LIST_DIR}/libutil/Timecode.cpp
       ${CMAKE_CURRENT_LIST_DIR}/libutil/TrackModifier.cpp
       ${CMAKE_CURRENT_LIST_DIR}/libutil/Utility.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/bmff/typebmff.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/itmf/CoverArtBox.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/itmf/generic.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/itmf/Tags.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/itmf/type.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/qtff/coding.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/qtff/ColorParameterBox.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/qtff/PictureAspectRatioBox.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/3gp.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_ac3.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_amr.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_av1C.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_avc1.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_avcC.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_avxx.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_chpl.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_colr.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_d263.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_dac3.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_damr.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_dref.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_dops.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_elst.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_enca.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_encv.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_free.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_ftab.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_ftyp.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_gmin.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_hdlr.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_hinf.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_hnti.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_href.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_mdat.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_mdhd.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_meta.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_mp4s.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_mp4v.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_mvhd.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_nmhd.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_ohdr.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_opus.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_pasp.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_root.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_rtp.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_s263.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_sdp.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_sdtp.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_smi.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_sound.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_standard.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_stbl.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_stdp.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_stsc.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_stsd.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_stsz.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_stz2.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_text.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_tfhd.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_tkhd.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_treftype.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_trun.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_tx3g.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_udta.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_url.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_urn.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_uuid.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_video.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_vpcC.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_vpxx.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/atom_vmhd.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/cmeta.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/descriptors.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/enum.tcc
       ${CMAKE_CURRENT_LIST_DIR}/src/exception.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/isma.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/log.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/mp4.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/mp4atom.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/mp4container.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/mp4descriptor.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/mp4file.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/mp4file_io.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/mp4info.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/mp4property.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/mp4track.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/mp4util.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/ocidescriptors.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/odcommands.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/qosqualifiers.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/rtphint.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/text.cpp
)

add_library(mp4v2-shared SHARED ${HEADER_FILES} ${SOURCE_FILES})
target_include_directories(mp4v2-shared PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/include
)
target_compile_options(mp4v2-shared PUBLIC -Wno-narrowing)

add_library(mp4v2-static STATIC ${HEADER_FILES} ${SOURCE_FILES})
target_include_directories(mp4v2-static PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_compile_options(mp4v2-static PUBLIC -Wno-narrowing)

#for debugging
#target_compile_options(mp4v2-static PUBLIC
#    -Wliteral-suffix
#    -ggdb
#    -O0
#)


set(UTILITY_HEADERS
        ${CMAKE_CURRENT_LIST_DIR}/util/impl.h)

add_executable(mp4art ${UTILITY_HEADERS} ${CMAKE_CURRENT_LIST_DIR}/util/mp4art.cpp)
target_link_libraries(mp4art mp4v2-static)

add_executable(mp4chaps ${UTILITY_HEADERS} ${CMAKE_CURRENT_LIST_DIR}/util/mp4chaps.cpp)
target_link_libraries(mp4chaps mp4v2-static)

add_executable(mp4extract ${UTILITY_HEADERS} ${CMAKE_CURRENT_LIST_DIR}/util/mp4extract.cpp)
target_link_libraries(mp4extract mp4v2-static)

add_executable(mp4file ${UTILITY_HEADERS} ${CMAKE_CURRENT_LIST_DIR}/util/mp4file.cpp)
target_link_libraries(mp4file mp4v2-static)

add_executable(mp4info ${UTILITY_HEADERS} ${CMAKE_CURRENT_LIST_DIR}/util/mp4info.cpp)
target_link_libraries(mp4info mp4v2-static)

add_executable(mp4subtitle ${UTILITY_HEADERS} ${CMAKE_CURRENT_LIST_DIR}/util/mp4subtitle.cpp)
target_link_libraries(mp4subtitle mp4v2-static)

add_executable(mp4tags ${UTILITY_HEADERS} ${CMAKE_CURRENT_LIST_DIR}/util/mp4tags.cpp)
target_link_libraries(mp4tags mp4v2-static)

add_executable(mp4track ${UTILITY_HEADERS} ${CMAKE_CURRENT_LIST_DIR}/util/mp4track.cpp)
target_link_libraries(mp4track mp4v2-static)

add_executable(mp4trackdump ${UTILITY_HEADERS} ${CMAKE_CURRENT_LIST_DIR}/util/mp4trackdump.cpp)
target_link_libraries(mp4trackdump mp4v2-static)

