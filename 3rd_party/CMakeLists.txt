include_directories(
  ${LIBAVCODEC_PATH}
  ${LIBAVFORMAT_PATH}
  ${libtool_include_path}
  ${gazeboserver_include_dirs}
  ${boost_include_dirs}
  ${freeimage_include_dir}
  ${bullet_include_dir}
  ${threadpool_include_dirs_split}
  ${FLTK_INCLUDE_DIR}
)

link_directories(  
 ${boost_library_dirs} 
 ${freeimage_library_dir}
)

add_subdirectory(assimp)
