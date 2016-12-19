add_subdirectory(images)
add_subdirectory(models)

SET (files 
  textures.txt
  doc.kml
)

INSTALL(FILES ${files} DESTINATION ${MEDIA_ROOT_DESTINATION}/models/chair3/)
