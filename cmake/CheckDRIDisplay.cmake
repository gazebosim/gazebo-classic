# FindDRI support
# Check for existance of glxinfo application
# Check for existance of support for pyopengl
MESSAGE(STATUS "Looking for display capabilities")
SET (VALID_DISPLAY FALSE)
SET (VALID_DRI_DISPLAY FALSE)

IF((DEFINED ENV{DISPLAY}) AND NOT ("$ENV{DISPLAY}" STREQUAL ""))
  MESSAGE(STATUS " + found a display available ($DISPLAY is set)")
  SET (VALID_DISPLAY TRUE)

  # Continue check for DRI support in the display
  # Try to run glxinfo. If not found, variable will be empty
  FIND_PROGRAM(GLXINFO glxinfo)

  # If not display found, it will throw an error
  # Another grep pattern: "direct rendering:[[:space:]]*Yes[[:space:]]*"
  IF (GLXINFO)
    EXECUTE_PROCESS(
      COMMAND glxinfo
      COMMAND grep GL_EXT_framebuffer_object
      ERROR_QUIET
      OUTPUT_VARIABLE GLX)

    IF (GLX)
      MESSAGE(STATUS " + found a valid dri display (glxinfo)")
      SET (VALID_DRI_DISPLAY TRUE)
    ENDIF ()
  ELSE ()
    EXECUTE_PROCESS(
      # RESULT_VARIABLE is store in a FAIL variable since the command
      # returns 0 if ok and 1 if error (inverse than cmake IF)
      COMMAND ${PROJECT_SOURCE_DIR}/tools/gl-test.py
      RESULT_VARIABLE GL_FAIL_RESULT
      ERROR_QUIET
      OUTPUT_QUIET)

    IF (NOT GL_FAIL_RESULT)
      MESSAGE(STATUS " + found a valid dri display (pyopengl)")
      SET (VALID_DRI_DISPLAY TRUE)
    ENDIF ()
  ENDIF ()
ENDIF ()

IF (NOT VALID_DISPLAY)
  MESSAGE(STATUS " ! valid display not found")
ENDIF ()

IF (NOT VALID_DRI_DISPLAY)
  MESSAGE(STATUS " ! valid dri display not found")
ENDIF ()
