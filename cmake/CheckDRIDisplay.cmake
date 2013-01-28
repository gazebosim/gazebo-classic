# FindDRI support
# Check for existance of glxinfo application
# Check for existance of support for pyopengl
cmake_minimum_required(VERSION 2.8)
PROJECT(Checker)

MESSAGE(STATUS "Looking for a valid DRI display")
SET (VALID_DRI_DISPLAY FALSE)

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

IF (NOT VALID_DRI_DISPLAY)
  MESSAGE(STATUS " ! valid dri display not found")
ENDIF ()
