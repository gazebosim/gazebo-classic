# FindDRI support
# Check for existance of glxinfo application
# Check for existance of support for pyopengl
cmake_minimum_required(VERSION 2.8)
PROJECT(Checker)

MESSAGE(STATUS "Looking for a valid DRI display")

# Try to run glxinfo. If not found, variable will be empty
EXECUTE_PROCESS(
    COMMAND glxinfo
    OUTPUT_VARIABLE GLXINFO_EXISTS)

IF (GLXINFO_EXISTS)
  EXECUTE_PROCESS(
    COMMAND glxinfo
    # Another grep pattern: "direct rendering:[[:space:]]*Yes[[:space:]]*"
    COMMAND grep GL_EXT_framebuffer_object
    OUTPUT_VARIABLE GLX)
ELSE ()
  EXECUTE_PROCESS(
      COMMAND {PROJECT_SOURCE_DIR}/tools/gl-test.py
      OUTPUT_QUIET
      ERROR_VARIABLE GL_TEST_ERROR)
ENDIF()

IF (GLX OR NOT GL_TEST_ERROR)
  MESSAGE(STATUS "  found a valid display (dri)")
  SET (VALID_DRI_DISPLAY TRUE)
ELSE()
  MESSAGE(STATUS "  valid dri display not found")
  SET (VALID_DRI_DISPLAY FALSE)
ENDIF ()
