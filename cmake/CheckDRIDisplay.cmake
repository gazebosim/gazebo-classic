# FindDRI support
# Check for existance of glxinfo application
# Check for existance of support for pyopengl
MESSAGE(STATUS "Looking for display capabilities")

IF ((DEFINED FORCE_GRAPHIC_TESTS_COMPILATION) AND (${FORCE_GRAPHIC_TESTS_COMPILATION}))
  SET (VALID_DISPLAY TRUE)
  SET (VALID_DRI_DISPLAY TRUE)
  MESSAGE(STATUS " + Force requested. All capabilities on without checking")
  RETURN()
ENDIF()

SET (VALID_DISPLAY FALSE)
SET (VALID_DRI_DISPLAY FALSE)
SET (CHECKER_ERROR "(no glxinfo or pyopengl)")

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
      COMMAND grep "direct rendering:[[:space:]]*Yes[[:space:]]*"
      ERROR_QUIET
      OUTPUT_VARIABLE GLX)

    IF (GLX)
      MESSAGE(STATUS " + found a valid dri display (glxinfo)")
      SET (VALID_DRI_DISPLAY TRUE)
    ELSE()
      SET (CHECKER_ERROR "using glxinfo")
    ENDIF ()
  ELSE ()
    EXECUTE_PROCESS(
      # RESULT_VARIABLE is store in a FAIL variable since the command
      # returns 0 if ok and 1 if error (inverse than cmake IF)
      COMMAND ${PROJECT_SOURCE_DIR}/tools/gl-test.py
      RESULT_VARIABLE GL_FAIL_RESULT
      ERROR_VARIABLE GL_ERROR
      OUTPUT_QUIET)

    IF (NOT GL_FAIL_RESULT)
      MESSAGE(STATUS " + found a valid dri display (pyopengl)")
      SET (VALID_DRI_DISPLAY TRUE)
   ELSE()
      # Check error string: no python module means no pyopengl
      STRING(FIND ${GL_ERROR} 
              "ImportError: No module named OpenGL.GLUT" ERROR_POS)
      # -1 will imply pyopengl is present but real DRI test fails
      IF ("${ERROR_POS}" STREQUAL "-1")
        SET (CHECKER_ERROR "using pyopengl")
      ENDIF ()
    ENDIF ()
  ENDIF ()
ENDIF ()

IF (NOT VALID_DISPLAY)
  MESSAGE(STATUS " ! valid display not found")
ENDIF ()

IF (NOT VALID_DRI_DISPLAY)
    MESSAGE(STATUS " ! valid dri display not found ${CHECKER_ERROR}")
ENDIF ()
