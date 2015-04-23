#include <iostream>
#include <string>
#include <sstream>
#include <windows.h>
#include <stdio.h>
#include <QtGui>
#include <QObject>
#include <QApplication>
#include <OGRE/Ogre.h>
#include "win_dirent.h"

#ifndef _GAZEBO_TEST_HH_
#define _GAZEBO_TEST_HH_

/// \brief OGRE Interface
class RenderEngine
{
  /// \brief Constructor
  public: RenderEngine();

  /// \brief Load OGRE resources.
  public: void Load();

  /// \brief Initialize
  public: void Init();

  /// \brief Draw the scene.
  public: void Render();

  /// \brief Create the scene (cameras, entities, lights)
  public: void CreateScene();

  /// \brief Resize the render window, and adjust the camera
  public: void Resize(int _width, int _height);

  /// \brief Create a window that OGRE can use for drawing
  public: Ogre::RenderWindow *CreateOgreWindow(
              const std::string & _ogreHandle,
              uint32_t _width,
              uint32_t _height);

  /// \brief Load all the plugins
  private: void LoadPlugins();

  /// \brief Setup OpenGL
  private: void SetupRenderSystem();

  /// \brief Setup resource paths
  private: void SetupResources();

  /// \brief OGRE root object
  private: Ogre::Root *root;

  /// \brief Manages OGRE log file
  private: Ogre::LogManager *logManager;

  /// \brief Used to create unique window names
  private: static uint32_t windowCounter;

  /// \brief Manages the scene
  private: Ogre::SceneManager *manager;

  /// \brief Draw window.
  private: Ogre::RenderWindow *window;

  /// \brief Camera
  private: Ogre::Camera *camera;

  /// \brief Viewport that maps cames to window
  private: Ogre::Viewport *viewport;

  /// \brief True is a scene has been created.
  private: bool sceneCreated;
};

/// \brief Widget that displays the scene, and received mouse events
class RenderWidget : public QWidget
{
  Q_OBJECT

  /// \brief Constructor
  public: RenderWidget(QWidget *parent);

  /// \brief Load
  public: void Load();

  /// \brief QT paint event. Use this to draw the scene
  public: void paintEvent(QPaintEvent *_e);

  /// \brief Receives QT mouse move events.
  protected: virtual void mouseMoveEvent(QMouseEvent *_e);

  /// \brief Receives QT mouse wheel events.
  protected: virtual void wheelEvent(QWheelEvent *_event);

  /// \brief Receives QT mouse button press events.
  protected: virtual void mousePressEvent(QMouseEvent *_event);

  /// \brief Received focus leave event.
  protected: virtual void leaveEvent(QEvent *_event);

  /// \brief Receives resize events
  protected: void resizeEvent(QResizeEvent *_e);

  /// \brief Get the OGRE window handle
  private: std::string GetOgreHandle() const;

  /// \brief Pointer to the render engine
  private: RenderEngine *renderEngine;

  /// \brief Render frame
  private: QFrame *renderFrame;
};

/// \brief Main window
class MainWindow : public QMainWindow
{
  Q_OBJECT

  /// \brief Constructor
  public: MainWindow();

  /// \brief Load
  public: void Load();

  /// \brief Pointer to the render widget
  private: RenderWidget *renderWidget;
};

/// \brief Subclass QApplication so that we can intercept all events.
class MyApplication : public QApplication
{
  /// \brief Constructor
  public: MyApplication(int &argc, char **argv);

  /// \brief Receives all events.
  public: virtual bool notify(QObject *_receiver, QEvent *_event);
};

#endif
