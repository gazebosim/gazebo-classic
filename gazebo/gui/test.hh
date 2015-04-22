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

#ifndef GAZEBO_TEST_HH
#define GAZEBO_TEST_HH

class RenderEngine
{
  public: RenderEngine();
  public: void Load();

  public: void Init();
  

  public: void Render();
  
  
  public: void CreateScene();//Ogre::RenderWindow *window);

  public: void Resize(int _width, int _height);
  

  public: Ogre::RenderWindow *CreateOgreWindow(
 // unsigned long _ogreHandle,
 const std::string & _ogreHandle,
                                  uint32_t _width,
                                  uint32_t _height);

  private: void RenderEngine::LoadPlugins();

  private: void RenderEngine::SetupRenderSystem();

  private: void RenderEngine::SetupResources();

  private: Ogre::Root *root;
  private: Ogre::LogManager *logManager;
  private: static uint32_t windowCounter;
  private: Ogre::SceneManager *manager;

  private: Ogre::RenderWindow *window;
  private: Ogre::Camera *camera;
  private: Ogre::Viewport *viewport;

  private: bool sceneCreated;
};


class RenderFrame : public QFrame
{
  Q_OBJECT
  public: RenderFrame(QWidget *parent=NULL);
  protected: virtual void mousePressEvent(QMouseEvent *_event);
};


class RenderWidget : public QWidget
{
  Q_OBJECT
  public: RenderWidget(QWidget *parent);
  public: void Load();

  public: void paintEvent(QPaintEvent *_e);
  
  protected: virtual void mouseMoveEvent(QMouseEvent *_e);

  protected: virtual void wheelEvent(QWheelEvent *_event);

  protected: virtual void mousePressEvent(QMouseEvent *_event);
  protected: virtual void leaveEvent(QEvent *_event);

  protected: void resizeEvent(QResizeEvent *_e);
  protected: void showEvent(QShowEvent *_e);

  protected: bool eventFilter(QObject *_obj, QEvent *_event);

  private: std::string GetOgreHandle() const;

  private: RenderEngine *renderEngine;

  private: RenderFrame *renderFrame;

  private Q_SLOTS: void SendMouseMoveEvent();
  private: QTimer *fake;
};

class MainWindow : public QMainWindow
{
  Q_OBJECT
  public: MainWindow();

  private: void CreateQT();

  public: void Load();

  protected: virtual void mousePressEvent(QMouseEvent *_event);
  protected: bool eventFilter(QObject *_obj, QEvent *_event);
 
  private: RenderWidget *renderWidget;
 
  private: QFrame *mouseFrame;
};

class MyApplication : public QApplication
{
  public: MyApplication(int &argc, char **argv);

  public: virtual bool notify(QObject *_receiver, QEvent *_event);
};

#endif
