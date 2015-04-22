#include "test.hh"

bool createOneByOne = true;
bool externalHandle = true;
bool renderFrameId = true;

uint32_t RenderEngine::windowCounter = 1;

/////////////////////////////////////////////////
RenderEngine::RenderEngine()
: root(0), window(0), camera(0), sceneCreated(false)
{
  std::cout << "RenderEngine::RenderEngine\n";

  // Create a new log manager and prevent output from going to stdout
  this->logManager = new Ogre::LogManager();
  std::string logPath = "C:/Users/nkoenig/";
  logPath += "/ogre.log";
  this->logManager->createLog(logPath, true, false, false);

  // Make the root
  this->root = new Ogre::Root();
}

/////////////////////////////////////////////////
void RenderEngine::Load()
{
  std::cout << "RenderEngine::Load\n";

  // Load all the plugins
  this->LoadPlugins();

  // Setup the rendering system, and create the context
  this->SetupRenderSystem();

  this->SetupResources();

  // Initialize the root node, and don't create a window
  this->root->initialise(false);

  if (createOneByOne)
  {
    this->CreateOgreWindow("0",1,1);
  }
}

/////////////////////////////////////////////////
void RenderEngine::Init()
{
  std::cout << "RenderEngine::Init\n";
  Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
}

/////////////////////////////////////////////////
void RenderEngine::Render()
{
if (this->sceneCreated)
{
  this->root->_fireFrameStarted();
  this->window->update();
  this->root->_fireFrameRenderingQueued();
  this->root->_fireFrameEnded();
  }
}
  
/////////////////////////////////////////////////
void RenderEngine::CreateScene()//Ogre::RenderWindow *window)
{
  //this->window = window;

  std::cout << "RenderEngine::CreateScene\n";
  this->manager = this->root->createSceneManager(Ogre::ST_GENERIC);
  this->manager->setAmbientLight(Ogre::ColourValue(0.0, 0.5, 0.0));
  Ogre::Entity *entity = this->manager->createEntity("ogrehead.mesh");
  Ogre::SceneNode *node =
  this->manager->getRootSceneNode()->createChildSceneNode();
  node->attachObject(entity);

  Ogre::Light *light = this->manager->createLight("MainLight");
  light->setType(Ogre::Light::LT_DIRECTIONAL);
  light->setDirection(0, -1, 1);
  light->setPosition(20, 80, 50);

  this->camera = this->manager->createCamera("MainCamera");
  this->camera->setPosition(0, 47, 500);
  this->camera->lookAt(0, 0, 0);
  this->camera->setNearClipDistance(.1);
  this->camera->setFarClipDistance(1000);

  this->viewport = this->window->addViewport(camera);
  this->viewport->setBackgroundColour(Ogre::ColourValue(0.5, 0.0, 0.0));

  std::cout << "Viewport WxH[" << this->viewport->getActualWidth() << " "
            << this->viewport->getActualHeight() << "]\n";

  std::cout << "Window WxH[" << this->window->getWidth() << " "
            << this->window->getHeight() << "]\n";

  /*this->camera->setAspectRatio(
    Ogre::Real(this->viewport->getActualWidth()) /
    Ogre::Real(this->viewport->getActualHeight()));
    */

  this->sceneCreated = true;
}

/////////////////////////////////////////////////
void RenderEngine::Resize(int _width, int _height)
{
  std::cout << "RenderEngine::Resize(" << _width << " " << _height << ")\n";

  if (this->window)
  {
    // this->window->reposition(-100,-100);
    this->window->resize(_width, _height);
    this->window->windowMovedOrResized();
  }

  /*if (this->camera)
  {
  //  this->viewport->setDimensions(0, 0, 1, 1);
    double ratio = static_cast<double>(this->viewport->getActualWidth()) /
      static_cast<double>(this->viewport->getActualHeight());

    double hfov =  1.047;
    double vfov = 2.0 * atan(tan(hfov / 2.0) / ratio);

    this->camera->setAspectRatio(ratio);
    this->camera->setFOVy(Ogre::Radian(vfov));
  }
  */
}

/////////////////////////////////////////////////
Ogre::RenderWindow *RenderEngine::CreateOgreWindow(
  //unsigned long _ogreHandle, uint32_t _width, uint32_t _height)
  const std::string &_ogreHandle, uint32_t _width, uint32_t _height)
{
  Ogre::StringVector paramsVector;
  Ogre::NameValuePairList params;
  Ogre::RenderWindow *window = NULL;

  std::string handle;
  if (externalHandle)
    handle = "externalWindowHandle";
  else
    handle = "parentWindowHandle";

  if (_width == 1)
    handle = "parentWindowHandle";

  std::cout << "CreateOgreWindow. Handle[" << _ogreHandle
            << "] WxH[" << _width << " " << _height << "] Type["
            << handle << "]\n";

  params[handle] = _ogreHandle;
  params["externalGLControl"] = "true";

  std::ostringstream stream;
  stream << "OgreWindow(" << windowCounter++ << ")";

  int attempts = 0;
  while (window == NULL && (attempts++) < 10)
  {
    try
    {
      window = this->root->createRenderWindow(
          stream.str(), _width, _height, false, &params);
    }
    catch(...)
    {
      std::cerr << " Unable to create the rendering window\n";
      window = NULL;
    }
  }

  if (attempts >= 10)
  {
    std::cerr << "Unable to create the rendering window\n";
    return 0;
  }

  if (window)
  {
    //window->setActive(false);
    //window->resize(_width, _height);
    window->reposition(0, 0);
    window->setAutoUpdated(true);
    window->setVisible(true);
  }

  if (externalHandle)
    this->window = window;
  return window;
}

//////////////////////////////////////////////////
void RenderEngine::LoadPlugins()
{
  std::list<std::string>::iterator iter;
  std::list<std::string> ogrePaths;
  ogrePaths.push_back("C");
  ogrePaths.push_back("C:/Users/nkoenig/code/gz/ogre_src_v1-8-1-vc12-x64-release-debug/build/install/Debug/bin/Debug/");

  for (iter = ogrePaths.begin(); iter != ogrePaths.end(); ++iter)
  {
    std::string path(*iter);
    DIR *dir = opendir(path.c_str());

    if (dir == NULL)
    {
      continue;
    }
    closedir(dir);

    std::vector<std::string> plugins;
    std::vector<std::string>::iterator piter;

    std::string prefix = "";
    std::string extension = ".dll";

    plugins.push_back(path+"/"+prefix+"RenderSystem_GL_d");

    for (piter = plugins.begin(); piter != plugins.end(); ++piter)
    {
      try
      {
        // Load the plugin into OGRE
        this->root->loadPlugin(*piter+extension);
      }
      catch(Ogre::Exception &e)
      {
        try
        {
          // Load the debug plugin into OGRE
          this->root->loadPlugin(*piter+"_d"+extension);
        }
        catch(Ogre::Exception &ed)
        {
          if ((*piter).find("RenderSystem") != std::string::npos)
          {
            std::cerr << "Unable to load Ogre Plugin[" << *piter
                  << "]. Rendering will not be possible."
      	    << "Make sure you have installed OGRE and Gazebo properly.\n";
          }
        }
      }
    }
  }
}

//////////////////////////////////////////////////
void RenderEngine::SetupRenderSystem()
{
  Ogre::RenderSystem *renderSys;
  const Ogre::RenderSystemList *rsList;
  rsList = &(this->root->getAvailableRenderers());

  int c = 0;

  renderSys = NULL;
  do
  {
    if (c == static_cast<int>(rsList->size()))
      break;

    renderSys = rsList->at(c);
    c++;
  }
  while (renderSys &&
         renderSys->getName().compare("OpenGL Rendering Subsystem") != 0);

  if (renderSys == NULL)
  {
    std::cerr << "unable to find OpenGL rendering system. OGRE is probably "
            "installed incorrectly. Double check the OGRE cmake output, "
            "and make sure OpenGL is enabled.\n";
    return;
  }

  // We operate in windowed mode
  renderSys->setConfigOption("Full Screen", "No");
  renderSys->setConfigOption("RTT Preferred Mode", "FBO");
  this->root->setRenderSystem(renderSys);
}

//////////////////////////////////////////////////
void RenderEngine::SetupResources()
{
  std::vector< std::pair<std::string, std::string> > archNames;
  std::vector< std::pair<std::string, std::string> >::iterator aiter;
  std::list<std::string>::const_iterator iter;
  std::list<std::string> paths;
  paths.push_back("C");
  paths.push_back("C:/Users/nkoenig/code/gz/gazebo/");

  std::list<std::string> mediaDirs;
  mediaDirs.push_back("media");
  mediaDirs.push_back("Media");

  for (iter = paths.begin(); iter != paths.end(); ++iter)
  {
    DIR *dir;
    if ((dir = opendir((*iter).c_str())) == NULL)
    {
      continue;
    }
    closedir(dir);

    archNames.push_back(
        std::make_pair((*iter)+"/", "General"));

    for (std::list<std::string>::iterator mediaIter = mediaDirs.begin();
         mediaIter != mediaDirs.end(); ++mediaIter)
    {
      std::string prefix = (*iter) + "/" + (*mediaIter);

      archNames.push_back(
          std::make_pair(prefix, "General"));
      archNames.push_back(
          std::make_pair(prefix + "/materials/programs", "General"));
      archNames.push_back(
          std::make_pair(prefix + "/materials/scripts", "General"));
      archNames.push_back(
          std::make_pair(prefix + "/materials/textures", "General"));
      archNames.push_back(
          std::make_pair(prefix + "/models", "General"));
    }

    for (aiter = archNames.begin(); aiter != archNames.end(); ++aiter)
    {
      try
      {
        Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
            aiter->first, "FileSystem", aiter->second);
      }
      catch(Ogre::Exception &/*_e*/)
      {
        std::cerr << "Unable to load Ogre Resources. Make sure the resources path in the world file is set correctly.\n";
        return;
      }
    }
  }
}

RenderWidget::RenderWidget(QWidget *parent)
: QWidget(parent)
{
  this->setObjectName("RenderWidget");
  this->renderEngine = new RenderEngine();

  this->setAttribute(Qt::WA_NoSystemBackground, true);
  this->setAttribute(Qt::WA_OpaquePaintEvent, true);
  this->setAttribute(Qt::WA_PaintOnScreen, true);

  this->renderFrame = new RenderFrame;
  this->renderFrame->setObjectName("RenderFrame");
  this->renderFrame->setLineWidth(1);
  this->renderFrame->setFrameShadow(QFrame::Sunken);
  this->renderFrame->setFrameShape(QFrame::Box);
  this->renderFrame->show();

  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->setContentsMargins(0, 0, 0, 0);
  mainLayout->addWidget(this->renderFrame);
  //mainLayout->addWidget(this->mouseFrame);
  this->setLayout(mainLayout);

  /*this->setMouseTracking(true);
  this->setFocus(Qt::OtherFocusReason);

  this->renderFrame->setMouseTracking(true);
  this->renderFrame->setFocus(Qt::OtherFocusReason);
  */

  this->renderEngine->Load();

  QApplication::flush();
  QApplication::syncX();
  std::string handle = this->GetOgreHandle();

  Ogre::RenderWindow *win = 
    this->renderEngine->CreateOgreWindow(handle,
      this->width(), this->height());

  // this->renderEngine->CreateScene(win);

  //this->create(this->winId(), true, false);
  //this->raise();
  // this->renderFrame->raise();

  // this->installEventFilter(this);
  // this->renderFrame->setAttribute(Qt::WA_TransparentForMouseEvents);
  // this->renderFrame->setAttribute(Qt::WA_NoMousePropagation);
  // this->renderFrame->setAttribute(Qt::WA_TransparentForMouseEvents);
  // this->setAttribute(Qt::WA_TransparentForMouseEvents);

}

///////////////////////////////////////////////
void RenderWidget::Load()
{
  this->show();
  this->renderEngine->Init();

  this->fake = new QTimer();
  connect(this->fake, SIGNAL(timeout()), this, SLOT(SendMouseMoveEvent()));
  // this->fake->start(33);
}

///////////////////////////////////////////////
void RenderWidget::paintEvent(QPaintEvent *_e)
{
    this->renderEngine->Render();

  this->update();

  _e->accept();
}

///////////////////////////////////////////////
void RenderWidget::SendMouseMoveEvent()
{
  QPoint pos = QCursor::pos();
  QPoint posRel = this->mapFromGlobal(pos);
  if (this->rect().contains(posRel))
  {
    //std::cout << "SendMouseMOveEvent Pos[" << pos.x() << " " << pos.y() << "]\n";
    bool mouse_over_this = false;

    QMouseEvent fakeEvent(QEvent::MouseMove, posRel, Qt::NoButton, QApplication::mouseButtons(), QApplication::keyboardModifiers());
    this->event(&fakeEvent);
  }
}

void RenderWidget::mouseMoveEvent(QMouseEvent *_e)
{
  std::cout << "Mouse Move Event\n";
}

void RenderWidget::wheelEvent(QWheelEvent *_event)
{
  std::cout << "Wheel Event\n";
}

void RenderWidget::mousePressEvent(QMouseEvent *_event)
{
  std::cout << "Mouse Press Event\n";
}

///////////////////////////////////////////////
void RenderWidget::resizeEvent(QResizeEvent *_e)
{
  if (this->renderEngine)
    this->renderEngine->Resize(_e->size().width(), _e->size().height());
}

void RenderWidget::showEvent(QShowEvent *_e)
{
  QWidget::showEvent(_e);
  this->setFocus();

  this->renderEngine->CreateScene();//win);

  std::cout << "Show Event\n";
}

/////////////////////////////////////////////////
bool RenderWidget::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == QEvent::MouseMove)
  {
    std::cout << "RenderWidget::eventFilter mouse move\n";
    this->mouseMoveEvent((QMouseEvent *)_event);
  }


/*  if (_obj == this->renderFrame)
  {
    if (_event->type() == QEvent::MouseMove)
    {
      std::cerr << "\t render frame\n";
      this->mouseMoveEvent((QMouseEvent *)_event);
    }
  }

  if (_event->type() == QEvent::Enter)
  {
    std::cout << "\t QEvent::Enter\n";
    this->setFocus(Qt::OtherFocusReason);
    return true;
  }
  */

  // std::cout << "\tOther\n";
  return false;
}

///////////////////////////////////////////////
void RenderWidget::leaveEvent(QEvent *_event)
{
  std::cout << "Leave Event\n";
}

///////////////////////////////////////////////
std::string RenderWidget::GetOgreHandle() const
{
  std::ostringstream stream;

  if (renderFrameId)
    stream << (unsigned long)this->renderFrame->winId();
  else
    stream << (unsigned long)this->winId();

  return stream.str();
}

///////////////////////////////////////////////
RenderFrame::RenderFrame(QWidget *parent)
: QFrame(parent)
{
}

///////////////////////////////////////////////
void RenderFrame::mousePressEvent(QMouseEvent *_event)
{
  std::cout << "MOUSE PRESS EVENT!!!!!\n";
}

///////////////////////////////////////////////
MainWindow::MainWindow()
 : QMainWindow()
{
  this->setWindowTitle("Main WIndow");
  this->show();
  std::cout << "MainWindow Constructor\n";
  this->setGeometry(20, 20, 1024, 768);

  this->setMouseTracking(true);
  this->setFocus(Qt::OtherFocusReason);

  this->installEventFilter(this);
}

/////////////////////////////////////////////////
bool MainWindow::eventFilter(QObject *_obj, QEvent *_event)
{

  if (_event->type() == QEvent::MouseMove)
  {
    std::cout << "MainWindow::eventFilter render frame\n";
    this->mouseMoveEvent((QMouseEvent *)_event);
    return true;
  }

  if (_event->type() == QEvent::Enter)
  {
    std::cout << "MainWindow::eventFilter enter\n";
    this->setFocus(Qt::OtherFocusReason);
    return true;
  }

  return false;
}

///////////////////////////////////////////////
void MainWindow::Load()
{
  QWidget *central = new QWidget(this);
  QHBoxLayout *centralLayout = new QHBoxLayout;
  centralLayout->setSpacing(0);
  centralLayout->setMargin(0);

  this->renderWidget = new RenderWidget(central);

  centralLayout->addWidget(this->renderWidget);
  central->setLayout(centralLayout);

  this->setCentralWidget(central);

  this->renderWidget->Load();


  //this->mouseFrame = new QFrame(this);
  //this->mouseFrame->setLineWidth(1);
  //this->mouseFrame->setFrameShadow(QFrame::Sunken);
  //this->mouseFrame->setFrameShape(QFrame::Box);
  //this->mouseFrame->show();
  //this->mouseFrame->raise();
  //this->mouseFrame->resize(1024,768);
  //this->mouseFrame->setStyleSheet("QFrame {background-color: transparent;}");
  //this->mouseFrame->setVisible(true);

  //this->mouseFrame->setAutoFillBackground(true);
  //QPalette p = this->mouseFrame->palette();
  //p.setColor(QPalette::Window, Qt::transparent);
  //this->mouseFrame->setPalette(p);

}

void MainWindow::mousePressEvent(QMouseEvent *_event)
{
  std::cout << "MainWindow::Mouse Press Event\n";
}


MyApplication::MyApplication(int &argc,char **argv)
: QApplication(argc, argv)
{
  
}

bool MyApplication::notify(QObject *_receiver, QEvent *_event)
{
if (_event->type() != 12 && _event->type() != 78 && _event->type() != 77)
{
  std::cout << "Notify[" << _receiver->objectName().toStdString() << "] Event[" << _event->type() << "]\n";
  }
  QApplication::notify(_receiver, _event);
  return true;
}

int main(int argc, char **argv)
{
  //QApplication *app = new QApplication(argc, argv);
  MyApplication *app = new MyApplication(argc, argv);

  MainWindow *main = new MainWindow();
  // main->show();

  // These two lines cause the window handle to be stable.
  // QApplication::flush();
  // QApplication::syncX();

  main->Load();

  app->exec();

  delete main;

  return 0;
}
