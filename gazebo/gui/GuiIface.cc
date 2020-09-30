/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifdef _WIN32
  // snprintf is available since VS 2015
  #if defined(_MSC_VER) && (_MSC_VER < 1900)
     #define snprintf _snprintf
  #endif
#endif

#include <signal.h>
#include <boost/program_options.hpp>
#include <boost/property_tree/ini_parser.hpp>

#include <ignition/common/Profiler.hh>
#include <ignition/math/SemanticVersion.hh>

#include "gazebo/gui/qt.h"
#include "gazebo/gazebo_client.hh"

#include "gazebo/common/Time.hh"
#include "gazebo/common/ModelDatabase.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/CommonTypes.hh"
#include "gazebo/gui/SplashScreen.hh"
#include "gazebo/gui/MainWindow.hh"
#include "gazebo/gui/ModelRightMenu.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/GuiPlugin.hh"
#include "gazebo/gui/RenderWidget.hh"

#ifdef WIN32
# define HOMEDIR "HOMEPATH"
#else
# define HOMEDIR "HOME"
#endif  // WIN32

// These are needed by QT. They need to stay valid during the entire
// lifetime of the application, and argc > 0 and argv must contain one valid
// character string
int g_argc = 1;
char **g_argv;

namespace po = boost::program_options;
po::variables_map vm;

boost::property_tree::ptree g_propTree;

// Names of all GUI plugins loaded at start. Parsed from command line arguments.
std::vector<std::string> g_plugins_to_load;

using namespace gazebo;

gui::ModelRightMenu *g_modelRightMenu = NULL;

std::string g_worldname = "default";

QApplication *g_app;
gui::SplashScreen *g_splashScreen = NULL;
gui::MainWindow *g_main_win = NULL;
rendering::UserCameraPtr g_active_camera;
bool g_fullscreen = false;

// This makes it possible to use common::Time in QT signals and slots.
// qRegisterMetaType is also required, see below.
Q_DECLARE_METATYPE(common::Time)

// This makes it possible to use std::string in QT signals and slots.
// qRegisterMetaType is also required, see below.
Q_DECLARE_METATYPE(std::string)

// This makes it possible to use std::set<std::string> in QT signals and slots.
// qRegisterMetaType is also required, see below.
Q_DECLARE_METATYPE(std::set<std::string>)

// This makes it possible to use ignition::msgs::JointCmd in signals and slots.
// qRegisterMetaType is also required, see below.
Q_DECLARE_METATYPE(ignition::msgs::JointCmd)

//////////////////////////////////////////////////
// QT message handler that pipes qt messages into gazebo's console system.
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
void messageHandler(QtMsgType _type, const QMessageLogContext &_context,
    const QString &_msg)
{
  std::string msg = _msg.toStdString();
  if (_context.function)
    msg += std::string("(") + _context.function + ")";
#else
void messageHandler(QtMsgType _type, const char *_msg)
{
  const char *msg = _msg;
#endif

  switch (_type)
  {
    case QtDebugMsg:
      gzdbg << msg << std::endl;
      break;
#if QT_VERSION >= QT_VERSION_CHECK(5, 5, 0)
    case QtInfoMsg:
      gzmsg << msg << std::endl;
      break;
#endif
    case QtWarningMsg:
      gzwarn << msg << std::endl;
      break;
    case QtFatalMsg:
    case QtCriticalMsg:
      gzerr << msg << std::endl;
      break;
    default:
      gzwarn << "Unknown QT Message type[" << _type << "]: "
        << msg << std::endl;
      break;
  }
}

//////////////////////////////////////////////////
void print_usage()
{
  std::cerr << "gzclient -- Gazebo GUI Client\n\n";
  std::cerr << "`gzclient` [options]\n\n";
  std::cerr << "Gazebo GUI client which allows visualization and user "
    << "interaction.\n\n";
}

//////////////////////////////////////////////////
void signal_handler(int)
{
  event::Events::sigInt();
  gazebo::gui::stop();
  gazebo::client::shutdown();
}

//////////////////////////////////////////////////
bool parse_args(int _argc, char **_argv)
{
  po::options_description v_desc("Options");
  v_desc.add_options()
    ("version,v", "Output version information.")
    ("verbose", "Increase the messages written to the terminal.")
    ("help,h", "Produce this help message.")
    ("gui-client-plugin", po::value<std::vector<std::string> >(),
     "Load a GUI plugin.")
    ("gui-plugin,g", po::value<std::vector<std::string> >(),
     "Load a System plugin (deprecated, backwards compatibility reasons).");

  po::options_description desc("Options");
  desc.add(v_desc);

  try
  {
    po::store(po::command_line_parser(_argc,
          _argv).options(desc).allow_unregistered().run(), vm);
    po::notify(vm);
  } catch(boost::exception &_e)
  {
    std::cerr << "Error. Gui Invalid arguments\n";
    return false;
  }

  if (vm.count("version"))
  {
    std::cout << GAZEBO_VERSION_HEADER << std::endl;
    return false;
  }

  if (vm.count("help"))
  {
    print_usage();
    std::cerr << v_desc << "\n";
    return false;
  }

  if (vm.count("verbose"))
  {
    gazebo::client::printVersion();
    gazebo::common::Console::SetQuiet(false);
  }

  /// Load the System plugins specified on the command line
  /// see https://github.com/osrf/gazebo/issues/2279 for details
  if (vm.count("gui-plugin"))
  {
    gzwarn << "g/gui-plugin is really loading a SystemPlugin. "
           << "To load a GUI plugin please use --gui-client-plugin \n";

    std::vector<std::string> pp =
      vm["gui-plugin"].as<std::vector<std::string> >();

    for (std::vector<std::string>::iterator iter = pp.begin();
         iter != pp.end(); ++iter)
    {
      gazebo::client::addPlugin(*iter);
    }
  }

  /// Load the GUI plugins specified on the command line
  if (vm.count("gui-client-plugin"))
  {
    std::vector<std::string> pp =
      vm["gui-client-plugin"].as<std::vector<std::string> >();

    for (std::vector<std::string>::iterator iter = pp.begin();
         iter != pp.end(); ++iter)
    {
       g_plugins_to_load.push_back(*iter);
    }
  }

  return true;
}

namespace gazebo
{
  namespace gui
  {
    void set_style()
    {
      QFile file(":/style.qss");
      file.open(QFile::ReadOnly);
      QString styleSheet = QLatin1String(file.readAll());
      g_app->setStyleSheet(styleSheet);
    }

    /////////////////////////////////////////////////
    void fini()
    {
      // Cleanup model database.
      common::ModelDatabase::Instance()->Fini();

      gui::clear_active_camera();
      rendering::fini();
      fflush(stdout);
    }
  }
}

/////////////////////////////////////////////////
void gui::init()
{
  g_modelRightMenu->Init();
  g_main_win->Init();
}

/////////////////////////////////////////////////
bool gui::loadINI(boost::filesystem::path _file)
{
  bool result = true;

  // Only use the environment variables if _file is empty.
  if (_file.empty())
  {
    // Get the gui.ini path environment variable
    char *guiINIFile = getenv("GAZEBO_GUI_INI_FILE");
    char *home = getenv(HOMEDIR);

    // If the environment variable was specified
    if (guiINIFile)
    {
      _file = guiINIFile;
      if (!boost::filesystem::exists(_file))
      {
        gzerr << "GAZEBO_GUI_INI_FILE does not exist: " << _file << std::endl;
        return false;
      }
    }
    else if (home)
    {
      // Check the home directory
      // Construct the path to gui.ini
      _file = home;
      _file = _file / ".gazebo" / "gui.ini";
    }
  }

  // Create the gui.ini file if it doesn't exist.
  if (!boost::filesystem::exists(_file))
  {
    gui::setINIProperty("geometry.x", 0);
    gui::setINIProperty("geometry.y", 0);
    gui::saveINI(_file);
    gzwarn << "Couldn't locate specified .ini. Creating file at " << _file
          << std::endl;
  }

  try
  {
    // Read all configuration properties
    boost::property_tree::ini_parser::read_ini(_file.string(), g_propTree);
  }
  catch(...)
  {
    gzerr << "Unable to read configuration file " << _file << "\n";
    result = false;
  }

  gzlog << "Loaded .ini file from: " << _file << std::endl;
  return result;
}

/////////////////////////////////////////////////
bool gui::load()
{
  gui::loadINI();

  g_modelRightMenu = new gui::ModelRightMenu();

  // Load the rendering engine.
  rendering::load();
  rendering::init();

  g_argv = new char*[g_argc];
  for (int i = 0; i < g_argc; i++)
  {
    g_argv[i] = new char[strlen("gazebo") + 1];
    snprintf(g_argv[i], strlen("gazebo") + 1, "gazebo");
  }

  // Register custom message handler
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
  qInstallMessageHandler(messageHandler);
#else
  qInstallMsgHandler(messageHandler);
#endif

#ifdef __APPLE__
  // gazebo issue #2531
  // seems to be related to QTBUG-71044
  // Setting the QT_MAC_WANTS_LAYER environment variable fixes the problem
  // on Mojave + Qt 5.12
  ignition::math::SemanticVersion sv;
  sv.Parse(QSysInfo::productVersion().toStdString());
  ignition::math::SemanticVersion mojave(10, 14);
  if (sv >= mojave)
  {
    QByteArray result = qgetenv("QT_MAC_WANTS_LAYER");
    if (result.isEmpty())
      qputenv("QT_MAC_WANTS_LAYER", QByteArray("1"));
  }
#endif

  g_app = new QApplication(g_argc, g_argv);
  set_style();

  if (!gui::register_metatypes())
    std::cerr << "Unable to register Qt metatypes" << std::endl;

  g_splashScreen = new gui::SplashScreen();

  g_main_win = new gui::MainWindow();

  g_main_win->Load();

  return true;
}

/////////////////////////////////////////////////
unsigned int gui::get_entity_id(const std::string &_name)
{
  if (g_main_win)
    return g_main_win->EntityId(_name);
  else
    return 0;
}


/////////////////////////////////////////////////
bool gui::run(int _argc, char **_argv)
{
  IGN_PROFILE_THREAD_NAME("gzclient");

  // Initialize the informational logger. This will log warnings, and errors.
  gzLogInit("client-", "gzclient.log");

  // Make sure the model database has started
  gazebo::common::ModelDatabase::Instance()->Start();

  if (!parse_args(_argc, _argv))
    return false;

  if (!gazebo::client::setup(_argc, _argv))
    return false;

  if (!gazebo::gui::load())
    return false;

  gazebo::gui::init();

  // the plugins have to be created after g_app has been created,
  // otherwise Qt will complain about no existing QApplication.
  GZ_ASSERT(g_app, "QApplication must have been created");

  gazebo::gui::MainWindow *mainWindow = gazebo::gui::get_main_window();

  GZ_ASSERT(mainWindow, "Main Window has to be available!");
  GZ_ASSERT(mainWindow->RenderWidget(),
            "Main window's RenderWidget must have been created");

  mainWindow->RenderWidget()->AddPlugins(g_plugins_to_load);

#ifndef _WIN32
  // Now that we're about to run, install a signal handler to allow for
  // graceful shutdown on Ctrl-C.
  struct sigaction sigact;
  sigact.sa_flags = 0;
  sigact.sa_handler = signal_handler;
  if (sigemptyset(&sigact.sa_mask) != 0)
    std::cerr << "sigemptyset failed while setting up for SIGINT" << std::endl;
  if (sigaction(SIGINT, &sigact, NULL))
  {
    std::cerr << "sigaction(2) failed while setting up for SIGINT" << std::endl;
    return false;
  }

  // The following was added in
  // https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2923,
  // but it is causing shutdown issues when gazebo is used with ros.
  // if (sigaction(SIGTERM, &sigact, NULL))
  // {
  //   std::cerr << "sigaction(15) failed while setting up for SIGTERM"
  //             << std::endl;
  //   return false;
  // }
#endif

  g_app->exec();

  gazebo::gui::fini();

  delete g_splashScreen;
  delete g_main_win;
  gazebo::client::shutdown();
  return true;
}

/////////////////////////////////////////////////
void gui::stop()
{
  event::Events::stop();
  gazebo::client::shutdown();
  g_active_camera.reset();
  g_app->quit();
}

/////////////////////////////////////////////////
void gui::set_world(const std::string &_name)
{
  g_worldname = _name;
}

/////////////////////////////////////////////////
std::string gui::get_world()
{
  return g_worldname;
}

/////////////////////////////////////////////////
void gui::set_active_camera(rendering::UserCameraPtr _cam)
{
  g_active_camera = _cam;
}

/////////////////////////////////////////////////
void gui::clear_active_camera()
{
  g_active_camera.reset();
}

/////////////////////////////////////////////////
rendering::UserCameraPtr gui::get_active_camera()
{
  return g_active_camera;
}

/////////////////////////////////////////////////
bool gui::has_entity_name(const std::string &_name)
{
  return g_main_win->HasEntityName(_name);
}

/////////////////////////////////////////////////
bool gui::register_metatypes()
{
  // Register common::Time as a type that can be used in signals and slots.
  // Q_DECLARE_METATYPE is also required, see above.
  qRegisterMetaType<common::Time>();

  // Register std::string as a type that can be used in signals and slots.
  // Q_DECLARE_METATYPE is also required, see above.
  qRegisterMetaType<std::string>();

  // Register std::set<std::string> as a type that can be used in signals and
  // slots. Q_DECLARE_METATYPE is also required, see above.
  qRegisterMetaType< std::set<std::string> >();

  // Register ignition::msgs::JointCmd as a type that can be used in signals and
  // slots. Q_DECLARE_METATYPE is also required, see above.
  qRegisterMetaType<ignition::msgs::JointCmd>();

  return true;
}

/////////////////////////////////////////////////
bool gui::saveINI(const boost::filesystem::path &_file)
{
  bool result = true;
  try
  {
    boost::property_tree::ini_parser::write_ini(_file.string(), g_propTree);
  }
  catch(...)
  {
    gzerr << "Unable to save INI file[" << _file << "]\n";
    result = false;
  }
  return result;
}

/////////////////////////////////////////////////
gui::MainWindow *gui::get_main_window()
{
  return g_main_win;
}
