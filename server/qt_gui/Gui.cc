/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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

#include <boost/thread.hpp>
#include "XMLConfig.hh"
#include "GLWindow.hh"
#include "Gui.hh"

using namespace gazebo;
QApplication *g_app = NULL;
Gui *g_gui = NULL;

void qt_setup(int argc, char **argv)
{
  g_app = new QApplication(argc, argv);

  /*QMainWindow *window = new QMainWindow;
  Ui::SharonWindow ui;
  ui.setupUi(window);
  window->show();
  */

  /*QUiLoader loader;
  QFile file("/u/nkoenig/Downloads/sharon_simple.ui");
  file.open(QFile::ReadOnly);
  QMainWindow *formWidget = static_cast<QMainWindow*>(loader.load(&file));
  file.close();

  formWidget->show();
  */
  g_gui = new Gui(0,0,0,0,"hello");
  g_gui->show();
  g_gui->Load(NULL);
}

void qt_run()
{
  g_app->exec();
}

/////////////////////////////////////////////////
Gui::Gui(int x, int y, int width, int height, const std::string &_name)
  : QMainWindow(), renderWidget(0)
{
  setupUi(this);

 (void) new QShortcut(Qt::CTRL + Qt::Key_Q, this, SLOT(close()));


  QVBoxLayout *layout = this->findChild<QVBoxLayout*>("verticalLayout");
  if (!layout)
    std::cout << "\n\n!!!NOT Found gazebo widget!!\n\n";

  //QWidget *mainWidget = new QWidget;
  //QVBoxLayout *mainLayout = new QVBoxLayout;
  //mainWidget->show();
  //this->setCentralWidget(formWidget);

  this->renderWidget = new GLWindow(this);
  layout->addWidget(renderWidget);
  this->renderWidget->show();
  //mainLayout->addWidget(this->renderWidget);
 
  //mainLayout->addWidget(formWidget);
  // mainWidget->setLayout(mainLayout);
  // mainWidget->show();

 QRadioButton *button = this->findChild<QRadioButton*>("radioButton");
 connect(button, SIGNAL(clicked()), this->renderWidget, SLOT(FrontView()));

 button = this->findChild<QRadioButton*>("radioButton_2");
 connect(button, SIGNAL(clicked()), this->renderWidget, SLOT(SideView()));

 button = this->findChild<QRadioButton*>("radioButton_3");
 connect(button, SIGNAL(clicked()), this->renderWidget, SLOT(AngledView()));

}

/////////////////////////////////////////////////
Gui::~Gui()
{
}

/////////////////////////////////////////////////
void Gui::Load(XMLConfigNode *node)
{
}

/////////////////////////////////////////////////
void Gui::Init()
{
  this->renderWidget->Init();
}

/////////////////////////////////////////////////
void Gui::Save(std::string &prefix, std::ostream &stream)
{
}

/////////////////////////////////////////////////
void Gui::closeEvent(QCloseEvent * /*_event*/)
{
  delete this->renderWidget;
}

/////////////////////////////////////////////////
void Gui::CreateCameras()
{
 this->renderWidget->show();
 this->renderWidget->CreateCameras();
}

/////////////////////////////////////////////////
void Gui::Update()
{
  g_app->processEvents();
  //this->renderWidget->paintEvent();
}
