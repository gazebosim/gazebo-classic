/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include <QWebView>

#include "gazebo/gui/MainWindow.hh"
#include "gazebo/gui/HotkeyDialog.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
HotkeyDialog::HotkeyDialog(QWidget *_parent)
  : QDialog(_parent)
{
  this->setObjectName("hotkeyChart");
  this->setWindowTitle("Gazebo Keyboard Shortcuts");
  this->setWindowFlags(Qt::Window | Qt::WindowCloseButtonHint |
      Qt::WindowStaysOnTopHint | Qt::CustomizeWindowHint);
  this->setWindowModality(Qt::NonModal);

  // Dialog size
  int desiredHeight = 980;
  int desired2ColumnWidth = 1850;
  int desired1ColumnWidth = 900;
  int minHeight = desiredHeight;
  int minWidth = desired2ColumnWidth;

  MainWindow *mainWindow = qobject_cast<MainWindow *>(_parent);
  if (mainWindow)
  {
    // Can't be higher than main window
    minHeight = std::min(desiredHeight, (mainWindow->height()-100));

    // Can't be wider than main window
    minWidth = std::min(desired2ColumnWidth, (mainWindow->width()-100));

    // If too small to fit 2 columns, decrease white space by tightening to
    // 1 column
    if (minWidth < desired2ColumnWidth)
    {
      minWidth = std::min(desired1ColumnWidth, minWidth);
    }
  }

  // Load the html file into the web view
  QWebView *view = new QWebView(this);
  view->setMinimumWidth(minWidth);
  view->setMinimumHeight(minHeight);
  view->load(QUrl("qrc:///hotkeyTable.html"));
  view->page()->setLinkDelegationPolicy(QWebPage::DelegateAllLinks);
  connect(view, SIGNAL(linkClicked(QUrl)), this, SLOT(OnLinkClicked(QUrl)));

  // Layout for the dialog
  QHBoxLayout *mainLayout = new QHBoxLayout();
  mainLayout->addWidget(view);
  mainLayout->setSizeConstraint(QLayout::SetFixedSize);
  this->setLayout(mainLayout);
}

/////////////////////////////////////////////////
void HotkeyDialog::OnLinkClicked(QUrl _url)
{
  // Open on a browser
  QDesktopServices::openUrl(_url);
}

