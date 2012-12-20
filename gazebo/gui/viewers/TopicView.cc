/*
 * Copyright 2012 Open Source Robotics Foundation
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
#include "gazebo/gui/Gui.hh"
#include "gazebo/gui/GuiEvents.hh"

#include "gazebo/transport/Transport.hh"
#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"

#include "gazebo/gui/viewers/TopicView.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
TopicView::TopicView(const std::string &_msgTypeName,
                                       QWidget *_parent)
: QWidget(_parent), msgTypeName(_msgTypeName)
{
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();

  this->setWindowIcon(QIcon(":/images/gazebo.svg"));
  this->setWindowTitle(tr("Gazebo: Topic View"));
  this->setObjectName("cameraSensor");

  // Create the topic label and combo box
  // {
  QHBoxLayout *topicLayout = new QHBoxLayout;
  QLabel *topicLabel = new QLabel(tr("Topic: "));
  this->topicCombo = new QComboBox(this);
  this->topicCombo->setObjectName("comboList");
  this->topicCombo->setMinimumSize(300, 25);
  this->UpdateTopicList();
  connect(this->topicCombo, SIGNAL(currentIndexChanged(int)),
          this, SLOT(OnTopicChanged(int)));


  topicLayout->addSpacing(10);
  topicLayout->addWidget(topicLabel);
  topicLayout->addWidget(this->topicCombo);
  topicLayout->addSpacing(10);
  topicLayout->addStretch(4);
  // }

  // Create the Hz and bandwidth labels
  // {
  QHBoxLayout *infoLayout = new QHBoxLayout;
  QLabel *hzLabel = new QLabel("Hz: ");
  this->hzEdit = new QLineEdit;
  this->hzEdit->setReadOnly(true);
  this->hzEdit->setFixedWidth(80);

  QLabel *bandwidthLabel = new QLabel("Bandwidth: ");
  this->bandwidthEdit = new QLineEdit;
  this->bandwidthEdit->setReadOnly(true);
  this->bandwidthEdit->setFixedWidth(110);

  infoLayout->addSpacing(10);
  infoLayout->addWidget(hzLabel);
  infoLayout->addWidget(this->hzEdit);
  infoLayout->addSpacing(4);
  infoLayout->addStretch(1);
  infoLayout->addWidget(bandwidthLabel);
  infoLayout->addWidget(this->bandwidthEdit);
  infoLayout->addStretch(4);
  // }

  // Create the frame used to display information
  this->frame = new QFrame;

  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->addLayout(topicLayout);
  mainLayout->addLayout(infoLayout);
  mainLayout->addWidget(frame);
  this->setLayout(mainLayout);
  this->layout()->setContentsMargins(4, 4, 4, 4);

  QTimer::singleShot(500, this, SLOT(Update()));
}

/////////////////////////////////////////////////
TopicView::~TopicView()
{
}

/////////////////////////////////////////////////
void TopicView::Update()
{
  // Update the child class.
  this->UpdateImpl();

  // Update the Hz output
  {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(2) << this->hz;
    this->hzEdit->setText(tr(stream.str().c_str()));
  }

  // Update the Bandwidth output
  {
    std::ostringstream stream;

    // Sum up the byte information
    int sumBytes = 0;
    for (std::list<int>::iterator iter = this->msgSizes.begin();
        iter != this->msgSizes.end(); ++iter)
    {
      sumBytes += *iter;
    }

    // Compute the bandwidth
    common::Time dt = this->times.back() - this->times.front();
    double bandwidth = sumBytes / dt.Double();

    // Format the bandwidth output
    stream << std::fixed << std::setprecision(2);

    if (bandwidth < 1000)
      stream << bandwidth << " B/s";
    else if (bandwidth < 1000000)
      stream << bandwidth / 1024.0f << " KB/s";
    else
      stream << bandwidth/1.049e6 << " MB/s";

    this->bandwidthEdit->setText(tr(stream.str().c_str()));
  }

  // Set the timer to update again.
  QTimer::singleShot(500, this, SLOT(Update()));
}

/////////////////////////////////////////////////
void TopicView::OnMsg(const common::Time &_dataTime, int _size)
{
  // Calculate the Hz value.
  if (_dataTime != this->prevTime)
    this->hz = 1.0 / (_dataTime - this->prevTime).Double();

  // Store the previous time for future Hz calculations.
  this->prevTime = _dataTime;

  // Store the message size and clock time that it was received.
  this->msgSizes.push_back(_size);
  this->times.push_back(common::Time::GetWallTime());

  // Maintain a buffer of only 100 data points.
  if (this->msgSizes.size() > 100)
  {
    this->msgSizes.pop_front();
    this->times.pop_front();
  }
}

/////////////////////////////////////////////////
void TopicView::OnTopicChanged(int _index)
{
  // Set the current topic based on the index of the item selected in the
  // combobox
  this->SetTopic(this->topicCombo->itemText(_index).toStdString());
}

/////////////////////////////////////////////////
void TopicView::UpdateTopicList()
{
  std::list<std::string> topics;

  // First clear out the combo box.
  this->topicCombo->clear();

  // Get the list of all topics filtered by our message type.
  topics = transport::getAdvertisedTopics(this->msgTypeName);

  // Add each topic to the combo box.
  for (std::list<std::string>::iterator iter = topics.begin();
       iter != topics.end(); ++iter)
  {
    // Get the shorthand notation for the topic.
    std::string topicName = this->node->EncodeTopicName(*iter);

    std::cout << "Adding topic[" << topicName << "]\n";
    this->topicCombo->addItem(QString::fromStdString(topicName));
  }
}
