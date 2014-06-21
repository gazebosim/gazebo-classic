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
#include <google/protobuf/descriptor.h>
#include <google/protobuf/message.h>

#include "gazebo/transport/TransportIface.hh"
#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Subscriber.hh"

#include "gazebo/gui/IncrementalPlot.hh"
#include "gazebo/gui/TopicPlot.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
TopicPlot::TopicPlot(QWidget *_parent)
  : QDialog(_parent)
{
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();

  this->clockSub = this->node->Subscribe("~/clock", &TopicPlot::OnClock, this);

  this->setWindowIcon(QIcon(":/images/gazebo.svg"));
  this->setWindowTitle("Gazebo: Plot");

  this->plotLayout = new QVBoxLayout;

  QScrollArea *plotScrollArea = new QScrollArea(this);
  plotScrollArea->setLineWidth(0);
  plotScrollArea->setFrameShape(QFrame::NoFrame);
  plotScrollArea->setFrameShadow(QFrame::Plain);
  plotScrollArea->setSizePolicy(QSizePolicy::Minimum,
                                QSizePolicy::Minimum);

  plotScrollArea->setWidgetResizable(true);
  plotScrollArea->viewport()->installEventFilter(this);

  QFrame *plotFrame = new QFrame(plotScrollArea);
  plotFrame->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
  plotFrame->setLayout(this->plotLayout);

  plotScrollArea->setWidget(plotFrame);

  this->plot = new IncrementalPlot(this);
  this->plotLayout->addWidget(this->plot);

  QHBoxLayout *mainLayout = new QHBoxLayout;
  mainLayout->addWidget(plotScrollArea, 1);

  this->setLayout(mainLayout);
  this->setSizeGripEnabled(true);

  this->setWindowFlags(Qt::Window);

  QTimer *displayTimer = new QTimer(this);
  connect(displayTimer, SIGNAL(timeout()), this, SLOT(Update()));
  displayTimer->start(60);
}

/////////////////////////////////////////////////
TopicPlot::~TopicPlot()
{
}

/////////////////////////////////////////////////
void TopicPlot::Init(const std::string &_topic, const std::string &_field)
{
  if (_topic.empty())
    return;

  this->msgSub.reset();

  this->msgTypeName = transport::getTopicMsgType(
      this->node->DecodeTopicName(_topic));

  this->msg = msgs::MsgFactory::NewMsg(this->msgTypeName);

  this->msgSub = this->node->Subscribe(_topic, &TopicPlot::OnMsg, this);

  this->topic = _topic;

  boost::split(this->fields, _field, boost::is_any_of("/"));

  this->plot->AddCurve(QString::fromStdString(this->topic));
}

/////////////////////////////////////////////////
void TopicPlot::SetPeriod(unsigned int _seconds)
{
  this->plot->SetPeriod(_seconds);
}

/////////////////////////////////////////////////
void TopicPlot::Update()
{
  boost::mutex::scoped_lock lock(this->mutex);

  // Update all the plots
  this->plot->Update();
}

/////////////////////////////////////////////////
void TopicPlot::OnClock(ConstTimePtr &_msg)
{
  this->time = msgs::Convert(*_msg);
}

/////////////////////////////////////////////////
void TopicPlot::OnMsg(const std::string &_msg)
{
  size_t openBracketIndex = std::string::npos;
  bool failed = false;

  // Convert the raw data to a message.
  this->msg->ParseFromString(_msg);

  const google::protobuf::FieldDescriptor *field;
  const google::protobuf::Message *tmpMsg = this->msg.get();
  const google::protobuf::Descriptor *descriptor = tmpMsg->GetDescriptor();
  const google::protobuf::Reflection *reflection = tmpMsg->GetReflection();

  // Loop through all the fields, except the last
  std::vector<std::string>::iterator iter = this->fields.begin();
  for (; iter != this->fields.end() - 1; ++iter)
  {
    if (!descriptor || !reflection)
    {
      gzerr << "Unable to find message to plot using specified fields.\n";
      return;
    }

    // Find the index of an open bracket within the field.
    openBracketIndex = (*iter).find("[");

    // If an open bracket exists, then field refers to a repeated message.
    if (openBracketIndex != std::string::npos)
    {
      // Get just the name of the field
      std::string fieldName = (*iter).substr(0, openBracketIndex);

      // Get the parameter inside the brackets.
      std::string fieldParam = (*iter).substr(openBracketIndex + 1,
          (*iter).find("]") - openBracketIndex - 1);

      // Get the Protobuf field
      field = descriptor->FindFieldByName(fieldName);

      int index = 0;
      size_t equalIndex = fieldParam.find("=");

      // If the field param has an equal sign, then a comparison between a 
      // field (left side of =) within the repeated message and a
      // value (right side of =) must be done in order to find the correct
      // repeated message.
      if (equalIndex != std::string::npos)
      {
        // A set of protobuf objects for searching the repeated messages.
        const google::protobuf::Message *rMsg = NULL;
        const google::protobuf::Descriptor *rDescriptor = NULL;
        const google::protobuf::Reflection *rReflection = NULL;
        const google::protobuf::FieldDescriptor *rField = NULL;

        if (equalIndex == 0)
        {
          gzerr << "Invalid field comparison. Missing left side "
                << "of equation for [" << fieldParam << "]\n";
          return;
        }

        if (equalIndex+1 == fieldParam.size())
        {
          gzerr << "Invalid field comparison. Missing right side "
                << "of equation for [" << fieldParam << "]\n";
          return;
        }

        // The key for the field within the repeated message.
        std::string key = fieldParam.substr(0, equalIndex);

        // The value for the field within the repeated message.
        std::string value = fieldParam.substr(equalIndex + 1);

        while (true)
        {
          try
          {
            // Get one of the repeated messages.
            rMsg = &(reflection->GetRepeatedMessage(*tmpMsg, field, index));
            failed = !rMsg;
          }
          catch(...)
          {
            failed = true;
          }

          if (failed)
          {
            gzerr << "Unable to find field[" << key << "] with value["
              << value << "] in topic[" << this->topic << "]\n";
            return;
          }

          // Get the descriptor and reflection for the message
          rDescriptor = rMsg->GetDescriptor();
          rReflection = rMsg->GetReflection();

          // Find the correct field within the repeated message.
          rField = rDescriptor->FindFieldByName(key);

          if (!rField)
          {
            gzerr << "Unable to find field[" << key << "] Within repeated "
                  << "message of type[" << rMsg->GetTypeName() << "]\n";
            return;
          }

          // Get the value for comparison
          std::string check = rReflection->GetString(*rMsg, rField);

          // Break if the value inside the repeated message matches the
          // value specified.
          if (check == value)
            break;

          index++;
        }
      }
      // Otherwise the field param contains an integer that is the index of
      // the repeated message to retrieve.
      else
      {
        try
        {
          index = boost::lexical_cast<int>(fieldParam);
        }
        catch (...)
        {
          gzerr << "Unable to convert[" << fieldParam << "] to an integer.\n";
          return;
        }
      }

      tmpMsg = &(reflection->GetRepeatedMessage(*tmpMsg, field, index));
    }
    // Otherwise the field refers to a non-repeated message
    else
    {
      try
      {
        const google::protobuf::FieldDescriptor *newField =
          descriptor->FindFieldByName(*iter);

        if (newField)
          tmpMsg = &(reflection->GetMessage(*tmpMsg, newField));

        failed = !tmpMsg || !newField;
      }
      catch (...)
      {
        failed = true;
      }

      // Print an error message, and return if the message couldn't be
      // found.
      if (failed)
      {
        gzerr << "Unable to find child message with name[" << *iter << "]\n";
        return;
      }
    }

    // Get descriptor and reflection pointers to the new message.
    descriptor = tmpMsg->GetDescriptor();
    reflection = tmpMsg->GetReflection();
  }

  // The last thing iter contains is the name of the field with the 
  // value to plot. Get the protobuf field now.
  field = descriptor->FindFieldByName(*iter);

  if (!field)
  {
    gzerr << "Unable to get field[" << *iter << "] for plotting\n";
    return;
  }

  // TODO Get the type of the field, and cast to double.
  // Get the value of the field.
  double value = 0.0;
  failed = false;

  try
  {
    switch (field->cpp_type())
    {
      case google::protobuf::FieldDescriptor::CPPTYPE_INT32:
        value = boost::lexical_cast<double>(
            reflection->GetInt32(*tmpMsg, field));
        break;
      case google::protobuf::FieldDescriptor::CPPTYPE_INT64:
        value = boost::lexical_cast<double>(
            reflection->GetInt64(*tmpMsg, field));
        break;
      case google::protobuf::FieldDescriptor::CPPTYPE_UINT32:
        value = boost::lexical_cast<double>(
            reflection->GetUInt32(*tmpMsg, field));
        break;
      case google::protobuf::FieldDescriptor::CPPTYPE_UINT64:
        value = boost::lexical_cast<double>(
            reflection->GetUInt64(*tmpMsg, field));
        break;
      case google::protobuf::FieldDescriptor::CPPTYPE_DOUBLE:
        value = reflection->GetDouble(*tmpMsg, field);
        break;
      case google::protobuf::FieldDescriptor::CPPTYPE_FLOAT:
        value = static_cast<double>(reflection->GetFloat(*tmpMsg, field));
        break;
      case google::protobuf::FieldDescriptor::CPPTYPE_BOOL:
        value = boost::lexical_cast<double>(
            reflection->GetBool(*tmpMsg, field));
        break;
      case google::protobuf::FieldDescriptor::CPPTYPE_ENUM:
        gzerr << "Unable to plot field with value type[ENUM]\n";
        failed = true;
        break;
      case google::protobuf::FieldDescriptor::CPPTYPE_STRING:
        gzerr << "Unable to plot field with value type[STRING]\n";
        failed = true;
        break;
      case google::protobuf::FieldDescriptor::CPPTYPE_MESSAGE:
        gzerr << "Unable to plot field with value type[MESSAGE]\n";
        failed = true;
        break;
      default:
        gzerr << "Invalid message type[" << field->cpp_type() << "]\n";
        failed = true;
        break;
    }
  }
  catch(...)
  {
    gzerr << "Unable to convert request value to a double.\n";
    failed = true;
  }

  if (!failed)
  {
    // Create a QT point for plotting purposes.
    QPointF pt(this->time.Double(), value);

    // Add the point to the plot.
    this->plot->Add(QString::fromStdString(this->topic), pt);
  }
}
