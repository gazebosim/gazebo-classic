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
#include "transport/Node.hh"
#include "transport/Transport.hh"
#include "gui/JointControlWidget.hh"

using namespace gazebo;
using namespace gui;

/*class MySlider : public QSlider
{
  public: MySlider(int _id, QParent *_parent)
          : QSlider(_parent), id(_id)
          {}
  Q_SIGNALS: void valueChanged(int _value, int _id)
  {
  }

  private: int id;
};
*/

/////////////////////////////////////////////////
JointControlWidget::JointControlWidget(const std::string &_modelName,
                                       QWidget *_parent)
  : QWidget(_parent)
{
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  this->jointPub = this->node->Advertise<msgs::JointCmd>(
      std::string("~/") + _modelName + "/joint_cmd");

  this->requestMsg = msgs::CreateRequest("entity_info");
  this->requestMsg->set_data(_modelName);

  std::cout << "Joint send request\n";
  msgs::Response response = transport::request("default", *this->requestMsg);
  std::cout << "Joint got request\n";

  msgs::Model modelMsg;
  modelMsg.ParseFromString(response.serialized_data());


  QFormLayout *formLayout = new QFormLayout;
  for (int i = 0; i < modelMsg.joint_size(); ++i)
  {
    QSlider *slider = new QSlider(Qt::Horizontal, this);
    formLayout->addRow(tr(modelMsg.joint(i).name().c_str()), slider);
    connect(slider, SIGNAL(valueChanged(int)), this, SLOT(OnChanged(int)));
  }
  this->setLayout(formLayout);
  this->layout()->setContentsMargins(2, 2, 2, 2);
}

/////////////////////////////////////////////////
JointControlWidget::~JointControlWidget()
{
}

/////////////////////////////////////////////////
void JointControlWidget::OnChanged(int _index, int _value)
{
  printf("%d %d\n", _index,_value);
}

/////////////////////////////////////////////////
void JointControlWidget::Load(const std::string &/*_modelName*/)
{
}
