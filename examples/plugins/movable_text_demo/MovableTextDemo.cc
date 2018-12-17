/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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
#include <sstream>
#include <gazebo/rendering/RenderingIface.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Visual.hh>
#include "MovableTextDemo.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(MovableTextDemo)

/////////////////////////////////////////////////
MovableTextDemo::MovableTextDemo() : GUIPlugin()
{
  // Layout
  auto mainLayout = new QVBoxLayout;
  mainLayout->setContentsMargins(0, 0, 0, 0);

  auto frame = new QFrame();
  mainLayout->addWidget(frame);

  this->setLayout(mainLayout);

  // Position and resize this widget
  this->move(10, 10);
  this->resize(350, 250);

  this->setStyleSheet(
      "QFrame {background-color: rgba(100, 100, 100, 255); color: black;}");
}

/////////////////////////////////////////////////
void MovableTextDemo::Load(sdf::ElementPtr /*_elem*/)
{
  // Get scene
  auto scene = rendering::get_scene();
  if (!scene)
  {
    gzerr << "Scene not found" << std::endl;
    return;
  }

  // Get root visual
  auto worldVis = scene->WorldVisual();
  if (!worldVis)
  {
    gzerr << "World visual not found" << std::endl;
    return;
  }

  // Movable text
  this->text.Load("DEMO_TEXT", "Gazebo");
  this->text.SetShowOnTop(true);

  // Visual to hold text
  rendering::VisualPtr vis(new rendering::Visual(
      "DEMO_TEXT_VIS", worldVis, false));
  vis->Load();
  vis->GetSceneNode()->attachObject(&(this->text));
  vis->GetSceneNode()->setInheritScale(false);

  // Widgets
  auto mainLayout = new QVBoxLayout();
  mainLayout->setContentsMargins(5, 5, 5, 5);

  auto frame = this->findChild<QFrame *>();
  frame->setLayout(mainLayout);

  // Text
  {
    auto w = new QLineEdit(QString::fromStdString(this->text.Text()));
    this->connect(w, &QLineEdit::editingFinished, [=]()
    {
      auto v = w->text().toStdString();
      this->text.SetText(v);
    });

    auto l = new QHBoxLayout();
    l->addWidget(new QLabel("Text"));
    l->addWidget(w);
    mainLayout->addLayout(l);
  }

  // Font
  {
    auto w = new QLineEdit(QString::fromStdString(this->text.FontName()));
    this->connect(w, &QLineEdit::editingFinished, [=]()
    {
      auto v = w->text().toStdString();
      try
      {
        this->text.SetFontName(v);
      }
      catch(Ogre::Exception &e)
      {
        gzerr << "Failed to change font to [" << v << "]: "
              << e.getFullDescription() << std::endl;
      }
    });

    auto l = new QHBoxLayout();
    l->addWidget(new QLabel("Font"));
    l->addWidget(w);
    mainLayout->addLayout(l);
  }

  // Color
  {
    auto color = this->text.Color();

    auto r = new QDoubleSpinBox();
    r->setValue(color.R());
    r->setMaximum(1.0);
    r->setMinimum(0.0);
    r->setSingleStep(0.1);

    auto g = new QDoubleSpinBox();
    g->setValue(color.G());
    g->setMaximum(1.0);
    g->setMinimum(0.0);
    g->setSingleStep(0.1);

    auto b = new QDoubleSpinBox();
    b->setValue(color.B());
    b->setMaximum(1.0);
    b->setMinimum(0.0);
    b->setSingleStep(0.1);

    auto a = new QDoubleSpinBox();
    a->setValue(color.A());
    a->setMaximum(1.0);
    a->setMinimum(0.0);
    a->setSingleStep(0.1);

    auto cb = [=](double)
    {
      ignition::math::Color color(r->value(),
                                  g->value(),
                                  b->value(),
                                  a->value());
      this->text.SetColor(color);
    };

    this->connect(r, static_cast<void (QDoubleSpinBox::*)(double)>(
        &QDoubleSpinBox::valueChanged), cb);
    this->connect(g, static_cast<void (QDoubleSpinBox::*)(double)>(
        &QDoubleSpinBox::valueChanged), cb);
    this->connect(b, static_cast<void (QDoubleSpinBox::*)(double)>(
        &QDoubleSpinBox::valueChanged), cb);
    this->connect(a, static_cast<void (QDoubleSpinBox::*)(double)>(
        &QDoubleSpinBox::valueChanged), cb);

    auto l = new QHBoxLayout();
    l->addWidget(new QLabel("Color"));
    l->addWidget(new QLabel("<b>R</b>"));
    l->addWidget(r);
    l->addWidget(new QLabel("<b>G</b>"));
    l->addWidget(g);
    l->addWidget(new QLabel("<b>B</b>"));
    l->addWidget(b);
    l->addWidget(new QLabel("<b>A</b>"));
    l->addWidget(a);
    mainLayout->addLayout(l);
  }

  // Char height
  {
    auto w = new QDoubleSpinBox();
    w->setValue(this->text.CharHeight());
    w->setMaximum(100.0);
    w->setMinimum(-100.0);
    w->setSingleStep(0.1);
    this->connect(w, static_cast<void (QDoubleSpinBox::*)(double)>(
        &QDoubleSpinBox::valueChanged), [=](double _v)
    {
      this->text.SetCharHeight(_v);
    });

    auto l = new QHBoxLayout();
    l->addWidget(new QLabel("Char height"));
    l->addWidget(w);
    l->addWidget(new QLabel("m"));
    mainLayout->addLayout(l);
  }

  // Space width
  {
    auto w = new QDoubleSpinBox();
    w->setValue(this->text.SpaceWidth());
    w->setMaximum(100.0);
    w->setMinimum(-100.0);
    w->setSingleStep(0.1);
    this->connect(w, static_cast<void (QDoubleSpinBox::*)(double)>(
        &QDoubleSpinBox::valueChanged), [=](double _v)
    {
      this->text.SetSpaceWidth(_v);
    });

    auto l = new QHBoxLayout();
    l->addWidget(new QLabel("Space width"));
    l->addWidget(w);
    l->addWidget(new QLabel("m"));
    mainLayout->addLayout(l);
  }

  // Baseline
  {
    auto w = new QDoubleSpinBox();
    w->setValue(this->text.Baseline());
    w->setMaximum(100.0);
    w->setMinimum(-100.0);
    w->setSingleStep(0.1);
    this->connect(w, static_cast<void (QDoubleSpinBox::*)(double)>(
        &QDoubleSpinBox::valueChanged), [=](double _v)
    {
      this->text.SetBaseline(_v);
    });

    auto l = new QHBoxLayout();
    l->addWidget(new QLabel("Baseline"));
    l->addWidget(w);
    l->addWidget(new QLabel("m"));
    mainLayout->addLayout(l);
  }

  // Show on top
  {
    auto w = new QCheckBox();
    w->setChecked(this->text.ShowOnTop());
    this->connect(w, &QCheckBox::toggled, [=](bool _v)
    {
      this->text.SetShowOnTop(_v);
    });

    auto l = new QHBoxLayout();
    l->addWidget(new QLabel("Show on top"));
    l->addWidget(w);
    mainLayout->addLayout(l);
  }

  // Alignment
  {
    auto h = new QComboBox();
    h->addItem("H_LEFT");
    h->addItem("H_CENTER");
    h->setCurrentIndex(0);

    auto v = new QComboBox();
    v->addItem("V_BELOW");
    v->addItem("V_ABOVE");
    h->setCurrentIndex(0);

    auto cb = [=](const QString &)
    {
      auto ha = h->currentIndex() == 0 ? rendering::MovableText::H_LEFT :
                                         rendering::MovableText::H_CENTER;
      auto va = v->currentIndex() == 0 ? rendering::MovableText::V_BELOW :
                                         rendering::MovableText::V_ABOVE;
      this->text.SetTextAlignment(ha, va);
    };

    this->connect(h, static_cast<void (QComboBox::*)(const QString &)>(
        &QComboBox::currentIndexChanged), cb);
    this->connect(v, static_cast<void (QComboBox::*)(const QString &)>(
        &QComboBox::currentIndexChanged), cb);

    auto l = new QHBoxLayout();
    l->addWidget(new QLabel("Alignment"));
    l->addWidget(h);
    l->addWidget(v);
    mainLayout->addLayout(l);
  }

  // Position
  {
    auto x = new QDoubleSpinBox();
    x->setMaximum(100.0);
    x->setMinimum(-100.0);
    x->setSingleStep(0.1);

    auto y = new QDoubleSpinBox();
    y->setMaximum(100.0);
    y->setMinimum(-100.0);
    y->setSingleStep(0.1);

    auto z = new QDoubleSpinBox();
    z->setMaximum(100.0);
    z->setMinimum(-100.0);
    z->setSingleStep(0.1);

    auto cb = [=](double)
    {
      ignition::math::Vector3d pos(x->value(),
                                   y->value(),
                                   z->value());
      vis->SetPosition(pos);
    };

    this->connect(x, static_cast<void (QDoubleSpinBox::*)(double)>(
        &QDoubleSpinBox::valueChanged), cb);
    this->connect(y, static_cast<void (QDoubleSpinBox::*)(double)>(
        &QDoubleSpinBox::valueChanged), cb);
    this->connect(z, static_cast<void (QDoubleSpinBox::*)(double)>(
        &QDoubleSpinBox::valueChanged), cb);

    auto l = new QHBoxLayout();
    l->addWidget(new QLabel("Position"));
    l->addWidget(new QLabel("<b>X</b>"));
    l->addWidget(x);
    l->addWidget(new QLabel("m"));
    l->addWidget(new QLabel("<b>Y</b>"));
    l->addWidget(y);
    l->addWidget(new QLabel("m"));
    l->addWidget(new QLabel("<b>Z</b>"));
    l->addWidget(z);
    l->addWidget(new QLabel("m"));
    mainLayout->addLayout(l);
  }
}

/////////////////////////////////////////////////
MovableTextDemo::~MovableTextDemo()
{
}

