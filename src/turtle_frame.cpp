/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "turtlesim/turtle_frame.h"

#include <QPointF>

#include <ros/package.h>
#include <cstdlib>
#include <ctime>

#define DEFAULT_BG_R 0x45
#define DEFAULT_BG_G 0x56
#define DEFAULT_BG_B 0xff

namespace turtlesim
{

// TurtleFrame(QWidget* parent = 0, Qt::WindowFlags f = 0); 表明TurtleFrame是一个窗口
TurtleFrame::TurtleFrame(QWidget* parent, Qt::WindowFlags f)
: QFrame(parent, f) // QFrame 窗口初始化
, path_image_(500, 500, QImage::Format_ARGB32) // QImage(int width, int height, Format format)
, path_painter_(&path_image_) // QPainter(QPaintDevice *device) 初始化，QPaintDevice泛指一切可绘制QT部件
, frame_count_(0)
, id_counter_(0)
{
  setFixedSize(500, 500); // 窗体大小高宽500*500
  setWindowTitle("TurtleSim");

  srand(time(NULL)); // 表示设置一个随机种子，每次运行都能保证随机种子不同

  update_timer_ = new QTimer(this);
  update_timer_->setInterval(16); // 16ms定时器间隔
  update_timer_->start();

  // 信号槽绑定，超时调用槽函数onUpdate
  connect(update_timer_, SIGNAL(timeout()), this, SLOT(onUpdate()));

  // 背景色参数
  nh_.setParam("background_r", DEFAULT_BG_R);
  nh_.setParam("background_g", DEFAULT_BG_G);
  nh_.setParam("background_b", DEFAULT_BG_B);

  QVector<QString> turtles;
  turtles.append("box-turtle.png");
  turtles.append("robot-turtle.png");
  turtles.append("sea-turtle.png");
  turtles.append("diamondback.png");
  turtles.append("electric.png");
  turtles.append("fuerte.png");
  turtles.append("groovy.png");
  turtles.append("hydro.svg");
  turtles.append("indigo.svg");
  turtles.append("jade.png");
  turtles.append("kinetic.png");
  turtles.append("lunar.png");
  turtles.append("melodic.png");

  // 加载所有小乌龟图片，保存在 turtle_images_中
  QString images_path = (ros::package::getPath("turtlesim") + "/images/").c_str();
  for (int i = 0; i < turtles.size(); ++i)
  {
    QImage img;
    img.load(images_path + turtles[i]);
    turtle_images_.append(img);
  }

  meter_ = turtle_images_[0].height();

  clear(); // 清屏，绘制背景图path_image_，以及小乌龟（此时还没有加载小乌龟）

  // 注册这四项服务。当有请求某一服务时，调用相应回调函数处理
  // service/clinet 使用细节可以参阅ros tutorials 中add_two_ints_server/add_two_ints_client用例
  clear_srv_ = nh_.advertiseService("clear", &TurtleFrame::clearCallback, this);
  reset_srv_ = nh_.advertiseService("reset", &TurtleFrame::resetCallback, this);
  spawn_srv_ = nh_.advertiseService("spawn", &TurtleFrame::spawnCallback, this);
  kill_srv_ = nh_.advertiseService("kill", &TurtleFrame::killCallback, this);

  ROS_INFO("Starting turtlesim with node name %s", ros::this_node::getName().c_str()) ;

  // 以小乌龟图片本身高度为基准，在窗口中进行坐标位置映射
  width_in_meters_ = (width() - 1) / meter_;
  height_in_meters_ = (height() - 1) / meter_;
  spawnTurtle("", width_in_meters_ / 2.0, height_in_meters_ / 2.0, 0);// 在中心位置生成第一个小乌龟

  // spawn all available turtle types
  // if里面表示初始时生成所有的小乌龟，把false改为true重新编译后，启动会看到所有的小乌龟
  if(false)
  {
    for(int index = 0; index < turtles.size(); ++index)
    {
      QString name = turtles[index];
      name = name.split(".").first();
      name.replace(QString("-"), QString(""));
      spawnTurtle(name.toStdString(), 1.0 + 1.5 * (index % 7), 1.0 + 1.5 * (index / 7), PI / 2.0, index);
    }
  }
}

TurtleFrame::~TurtleFrame()
{
  delete update_timer_;
}

// 生成一个新的指定名称小乌龟
bool TurtleFrame::spawnCallback(turtlesim::Spawn::Request& req, turtlesim::Spawn::Response& res)
{
  std::string name = spawnTurtle(req.name, req.x, req.y, req.theta);
  if (name.empty())
  {
    ROS_ERROR("A turtled named [%s] already exists", req.name.c_str());
    return false;
  }

  res.name = name;

  return true;
}

// 删除指定名称的小乌龟
bool TurtleFrame::killCallback(turtlesim::Kill::Request& req, turtlesim::Kill::Response&)
{
  M_Turtle::iterator it = turtles_.find(req.name);
  if (it == turtles_.end())
  {
    ROS_ERROR("Tried to kill turtle [%s], which does not exist", req.name.c_str());
    return false;
  }

  turtles_.erase(it);
  update(); // paintEvent

  return true;
}


bool TurtleFrame::hasTurtle(const std::string& name)
{
  return turtles_.find(name) != turtles_.end();
}

std::string TurtleFrame::spawnTurtle(const std::string& name, float x, float y, float angle)
{
  return spawnTurtle(name, x, y, angle, rand() % turtle_images_.size());
}


// 生成小乌龟, 并保存到turtles_字典中
std::string TurtleFrame::spawnTurtle(const std::string& name, float x, float y, float angle, size_t index)
{
  std::string real_name = name;
  if (real_name.empty())
  {
    do
    {
      std::stringstream ss;
      ss << "turtle" << ++id_counter_; // 小乌龟名，turtle1/turtle2...  以此类推
      real_name = ss.str();
    } while (hasTurtle(real_name));
  }
  else
  {
    if (hasTurtle(real_name))
    {
      return "";
    }
  }

  // new Turtle 每一个turtle 对应一个ros node， 图片， 位置，朝向角度
  TurtlePtr t(new Turtle(ros::NodeHandle(real_name), turtle_images_[index], QPointF(x, height_in_meters_ - y), angle));
  turtles_[real_name] = t;
  update(); // paintEvent

  ROS_INFO("Spawning turtle [%s] at x=[%f], y=[%f], theta=[%f]", real_name.c_str(), x, y, angle);

  return real_name;
}

void TurtleFrame::clear()
{
  int r = DEFAULT_BG_R;
  int g = DEFAULT_BG_G;
  int b = DEFAULT_BG_B;

  nh_.param("background_r", r, r);
  nh_.param("background_g", g, g);
  nh_.param("background_b", b, b);

  path_image_.fill(qRgb(r, g, b));
  update(); // 调用paintEvent
}

void TurtleFrame::onUpdate()
{
  ros::spinOnce(); // 调用回调函数一次

  updateTurtles();

  if (!ros::ok())
  {
    close();
  }
}


// 绘制背景图以及小乌龟
void TurtleFrame::paintEvent(QPaintEvent*)
{
  QPainter painter(this);

  painter.drawImage(QPoint(0, 0), path_image_); // 背景图绘制

  M_Turtle::iterator it = turtles_.begin();
  M_Turtle::iterator end = turtles_.end();
  for (; it != end; ++it)  // 字典结构遍历
  {
    // 小乌龟绘制，小乌龟运动轨迹在updateTurtles 中it->second->update (Turtle::update) 调用中执行
    it->second->paint(painter); // map字典结构中每一个键值对 <first,second>，second即表示是turtlePtr
  }
}

void TurtleFrame::updateTurtles()
{
  if (last_turtle_update_.isZero())
  {
    last_turtle_update_ = ros::WallTime::now();
    return;
  }

  bool modified = false;
  M_Turtle::iterator it = turtles_.begin();
  M_Turtle::iterator end = turtles_.end();
  for (; it != end; ++it)
  {
    // map字典结构中每一个键值对 <first,second>，second即表示是turtlePtr
    modified |= it->second->update(0.001 * update_timer_->interval(), path_painter_, path_image_, width_in_meters_, height_in_meters_);
  }
  if (modified)
  {
    update();// QT机制 调用paintEvent
  }

  ++frame_count_;
}


bool TurtleFrame::clearCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  ROS_INFO("Clearing turtlesim.");
  clear();
  return true;
}

bool TurtleFrame::resetCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  ROS_INFO("Resetting turtlesim.");
  turtles_.clear();
  id_counter_ = 0;
  spawnTurtle("", width_in_meters_ / 2.0, height_in_meters_ / 2.0, 0);
  clear();
  return true;
}

}
