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

#include <QFrame>
#include <QImage>
#include <QPainter>
#include <QPaintEvent>
#include <QTimer>
#include <QVector>

// This prevents a MOC error with versions of boost >= 1.48
#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <ros/ros.h>

# include <std_srvs/Empty.h>
# include <turtlesim/Spawn.h>
# include <turtlesim/Kill.h>
# include <map>

# include "turtle.h"
#endif

namespace turtlesim  // turtlesim域  turtlesim 所用的一切都定义在此域中
{

class TurtleFrame : public QFrame  // TurtleFrame 继承自QFrame
{
  Q_OBJECT  // QT机制 Q_OBJECT表明该类能够共享使用信号-槽 （信号-槽机制简单理解即 出现某信号时，调用槽函数）
public:
  TurtleFrame(QWidget* parent = 0, Qt::WindowFlags f = 0);
  ~TurtleFrame();

  std::string spawnTurtle(const std::string& name, float x, float y, float angle);
  std::string spawnTurtle(const std::string& name, float x, float y, float angle, size_t index);

protected:
  void paintEvent(QPaintEvent* event);   // QT 实例update 自动调用paintEvent

private slots:
  void onUpdate();

private:
  void updateTurtles();
  void clear();
  bool hasTurtle(const std::string& name);

  bool clearCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
  bool resetCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
  bool spawnCallback(turtlesim::Spawn::Request&, turtlesim::Spawn::Response&);
  bool killCallback(turtlesim::Kill::Request&, turtlesim::Kill::Response&);

  ros::NodeHandle nh_;
  QTimer* update_timer_;
  QImage path_image_;
  QPainter path_painter_;

  uint64_t frame_count_;

  ros::WallTime last_turtle_update_;

  // turtleFrame 提供四种服务，其中spawn 用于再创建一个小乌龟
  ros::ServiceServer clear_srv_;
  ros::ServiceServer reset_srv_;
  ros::ServiceServer spawn_srv_;
  ros::ServiceServer kill_srv_;

  typedef std::map<std::string, TurtlePtr> M_Turtle;
  M_Turtle turtles_; // 字典结构 存储turtle  <turtle name: tirtlePtr>
  uint32_t id_counter_;

  QVector<QImage> turtle_images_; // 存放小乌龟图片

  float meter_;
  float width_in_meters_;
  float height_in_meters_;
};

}
