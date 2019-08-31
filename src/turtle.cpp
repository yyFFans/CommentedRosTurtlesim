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

#include "turtlesim/turtle.h"

#include <QColor>
#include <QRgb>

#define DEFAULT_PEN_R 0xb3
#define DEFAULT_PEN_G 0xb8
#define DEFAULT_PEN_B 0xff

namespace turtlesim
{

Turtle::Turtle(const ros::NodeHandle& nh, const QImage& turtle_image, const QPointF& pos, float orient)
: nh_(nh)
, turtle_image_(turtle_image) // 小乌龟图片
, pos_(pos) // 小乌龟位置
, orient_(orient) // 朝向
, lin_vel_(0.0) // 位姿变化 位移
, ang_vel_(0.0) // 位姿变化 旋转
, pen_on_(true) // 画笔开启，开启则绘制显示运动轨迹，参见使用处
, pen_(QColor(DEFAULT_PEN_R, DEFAULT_PEN_G, DEFAULT_PEN_B)) // 画笔，用于画轨迹线
{
  pen_.setWidth(3);

  // 订阅了cmd_vel话题，该话题在teleop_turtle_key中发布，用于控制小乌龟位姿变化
  // cmd_vel话题发布时，即触发velocityCallback调用，保存位姿变化
  velocity_sub_ = nh_.subscribe("cmd_vel", 1, &Turtle::velocityCallback, this);

  // 创建两个话题，订阅者可以通过这两个话题获取位姿信息，背景色信息
  pose_pub_ = nh_.advertise<Pose>("pose", 1);
  color_pub_ = nh_.advertise<Color>("color_sensor", 1);

  // 画笔服务，客户端可以请求该服务来设置画笔
  set_pen_srv_ = nh_.advertiseService("set_pen", &Turtle::setPenCallback, this);

  // 两种位姿变换（相对/绝对）服务，tutorials中的用例没有用到，可以自己写用例查看效果
  // 回调函数只是先保存请求变换，真正更新处理在update函数中
  teleport_relative_srv_ = nh_.advertiseService("teleport_relative", &Turtle::teleportRelativeCallback, this);
  teleport_absolute_srv_ = nh_.advertiseService("teleport_absolute", &Turtle::teleportAbsoluteCallback, this);

  meter_ = turtle_image_.height();
  rotateImage(); // 根据朝向旋转小乌龟图片
}


void Turtle::velocityCallback(const geometry_msgs::Twist::ConstPtr& vel)
{
  last_command_time_ = ros::WallTime::now();
  lin_vel_ = vel->linear.x;
  ang_vel_ = vel->angular.z;
}

bool Turtle::setPenCallback(turtlesim::SetPen::Request& req, turtlesim::SetPen::Response&)
{
  pen_on_ = !req.off;
  if (req.off)
  {
    return true;
  }

  QPen pen(QColor(req.r, req.g, req.b));
  if (req.width != 0)
  {
    pen.setWidth(req.width);
  }

  pen_ = pen;
  return true;
}

bool Turtle::teleportRelativeCallback(turtlesim::TeleportRelative::Request& req, turtlesim::TeleportRelative::Response&)
{
  teleport_requests_.push_back(TeleportRequest(0, 0, req.angular, req.linear, true));
  return true;
}

bool Turtle::teleportAbsoluteCallback(turtlesim::TeleportAbsolute::Request& req, turtlesim::TeleportAbsolute::Response&)
{
  teleport_requests_.push_back(TeleportRequest(req.x, req.y, req.theta, 0, false));
  return true;
}

void Turtle::rotateImage()
{
  QTransform transform;
  transform.rotate(-orient_ * 180.0 / PI + 90.0);
  turtle_rotated_image_ = turtle_image_.transformed(transform);
}

bool Turtle::update(double dt, QPainter& path_painter, const QImage& path_image, qreal canvas_width, qreal canvas_height)
{
  bool modified = false;
  qreal old_orient = orient_;

  // first process any teleportation requests, in order
  V_TeleportRequest::iterator it = teleport_requests_.begin();
  V_TeleportRequest::iterator end = teleport_requests_.end();
  for (; it != end; ++it)
  {
    const TeleportRequest& req = *it;

    // 根据teleport_requests_请求进行位姿更新
    QPointF old_pos = pos_;
    if (req.relative)
    {
      orient_ += req.theta;
      pos_.rx() += std::cos(orient_) * req.linear;
      pos_.ry() += - std::sin(orient_) * req.linear;
    }
    else
    {
      pos_.setX(req.pos.x());
      pos_.setY(std::max(0.0, static_cast<double>(canvas_height - req.pos.y())));
      orient_ = req.theta;
    }

    if (pen_on_)
    {
      path_painter.setPen(pen_);
      path_painter.drawLine(pos_ * meter_, old_pos * meter_); // 绘制运动轨迹线
    }
    modified = true;
  }

  teleport_requests_.clear();

  // 命令间隔保护，
  if (ros::WallTime::now() - last_command_time_ > ros::WallDuration(1.0))
  {
    lin_vel_ = 0.0;
    ang_vel_ = 0.0;
  }

  QPointF old_pos = pos_;

  orient_ = orient_ + ang_vel_ * dt;
  // Keep orient_ between -pi and +pi
  orient_ -= 2*PI * std::floor((orient_ + PI)/(2*PI));
  pos_.rx() += std::cos(orient_) * lin_vel_ * dt;
  pos_.ry() += - std::sin(orient_) * lin_vel_ * dt;

  // Clamp to screen size
  if (pos_.x() < 0 || pos_.x() > canvas_width ||
      pos_.y() < 0 || pos_.y() > canvas_height)
  {
    ROS_WARN("Oh no! I hit the wall! (Clamping from [x=%f, y=%f])", pos_.x(), pos_.y());
  }

  // static_cast 类型转换
  pos_.setX(std::min(std::max(static_cast<double>(pos_.x()), 0.0), static_cast<double>(canvas_width)));
  pos_.setY(std::min(std::max(static_cast<double>(pos_.y()), 0.0), static_cast<double>(canvas_height)));

  // Publish pose of the turtle
  Pose p;
  p.x = pos_.x();
  p.y = canvas_height - pos_.y();
  p.theta = orient_;
  p.linear_velocity = lin_vel_;
  p.angular_velocity = ang_vel_;
  pose_pub_.publish(p); // 发布新位姿

  // Figure out (and publish) the color underneath the turtle
  {
    Color color;
    QRgb pixel = path_image.pixel((pos_ * meter_).toPoint());
    color.r = qRed(pixel);
    color.g = qGreen(pixel);
    color.b = qBlue(pixel);
    color_pub_.publish(color); // 发布背景色
  }

  ROS_DEBUG("[%s]: pos_x: %f pos_y: %f theta: %f", nh_.getNamespace().c_str(), pos_.x(), pos_.y(), orient_);

  if (orient_ != old_orient)
  {
    rotateImage();
    modified = true;
  }
  if (pos_ != old_pos)
  {
    if (pen_on_)
    {
      path_painter.setPen(pen_);
      path_painter.drawLine(pos_ * meter_, old_pos * meter_);
    }
    modified = true;
  }

  return modified;
}

void Turtle::paint(QPainter& painter)
{
  QPointF p = pos_ * meter_;
  p.rx() -= 0.5 * turtle_rotated_image_.width();
  p.ry() -= 0.5 * turtle_rotated_image_.height();
  painter.drawImage(p, turtle_rotated_image_);
}

}
