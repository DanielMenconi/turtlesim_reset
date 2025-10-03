#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>
#include <rviz_panel_tutorial/demo_panel.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <QTimer>


namespace turtlesim_reset
{
DemoPanel::DemoPanel(QWidget * parent) : Panel(parent)
{
  // Create a label and a button, displayed vertically (the V in VBox means vertical)
  const auto layout = new QVBoxLayout(this);
  label_ = new QLabel("[no data]");
  button_ = new QPushButton("GO!");
  layout->addWidget(label_);
  layout->addWidget(button_);
  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, &DemoPanel::moveTowardsTarget);


  // Connect the event of when the button is released to our callback,
  // so pressing the button results in the callback being called.
  QObject::connect(button_, &QPushButton::released, this, &DemoPanel::buttonActivated);
}

DemoPanel::~DemoPanel() = default;

void DemoPanel::onInitialize()
{
  // Access the abstract ROS Node and
  // in the process lock it for exclusive use until the method is done.
  node_ptr_ = getDisplayContext()->getRosNodeAbstraction().lock();

  // Get a pointer to the familiar rclcpp::Node for making subscriptions/publishers
  // (as per normal rclcpp code)
  rclcpp::Node::SharedPtr node = node_ptr_->get_raw_node();
  pose_sub_ = node->create_subscription<turtlesim::msg::Pose>(
        "/turtle1/pose", 10,
        std::bind(&DemoPanel::poseCallback, this, std::placeholders::_1));
  cmd_vel_pub_ = node->create_publisher<geometry_msgs::msg::Twist>(
    "/turtle1/cmd_vel", 10);
      
}
void DemoPanel::moveTowardsTarget()
{
    auto msg = geometry_msgs::msg::Twist();

    double dx = target_x_ - current_x_; // current_x_ aggiornato in poseCallback
    double dy = target_y_ - current_y_;
    double distance = std::sqrt(dx*dx + dy*dy);

    if (distance < 0.1) {
        timer_->stop();
        cmd_vel_pub_->publish(msg); // Twist zero = fermo
        label_->setText("Target reached!");
        return;
    }

    double angle_to_goal = std::atan2(dy, dx);
    double angle_error = angle_to_goal - current_theta_;

    msg.linear.x = 1.0;
    msg.angular.z = 2.0 * angle_error;

    cmd_vel_pub_->publish(msg);
}


// When the subscriber gets a message, this callback is triggered,
// and then we copy its data into the widget's label

void DemoPanel::poseCallback(const turtlesim::msg::Pose::SharedPtr msg)
{
    double x = msg->x;
    double y = msg->y;
    double theta = msg->theta;

    current_x_ = x;
    current_y_ = y;
    current_theta_ = theta;

    QString s = QString("x: %1, y: %2, θ: %3").arg(x, 0, 'f', 2).arg(y, 0, 'f', 2).arg(theta, 0, 'f', 2);

    QMetaObject::invokeMethod(label_, "setText", Qt::QueuedConnection,
                              Q_ARG(QString, s));
}



// When the widget's button is pressed, this callback is triggered,
// and then we publish a new message on our topic.
void DemoPanel::buttonActivated()
{
 
   timer_->start(100); // chiama moveTowardsTarget() ogni 100 ms
    label_->setText("Moving turtle...");

}

}  // namespace rviz_panel_tutorial

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(turtlesim_reset::DemoPanel, rviz_common::Panel)
