#include <gazebo/gazebo.hh>
#include <gazebo/plugins/LedPlugin.hh>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <vector>

using namespace std;
using namespace gazebo;

class LedControlSetting : public LedSetting {
  public:
    LedControlSetting(
        const sdf::ElementPtr &sdf,
	const physics::ModelPtr &model,
	const common::Time &currentTime,
	ros::NodeHandle &node): LedSetting(sdf, model, currentTime) {
      std::string topic = "/led/" + this->Link()->GetName() + "/" + this->Name();
      ROS_INFO_STREAM("LED available on " << topic);
      subscriber = node.subscribe(topic, 1000, &LedControlSetting::handle, this);
    }
    ~LedControlSetting() {
      subscriber.shutdown();
    }
  protected:
    void handle(const std_msgs::Bool::ConstPtr &msg) {
      if (msg->data) {
        SwitchOn();
      } else {
        SwitchOff();
      }
    }
  private:
    ros::Subscriber subscriber;
};

class LedControlPlugin : public LedPlugin {
  protected:
    std::shared_ptr<FlashLightSetting> CreateSetting(
        const sdf::ElementPtr &sdf,
        const physics::ModelPtr &model,
        const common::Time &currentTime) {
      return std::make_shared<LedControlSetting>(sdf, model, currentTime, rosNode);
    }

  private:
    ros::NodeHandle rosNode;
};

GZ_REGISTER_MODEL_PLUGIN(LedControlPlugin);

