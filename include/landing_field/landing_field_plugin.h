#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>

namespace gazebo {

static const std::string kDefaultFrameId = "world";
static const std::string kDefaultLinkName = "base_link";

class LandingFieldPlugin : public ModelPlugin {
 public:
  LandingFieldPlugin()
      : ModelPlugin(),
        link_name_(kDefaultLinkName),
        node_handle_(nullptr),

  virtual ~LandingFieldPlugin();

 protected:

  /// \brief Load the plugin.
  /// \param[in] _model Pointer to the model that loaded this plugin.
  /// \param[in] _sdf SDF element that describes the plugin.
  void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/);

  /// \brief Called when the world is updated.
  /// \param[in] _info Update timing information.
  void OnUpdate(const common::UpdateInfo& /*_info*/);

  /// \brief    Pointer to the update event connection.
  event::ConnectionPtr update_connection_;

  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::LinkPtr link_;

  std::string link_name_;

  /// \brief    Variables for custom wind field generation.
  float min_x_;
  float min_y_;
  int n_x_;
  int n_y_;
  float res_x_;
  float res_y_;
  std::vector<float> vertical_spacing_factors_;
  std::vector<float> bottom_z_;
  std::vector<float> top_z_;
  
  /// \brief  Reads wind data from a text file and saves it.
  /// \param[in] custom_wind_field_path Path to the wind field from ~/.ros.
  void ReadCustomWindField();

  gazebo::transport::NodePtr node_handle_;

};
}