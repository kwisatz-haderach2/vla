#ifndef VLA_WRITING_INK_VISUAL_PLUGIN_H_
#define VLA_WRITING_INK_VISUAL_PLUGIN_H_

#include <string>
#include <vector>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/rendering/DynamicLines.hh>
#include <gazebo/rendering/Visual.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

namespace vla_writing {

/**
 * Gazebo Classic rendering plugin which draws only the motion actually
 * observed at the pen-tip visual.  It is intentionally independent of ROS:
 * if this plugin is unavailable, MoveIt and the OpenCV monitor still work.
 */
class InkVisualPlugin : public gazebo::VisualPlugin {
 public:
  InkVisualPlugin();
  ~InkVisualPlugin() override;

  void Load(gazebo::rendering::VisualPtr visual,
            sdf::ElementPtr sdf) override;

 private:
  void OnPreRender();
  gazebo::rendering::VisualPtr FindVisual(
      const gazebo::rendering::VisualPtr& root,
      const std::string& name_hint) const;
  bool IsPenOnPaper(const ignition::math::Vector3d& p) const;
  void StartStroke(const ignition::math::Vector3d& p);
  void AddPoint(const ignition::math::Vector3d& p);
  void EndStroke();

  gazebo::rendering::VisualPtr paper_visual_;
  gazebo::rendering::VisualPtr pen_visual_;
  gazebo::rendering::DynamicLines* current_line_;
  std::vector<gazebo::rendering::DynamicLines*> lines_;
  gazebo::event::ConnectionPtr pre_render_connection_;

  std::string pen_name_hint_;
  std::string material_name_;
  double paper_width_;
  double paper_height_;
  double write_z_;
  double contact_threshold_;
  double z_offset_;
  double min_point_distance_;
  bool previous_contact_;
  bool have_last_point_;
  ignition::math::Vector3d last_point_;
};

}  // namespace vla_writing

#endif  // VLA_WRITING_INK_VISUAL_PLUGIN_H_
