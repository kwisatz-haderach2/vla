#include "vla_writing/ink_visual_plugin.h"

#include <cmath>
#include <functional>

#include <gazebo/common/Console.hh>
#include <gazebo/rendering/Scene.hh>

namespace vla_writing {

InkVisualPlugin::InkVisualPlugin()
    : current_line_(nullptr),
      paper_width_(0.42),
      paper_height_(0.297),
      write_z_(0.0015),
      contact_threshold_(0.003),
      z_offset_(0.0005),
      min_point_distance_(0.001),
      previous_contact_(false),
      have_last_point_(false) {}

InkVisualPlugin::~InkVisualPlugin() {
  if (pre_render_connection_) {
    pre_render_connection_.reset();
  }
  if (paper_visual_) {
    for (auto* line : lines_) {
      if (line) paper_visual_->DeleteDynamicLine(line);
    }
  }
  lines_.clear();
  current_line_ = nullptr;
}

void InkVisualPlugin::Load(gazebo::rendering::VisualPtr visual,
                           sdf::ElementPtr sdf) {
  paper_visual_ = visual;
  if (sdf) {
    if (sdf->HasElement("paper_width"))
      paper_width_ = sdf->Get<double>("paper_width");
    if (sdf->HasElement("paper_height"))
      paper_height_ = sdf->Get<double>("paper_height");
    if (sdf->HasElement("write_z")) write_z_ = sdf->Get<double>("write_z");
    if (sdf->HasElement("contact_threshold"))
      contact_threshold_ = sdf->Get<double>("contact_threshold");
    if (sdf->HasElement("z_offset")) z_offset_ = sdf->Get<double>("z_offset");
    if (sdf->HasElement("min_point_distance"))
      min_point_distance_ = sdf->Get<double>("min_point_distance");
    if (sdf->HasElement("pen_name_hint"))
      pen_name_hint_ = sdf->Get<std::string>("pen_name_hint");
    if (sdf->HasElement("material"))
      material_name_ = sdf->Get<std::string>("material");
  }
  if (pen_name_hint_.empty()) pen_name_hint_ = "pen_tip_visual";
  if (material_name_.empty()) material_name_ = "Gazebo/Black";

  pre_render_connection_ = gazebo::event::Events::ConnectPreRender(
      std::bind(&InkVisualPlugin::OnPreRender, this));
  gzdbg << "InkVisualPlugin loaded on "
        << (paper_visual_ ? paper_visual_->Name() : "<null>") << "\n";
}

gazebo::rendering::VisualPtr InkVisualPlugin::FindVisual(
    const gazebo::rendering::VisualPtr& root,
    const std::string& name_hint) const {
  if (!root) return gazebo::rendering::VisualPtr();
  if (root->Name().find(name_hint) != std::string::npos) return root;
  const unsigned int n = root->GetChildCount();
  for (unsigned int i = 0; i < n; ++i) {
    gazebo::rendering::VisualPtr found = FindVisual(root->GetChild(i), name_hint);
    if (found) return found;
  }
  return gazebo::rendering::VisualPtr();
}

bool InkVisualPlugin::IsPenOnPaper(const ignition::math::Vector3d& p) const {
  return p.X() >= 0.0 && p.X() <= paper_width_ &&
         p.Y() <= 0.0 && p.Y() >= -paper_height_ &&
         std::abs(p.Z() - write_z_) <= contact_threshold_;
}

void InkVisualPlugin::StartStroke(const ignition::math::Vector3d& p) {
  if (!paper_visual_) return;
  current_line_ = paper_visual_->CreateDynamicLine(gazebo::rendering::RENDERING_LINE_STRIP);
  if (!current_line_) return;
  // DynamicLines inherits Ogre::SimpleRenderable; setMaterial is available in
  // Gazebo 11 and makes the line visible in both the default and camera views.
  current_line_->setMaterial(material_name_);
  lines_.push_back(current_line_);
  have_last_point_ = false;
  AddPoint(p);
}

void InkVisualPlugin::AddPoint(const ignition::math::Vector3d& p) {
  if (!current_line_) return;
  ignition::math::Vector3d ink_point(p.X(), p.Y(), z_offset_);
  if (have_last_point_ && (ink_point - last_point_).Length() < min_point_distance_) return;
  current_line_->AddPoint(ink_point, ignition::math::Color(0.02, 0.02, 0.02, 1.0));
  current_line_->Update();
  last_point_ = ink_point;
  have_last_point_ = true;
}

void InkVisualPlugin::EndStroke() {
  if (current_line_) current_line_->Update();
  current_line_ = nullptr;
  have_last_point_ = false;
}

void InkVisualPlugin::OnPreRender() {
  if (!paper_visual_) return;
  if (!pen_visual_) {
    gazebo::rendering::ScenePtr scene = paper_visual_->GetScene();
    // Gazebo 11 exposes the scene root as WorldVisual() (GetRootVisual()
    // belongs to Visual, not Scene).  Searching from that root also finds
    // pen-tip visuals in models which are spawned after the paper.
    if (scene) {
      const gazebo::rendering::VisualPtr root = scene->WorldVisual();
      pen_visual_ = FindVisual(root, pen_name_hint_);
      // Some URDF/Gazebo combinations drop the optional visual name while
      // flattening the model.  Retain a link-name fallback so ink rendering
      // still follows the actual pen rather than silently staying blank.
      if (!pen_visual_ && pen_name_hint_ != "pen_tip")
        pen_visual_ = FindVisual(root, "pen_tip");
    }
    if (!pen_visual_) return;
  }

  const ignition::math::Pose3d paper_pose = paper_visual_->WorldPose();
  const ignition::math::Pose3d pen_pose = pen_visual_->WorldPose();
  // Pose3 multiplication is defined for poses, not bare vectors in the
  // Ignition Math version shipped with Gazebo 11.  Transform the displacement
  // by the inverse paper rotation explicitly.
  const ignition::math::Vector3d local =
      paper_pose.Rot().Inverse().RotateVector(pen_pose.Pos() - paper_pose.Pos());
  const bool contact = IsPenOnPaper(local);

  if (contact && !previous_contact_) StartStroke(local);
  if (contact && previous_contact_) AddPoint(local);
  if (!contact && previous_contact_) EndStroke();
  previous_contact_ = contact;
}

}  // namespace vla_writing

GZ_REGISTER_VISUAL_PLUGIN(vla_writing::InkVisualPlugin)
