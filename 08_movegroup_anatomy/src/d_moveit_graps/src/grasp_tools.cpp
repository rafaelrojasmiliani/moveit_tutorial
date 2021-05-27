std::make_shared<moveit_grasps::TwoFingerGraspScoreWeights>();

std::shared_ptr<moveit_grasps::TwoFingerGraspScoreWeights>
get_grasp_two_finger_score(double _x, double _y, double _z, double _r,
                           double _p, double _y, double _depth, double _width) {
  std::shared_ptr<moveit_grasps::TwoFingerGraspScoreWeights> result;

  result = std::make_shared<moveit_grasps::TwoFingerGraspScoreWeights>();
  result->orientation_x_score_weight_ = 2.0;
  result->orientation_y_score_weight_ = 2.0;
  result->orientation_z_score_weight_ = 2.0;
  result->translation_x_score_weight_ = 1.0;
  result->translation_y_score_weight_ = 1.0;
  result->translation_z_score_weight_ = 1.0;
  // Finger gripper specific weights.
  result->depth_score_weight_ = 2.0;
  result->width_score_weight_ = 2.0;

  return result;
}

moveit_grasps::TwoFingerGraspCandidateConfig
get_grasp_generator_config(bool _face, bool _x, bool _y, bool _z) {

  moveit_grasps::TwoFingerGraspCandidateConfig result =
      moveit_grasps::TwoFingerGraspCandidateConfig();
  result.disableAll();
  result.enable_face_grasps_ = true;
  result.generate_y_axis_grasps_ = true;
  result.generate_x_axis_grasps_ = true;
  result.generate_z_axis_grasps_ = true;

  return result;
}
