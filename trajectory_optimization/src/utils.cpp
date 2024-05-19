#include <trajectory_optimization/trajectory_optimization_node.hpp>

#include <blasfeo_d_aux_ext_dep.h>  // for printing dense matrices

namespace trajectory_optimization {

/**
 * @brief Performs linear interpolation to find the corresponding y-value for a given x-value.
 *
 * This function takes two vectors, X and Y, representing the x and y values of a dataset, and a desired x-value.
 * It performs linear interpolation to find the corresponding y-value for the desired x-value.
 *
 * @param X The vector of x-values.
 * @param Y The vector of y-values.
 * @param desired_x The desired x-value.
 * @param output_y The output variable to store the interpolated y-value.
 * @return True if the interpolation is successful, false otherwise.
 */
bool TrajectoryOptimizationNode::linearInterpolation(const std::vector<double>& X, const std::vector<double>& Y,
                                                     const double& desired_x, double& output_y) {
  if (desired_x < *min_element(X.begin(), X.end()) || desired_x > *max_element(X.begin(), X.end())) {
    RCLCPP_ERROR(get_logger(), "Desired Time is not in between of Time-Min and Time-Max of the given vector!");
    RCLCPP_DEBUG(get_logger(), "Desired Time: %f s", desired_x);
    RCLCPP_DEBUG(get_logger(), "Time-Min: %f s", *min_element(X.begin(), X.end()));
    RCLCPP_DEBUG(get_logger(), "Time-Max: %f s", *max_element(X.begin(), X.end()));
    return false;
  }
  if (X.size() != Y.size()) {
    RCLCPP_ERROR(get_logger(), "Input vectors don't have the same length!");
    return false;
  }

  //go through array and search for sampling points
  size_t i;
  for (i = 0; i < X.size(); i++) {
    if (X[i] < desired_x) {
      continue;
    } else if (X[i] == desired_x) {
      output_y = Y[i];
      return true;
    } else {
      break;
    }
  }
  output_y = Y[i - 1] + ((Y[i] - Y[i - 1]) / (X[i] - X[i - 1])) * (desired_x - X[i - 1]);
  return true;
}

/**
 * @brief Converts a trajectory from the vehicle frame to the output frame.
 *
 * This function converts a trajectory from the vehicle frame to the output frame using the tf2 library.
 *
 * @param trajectory The trajectory to be converted.
 */
void TrajectoryOptimizationNode::trajectory2outputFrame(trajectory_planning_msgs::msg::Trajectory& trajectory) {
  if (trajectory_frame_id_ != vehicle_frame_id_) {
    trajectory_planning_msgs::msg::Trajectory tf_trajectory;
    try {
      tf_trajectory = tf2_buffer_->transform(trajectory, trajectory_frame_id_, tf2::durationFromSec(0.01));
    } catch (tf2::TransformException& ex) {
      RCLCPP_WARN(this->get_logger(), "Transformation into output frame is not available. Publishing no trajectory. Ex: %s",
                  ex.what());
      return;
    }
    trajectory = tf_trajectory;
  }
}

/**
 * @brief Prints the solution of the trajectory optimization problem.
 *
 * This function prints the solution of the trajectory optimization problem, including the resulting xtraj and utraj,
 * as well as some solver statistics.
 *
 * @param status The status of the trajectory optimization solver.
 */
void TrajectoryOptimizationNode::printSolution(int status) {
  // get statistics
  double kkt_norm_inf;
  double elapsed_time;
  int sqp_iter;
  ocp_nlp_get(nlp_config_, nlp_solver_, "time_tot", &elapsed_time);
  ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 0, "kkt_norm_inf", &kkt_norm_inf);
  ocp_nlp_get(nlp_config_, nlp_solver_, "sqp_iter", &sqp_iter);

  printf("\n--- xtraj ---\n");
  d_print_exp_tran_mat(TRAJECTORY_PLANNING_NX, n_shots_ + 1, xtraj_, TRAJECTORY_PLANNING_NX);
  printf("\n--- utraj ---\n");
  d_print_exp_tran_mat(TRAJECTORY_PLANNING_NU, n_shots_, utraj_, TRAJECTORY_PLANNING_NU);
  // ocp_nlp_out_print(nlp_solver_->dims, nlp_out_);

  if (status == ACADOS_SUCCESS) {
    printf("\033[1;32mtrajectory_planning_acados_solve(): SUCCESS!\033[0m\n");
  } else {
    printf("\033[1;31mtrajectory_planning_acados_solve() failed with status %d.\033[0m\n", status);
  }

  trajectory_planning_acados_print_stats(acados_ocp_capsule_);

  printf("\nSolver info:\n");
  printf("SQP iterations %2d\n minimum time for %d solve %f [ms]\n KKT %e\n", sqp_iter, 1, elapsed_time * 1000,
         kkt_norm_inf);
}

}  // namespace trajectory_optimization