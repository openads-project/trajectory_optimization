#include <math.h>

#include <chrono>
#include <functional>
#include <thread>

#include <trajectory_optimization/trajectory_optimization_node.hpp>

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(trajectory_optimization::TrajectoryOptimizationNode)


/**
 * @brief Namespace for trajectory_optimization package
 *
 */
namespace trajectory_optimization {


// parameter names

// constants
const std::string TrajectoryOptimizationNode::kDriveableSpaceTopic = "~/driveable_space";
const std::string TrajectoryOptimizationNode::kEgoDataTopic = "~/ego_data";
const std::string TrajectoryOptimizationNode::kObjectListTopic = "~/object_list";
const std::string TrajectoryOptimizationNode::kRouteTopic = "~/route";

const std::string TrajectoryOptimizationNode::kTrajectoryTopic = "~/trajectory";

const std::string TrajectoryOptimizationNode::kPlanningFreqParam = "planning_frequency";


/**
 * @brief Creates a TrajectoryOptimizationNode node
 *
 */
TrajectoryOptimizationNode::TrajectoryOptimizationNode(const rclcpp::NodeOptions& options) : Node("trajectory_optimization_node", options) {

  this->declareParameters();
  this->loadParameters();
  this->setup();
}

/**
 * @brief Destroys a TrajectoryOptimizationNode node
 *
 */
TrajectoryOptimizationNode::~TrajectoryOptimizationNode() {
    // deallocate memory
    delete[] xtraj_;
    delete[] utraj_;
    
    int status;
    // free solver
    status = trajectory_planning_acados_free(acados_ocp_capsule_);
    if (status) {
        printf("trajectory_planning_acados_free() returned status %d. \n", status);
    }
    // free solver capsule
    status = trajectory_planning_acados_free_capsule(acados_ocp_capsule_);
    if (status) {
        printf("trajectory_planning_acados_free_capsule() returned status %d. \n", status);
    }
    RCLCPP_INFO(this->get_logger(), "TrajectoryOptimizationNode destroyed");
}

/**
 * @brief Declares all parameters that this node uses
 */
void TrajectoryOptimizationNode::declareParameters() {

  // set parameter description
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.description = "Planning Frequency in Hz";

  // declare parameter
  this->declare_parameter(kPlanningFreqParam, rclcpp::ParameterType::PARAMETER_DOUBLE, param_desc);
}

/**
 * @brief Loads ROS parameters used in the node.
 *
 */
void TrajectoryOptimizationNode::loadParameters() {

  // load parameter
  try {
    planning_freq_ = this->get_parameter(kPlanningFreqParam).as_double();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_FATAL(this->get_logger(), "Parameter '%s' is required", kPlanningFreqParam.c_str());
    exit(EXIT_FAILURE);
  }
}


/**
 * @brief Sets up subscribers, publishers, and more.
 *
 */
void TrajectoryOptimizationNode::setup() {

  // create a callback for dynamic parameter configuration
  parameters_callback_ = this->add_on_set_parameters_callback(
    std::bind(&TrajectoryOptimizationNode::parametersCallback, this, std::placeholders::_1));

  // set up subscriber for input topics
  ego_data_sub_ =
    this->create_subscription<perception_msgs::msg::EgoData>(
      kEgoDataTopic, 10,
      std::bind(&TrajectoryOptimizationNode::egoDataCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", ego_data_sub_->get_topic_name());

  driveable_space_sub_ =
    this->create_subscription<route_planning_msgs::msg::DriveableSpace>(
      kDriveableSpaceTopic, 10,
      std::bind(&TrajectoryOptimizationNode::driveableSpaceCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", driveable_space_sub_->get_topic_name());

  object_list_sub_ =
    this->create_subscription<perception_msgs::msg::ObjectList>(
      kObjectListTopic, 10,
      std::bind(&TrajectoryOptimizationNode::objectListCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", object_list_sub_->get_topic_name());

  route_sub_ =
    this->create_subscription<route_planning_msgs::msg::Route>(
      kRouteTopic, 10,
      std::bind(&TrajectoryOptimizationNode::routeCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", route_sub_->get_topic_name());

  // set up publisher for output topic
  trajectory_pub_ = this->create_publisher<trajectory_planning_msgs::msg::Trajectory>(
    kTrajectoryTopic, 10);
  RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", trajectory_pub_->get_topic_name());

  // create timer for planning cycle
  planning_timer_ =
    this->create_wall_timer(std::chrono::duration<double>(planning_freq_),
                            std::bind(&TrajectoryOptimizationNode::planningCycle,
                            this));

  setupSolver();
}

void TrajectoryOptimizationNode::setupSolver() {
  // setup acados solver
  acados_ocp_capsule_ = trajectory_planning_acados_create_capsule();

  // there is an opportunity to change the number of shooting intervals in C without new code generation
  N_ = TRAJECTORY_PLANNING_N; // TODO: param
  // allocate the array and fill it accordingly
  double* new_time_steps = NULL; // TODO: calculate from N_
  int status = trajectory_planning_acados_create_with_discretization(acados_ocp_capsule_, N_, new_time_steps);

  if (status) {
    RCLCPP_INFO(this->get_logger(), "trajectory_planning_acados_create() returned status %d. Exiting.", status);
    exit(1);
  }

  nlp_config_ = trajectory_planning_acados_get_nlp_config(acados_ocp_capsule_);
  nlp_dims_ = trajectory_planning_acados_get_nlp_dims(acados_ocp_capsule_);
  nlp_in_ = trajectory_planning_acados_get_nlp_in(acados_ocp_capsule_);
  nlp_out_ = trajectory_planning_acados_get_nlp_out(acados_ocp_capsule_);
  nlp_solver_ = trajectory_planning_acados_get_nlp_solver(acados_ocp_capsule_);
  nlp_opts_ = trajectory_planning_acados_get_nlp_opts(acados_ocp_capsule_);

  // initial condition
  double lbx0[TRAJECTORY_PLANNING_NBX0];
  double ubx0[TRAJECTORY_PLANNING_NBX0];
  lbx0[0] = 0;
  ubx0[0] = 0;
  lbx0[1] = 3.141592653589793;
  ubx0[1] = 3.141592653589793;
  lbx0[2] = 0;
  ubx0[2] = 0;
  lbx0[3] = 0;
  ubx0[3] = 0;

  ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, 0, "lbx", lbx0);
  ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, 0, "ubx", ubx0);

  // initialization for state values
  double x_init[TRAJECTORY_PLANNING_NX];
  x_init[0] = 0.0;
  x_init[1] = 0.0;
  x_init[2] = 0.0;
  x_init[3] = 0.0;

  // initial value for control input
  double u0[TRAJECTORY_PLANNING_NU];
  u0[0] = 0.0;

  // initialize solution
  int rti_phase = 0;
  for (int i = 0; i < N_; i++) {
    ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, i, "x", x_init);
    ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, i, "u", u0);
  }
  ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, N_, "x", x_init);
  ocp_nlp_solver_opts_set(nlp_config_, nlp_opts_, "rti_phase", &rti_phase);

  xtraj_ = new double[TRAJECTORY_PLANNING_NX * (N_+1)];
  utraj_ = new double[TRAJECTORY_PLANNING_NU * N_];

}


/**
 * @brief This callback is invoked when a parameter value has changed
 *
 * @param[in] parameters                                  input
 *
 * @return    rcl_interfaces::msg::SetParametersResult    output
 */
rcl_interfaces::msg::SetParametersResult TrajectoryOptimizationNode::parametersCallback(
  const std::vector<rclcpp::Parameter> &parameters) {

  // update timer with newly configured period parameter value
  rcl_interfaces::msg::SetParametersResult result;
  for (const auto &param : parameters) {
    if (param.get_name() == kPlanningFreqParam) {
      planning_freq_ = param.as_double();
    }
  }

  // mark parameter change successful
  result.successful = true;
  result.reason = "success";

  return result;
}


/**
 * @brief This callback is invoked when the subscriber receives a message
 *
 * @param[in] msg   input ego data
 */
void TrajectoryOptimizationNode::egoDataCallback(
  const perception_msgs::msg::EgoData::ConstSharedPtr msg) {

  RCLCPP_INFO(this->get_logger(), "Received ego data");
  // TODO: process ego data
}

/**
 * @brief This callback is invoked when the subscriber receives a message
 *
 * @param[in] msg   input drivable space
 */
void TrajectoryOptimizationNode::driveableSpaceCallback(
  const route_planning_msgs::msg::DriveableSpace::ConstSharedPtr msg) {

  RCLCPP_INFO(this->get_logger(), "Received driveable space");
  // TODO: process driveable space
}

/**
 * @brief This callback is invoked when the subscriber receives a message
 *
 * @param[in] msg   input object list
 */
void TrajectoryOptimizationNode::objectListCallback(
  const perception_msgs::msg::ObjectList::ConstSharedPtr msg) {

  RCLCPP_INFO(this->get_logger(), "Received object list");
  // TODO: process object list
}

/**
 * @brief This callback is invoked when the subscriber receives a message
 *
 * @param[in] msg   input route
 */
void TrajectoryOptimizationNode::routeCallback(
  const route_planning_msgs::msg::Route::ConstSharedPtr msg) {

  RCLCPP_INFO(this->get_logger(), "Received route");
  // TODO: process route
}

/**
 * @brief This function prints the solution of the ocp
 * 
 */
void TrajectoryOptimizationNode::printSolution(int status, double elapsed_time, int sqp_iter, double kkt_norm_inf) {
  printf("\n--- xtraj ---\n");
  d_print_exp_tran_mat( TRAJECTORY_PLANNING_NX, N_+1, xtraj_, TRAJECTORY_PLANNING_NX);
  printf("\n--- utraj ---\n");
  d_print_exp_tran_mat( TRAJECTORY_PLANNING_NU, N_, utraj_, TRAJECTORY_PLANNING_NU );
  // ocp_nlp_out_print(nlp_solver->dims, nlp_out);

  printf("\nsolved ocp %d times, solution printed above\n\n", 1);

  if (status == ACADOS_SUCCESS)
  {
      printf("trajectory_planning_acados_solve(): SUCCESS!\n");
  }
  else
  {
      printf("trajectory_planning_acados_solve() failed with status %d.\n", status);
  }

  trajectory_planning_acados_print_stats(acados_ocp_capsule_);

  printf("\nSolver info:\n");
  printf(" SQP iterations %2d\n minimum time for %d solve %f [ms]\n KKT %e\n",
          sqp_iter, 1, elapsed_time*1000, kkt_norm_inf);

}

/**
 * @brief This function is invoked every period seconds by the timer
 *
 */
void TrajectoryOptimizationNode::planningCycle() {
  
  trajectory_planning_msgs::msg::Trajectory::UniquePtr trajectory = std::make_unique<trajectory_planning_msgs::msg::Trajectory>();
  trajectory_planning_msgs::trajectory_access::initializeTrajectory(*trajectory, trajectory_planning_msgs::msg::DRIVABLE::TYPE_ID, TRAJECTORY_PLANNING_N);  

  trajectory_pub_->publish(std::move(trajectory));
  RCLCPP_INFO(this->get_logger(), "Published trajectory");

  ////////////////////////////////////////////////////////
  // sample ocp step
  double kkt_norm_inf;
  double elapsed_time;
  int sqp_iter;

  int status = trajectory_planning_acados_solve(acados_ocp_capsule_);

  // get elapsed time
  ocp_nlp_get(nlp_config_, nlp_solver_, "time_tot", &elapsed_time);

  // get solution
  for (int ii = 0; ii <= nlp_dims_->N; ii++)
      ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, ii, "x", &xtraj_[ii*TRAJECTORY_PLANNING_NX]);
  for (int ii = 0; ii < nlp_dims_->N; ii++)
      ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, ii, "u", &utraj_[ii*TRAJECTORY_PLANNING_NU]);

  ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 0, "kkt_norm_inf", &kkt_norm_inf);
  ocp_nlp_get(nlp_config_, nlp_solver_, "sqp_iter", &sqp_iter);

  // print solution
  printSolution(status, elapsed_time, sqp_iter, kkt_norm_inf);

  // update condition
  double lbx0[TRAJECTORY_PLANNING_NBX0];
  double ubx0[TRAJECTORY_PLANNING_NBX0];
  // fill condition with the last state of the solution
  for (int i = 0; i < TRAJECTORY_PLANNING_NBX0; i++) {
    lbx0[i] = xtraj_[TRAJECTORY_PLANNING_NX * (N_+1) - TRAJECTORY_PLANNING_NBX0 + i];
    ubx0[i] = xtraj_[TRAJECTORY_PLANNING_NX * (N_+1) - TRAJECTORY_PLANNING_NBX0 + i];
  }

  ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, 0, "lbx", lbx0);
  ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, 0, "ubx", ubx0);

}


}