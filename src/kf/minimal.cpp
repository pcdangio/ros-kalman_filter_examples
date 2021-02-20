#include <kalman_filter/kf.hpp>

int32_t main(int32_t argc, char** argv)
{
    // Set up a new KF that has a single state, a single input, and a single observer.
    kalman_filter::kf_t kf(1,1,1);

    // Populate the model matrices accordingly.
    kf.A(0,0) = 1.0;    // State Transition
    kf.B(0,0) = 1.0;    // Control Input
    kf.H(0,0) = 1.0;    // Observation

    // Set up process and observation covariance matrices.
    kf.Q(0,0) = 0.01;   // Process Covariance
    kf.R(0,0) = 1;      // Observation Covariance

    // OPTIONAL: Set the initial state and covariance of the model.
    Eigen::VectorXd x_o(1);
    x_o(0) = 2;
    Eigen::MatrixXd P_o(1,1);
    P_o(0,0) = 0.5;
    kf.initialize_state(x_o, P_o);

    // The following code can be run continuously in a loop:

    // Calculate some new input:
    double_t u = 5.0;
    // Pass input into the filter.
    // The index specifies the position of the input in the control input vector.
    kf.new_input(0, u);

    // Take some measurement as an observation:
    double_t z = 2.0;
    // Pass observation into the filter.
    // The index specifies which observer the observation is for.
    kf.new_observation(0, z);

    // Run the filter predict/update iteration.
    kf.iterate();

    // You may grab the current state and covariance estimates from the filter at any time:
    const Eigen::VectorXd& estimated_state = kf.state();
    const Eigen::MatrixXd& estimated_covariance = kf.covariance();
}