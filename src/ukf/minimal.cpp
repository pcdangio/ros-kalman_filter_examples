#include <kalman_filter/ukf.hpp>

// Create extension of ukf_t to incorporate model dynamics.
class model_t
    : public kalman_filter::ukf_t
{
public:
    // Set up with 2 variables and 1 observer.
    // You may choose however many variables and observers you like.
    model_t()
        : ukf_t(2,1)
    {}

    // OPTIONAL: Stores the current control input.
    double_t u;

private:
    // Implement/override the UKF's state transition model.
    void state_transition(const Eigen::VectorXd& xp, Eigen::VectorXd& x) const override
    {
        // Write your state transition model here.

        // For example:
        x(0) = std::cos(xp(1));
        x(1) = u;
    }
    // Implement/override the UKFs observation model.
    void observation(const Eigen::VectorXd& x, Eigen::VectorXd& z) const override
    {
        // Write your observation model here.

        // For example:
        z(0) = x(1);
    }
};


int32_t main(int32_t argc, char** argv)
{
    model_t ukf;

    // Set up process and measurement covariance matrices.
    ukf.Q(0,0) = 0.1;
    ukf.Q(1,1) = 0.1;
    ukf.R(0,0) = 1;

    // OPTIONAL: Set the initial state and covariance of the model.
    Eigen::VectorXd x_o(2);
    x_o(0) = 0;
    x_o(1) = 0;
    Eigen::MatrixXd P_o(2,2);
    P_o(0,0) = 0.5;
    P_o(1,1) = 0.5;
    ukf.initialize_state(x_o, P_o);

    // The following can be run in a loop:

    // Calculate some new control input and store within the model:
    ukf.u = 5.0;

    // Take some measurement as an observation:
    double_t z = 2.0;
    // Pass observation into the filter.
    // The index specifies which observer the observation is for.
    ukf.new_observation(0, z);

    // Run the filter predict/update iteration.
    ukf.iterate();

    // You may grab the current state and covariance estimates from the filter at any time:
    const Eigen::VectorXd& estimated_state = ukf.state();
    const Eigen::MatrixXd& estimated_covariance = ukf.covariance();
}