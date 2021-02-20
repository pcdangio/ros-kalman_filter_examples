#include <kalman_filter/ukf.hpp>

#include <random>   // Needed for creating simulated measurements in this example.
#include <iostream> // Needed for printing results to console.

// Create extension of ukf_t to incorporate model dynamics.
class model_t
    : public kalman_filter::ukf_t
{
public:
    // Set up with 2 variables and 1 observer.
    model_t()
        : ukf_t(2,1)
    {

    }

    // Stores the current control input.
    double_t u;

private:
    // Implement/override the UKF's state transition model.
    void state_transition(const Eigen::VectorXd& xp, Eigen::VectorXd& x) const override
    {
        x(0) = std::cos(xp(1));
        x(1) = u;
    }
    // Implement/override the UKFs observation model.
    void observation(const Eigen::VectorXd& x, Eigen::VectorXd& z) const override
    {
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

    // Set the initial state and covariance of the model.
    Eigen::VectorXd x_o(2);
    x_o(0) = 0;
    x_o(1) = 0;
    Eigen::MatrixXd P_o(2,2);
    P_o(0,0) = 0.5;
    P_o(1,1) = 0.5;
    ukf.initialize_state(x_o, P_o);

    // Set up random number generator for simulating noisy measurements.
    std::default_random_engine random_generator;
    // Assume zero mean noise with standard deviation indicated by measurement covariance matrix.
    std::normal_distribution<double_t> random_distribution(0, std::sqrt(ukf.R(0,0)));

    // Print out console output column headers:
    std::cout << "x1_true\tx1_est\tx2_true\tx2_est\tu\tz\tP_x1\tP_x2" << std::endl;

    // Use a loop to perform iterations with the filter.
    // Set number of iterations:
    uint32_t N = 50;
    // Set up x_true to compare KF estimations against in example.
    Eigen::VectorXd x_true(2);
    x_true = x_o;
    for(uint32_t n = 0; n < N; ++n)
    {
        // For this example, choose input u = sin(2*pi*n/N).
        ukf.u = std::sin(2.0*M_PI*static_cast<double>(n)/static_cast<double>(N));

        // For sake of example, calculate x_true for comparison.
        x_true(0) = std::cos(x_true(1));
        x_true(1) = ukf.u;

        // For sake of example, create a simulated measurement.
        // Get measurement noise:
        double_t measurement_noise = random_distribution(random_generator);
        // Add measurement noise onto true signal to create simulated measurement.
        double_t z = x_true(1) + measurement_noise;
        
        // Add measurement to the KF.
        ukf.new_observation(0, z);

        // Run the filter predict/update iteration.
        ukf.iterate();

        // Print out this iteration's results:
        const Eigen::VectorXd& estimated_state = ukf.state();
        const Eigen::MatrixXd& estimated_covariance = ukf.covariance();
        std::cout << x_true(0) << "\t" << estimated_state(0) << "\t" << x_true(1) << "\t" << estimated_state(1) << "\t" << ukf.u << "\t" << z << "\t" << estimated_covariance(0,0) << "\t" << estimated_covariance(1,1) << std::endl;
    }
}