#include <kalman_filter/kf.hpp>

#include <random>   // Needed for creating simulated measurements in this example.
#include <iostream> // Needed for printing results to console.

int32_t main(int32_t argc, char** argv)
{
    // This Kalman Filter will model the following system:
    // x = xp + u
    // Where x is the current state, xp is the prior state, and u is the current input.
    // It has the following observation model:
    // y = x

    // Set up a new KF that has a single state, a single input, and a single observer.
    kalman_filter::kf_t kf(1,1,1);

    // Populate the model matrices accordingly.
    kf.A(0,0) = 1.0;
    kf.B(0,0) = 1.0;
    kf.H(0,0) = 1.0;

    // Set up process and measurement covariance matrices.
    kf.Q(0,0) = 0.01;
    kf.R(0,0) = 1;

    // Set the initial state and covariance of the model.
    Eigen::VectorXd x_o(1);
    x_o(0) = 2;
    Eigen::MatrixXd P_o(1,1);
    P_o(0,0) = 0.5;
    kf.initialize_state(x_o, P_o);

    // Set up random number generator for simulating noisy measurements.
    std::default_random_engine random_generator;
    // Assume zero mean noise with standard deviation indicated by measurement covariance matrix.
    std::normal_distribution<double_t> random_distribution(0, std::sqrt(kf.R(0,0)));

    // Print out console output column headers:
    std::cout << "x_true\tx_est\tu\tz\tP" << std::endl;

    // Use a loop to perform iterations with the filter.
    // Set number of iterations:
    uint32_t N = 50;
    // Set up x_true to compare KF estimations against in example.
    double_t x_true = x_o(0,0);
    for(uint32_t n = 0; n < N; ++n)
    {
        // For this example, choose input u = sin(2*pi*n/N).
        double_t u = std::sin(2.0*M_PI*static_cast<double>(n)/static_cast<double>(N));
        // Set input in KF.
        kf.new_input(0, u);

        // For sake of example, calculate x_true for comparison.
        x_true = x_true + u;

        // For sake of example, create a simulated measurement.
        // Get measurement noise:
        double_t measurement_noise = random_distribution(random_generator);
        // Add measurement noise onto true signal to create simulated measurement.
        double_t z = x_true + measurement_noise;
        
        // Add measurement to the KF.
        kf.new_observation(0, z);

        // Run the filter predict/update iteration.
        kf.iterate();

        // Print out this iteration's results:
        const Eigen::VectorXd& estimated_state = kf.state();
        const Eigen::MatrixXd& estimated_covariance = kf.covariance();
        std::cout << x_true << "\t" << estimated_state(0) << "\t" << u << "\t" << z << "\t" << estimated_covariance(0,0) << std::endl;
    }
}