#include <kalman_filter/ukf.hpp>

#include <chrono>       // Needed for timing.
#include <iostream>     // Needed for console ouput.
#include <iomanip>      // Needed for formatting console output.

// Create UKF model.
class model_t
    : public kalman_filter::ukf_t
{
public:
    // Configure n_variables / n_observers below.
    model_t()
        :ukf_t(30,30)
    {

    }

private:
    void state_transition(const Eigen::VectorXd& xp, Eigen::VectorXd& x) const override
    {
        x.setZero();
    }
    void observation(const Eigen::VectorXd& x, Eigen::VectorXd& z) const override
    {
        z.setZero();
    }
};

int32_t main(int32_t argc, char** argv)
{
    // Specify number of runs.
    uint32_t n_runs = 10000;

    // Create UKF object.
    model_t ukf;

    // Start timing.
    auto start_time = std::chrono::high_resolution_clock::now();
    // Start loop.
    for(uint32_t n = 0; n < n_runs; ++n)
    {
        // Add observations.
        for(uint32_t z = 0; z < ukf.n_observers(); ++z)
        {
            ukf.new_observation(z, 10.0);
        }

        // Iterate.
        ukf.iterate();
    }
    // Stop timing.
    auto stop_time = std::chrono::high_resolution_clock::now();

    // Calculate time difference.
    std::chrono::duration<double_t> delta = stop_time - start_time;

    // Calculate average iteration time.
    double_t average_time = delta.count() / static_cast<double_t>(n_runs);

    // Calculate max theoretical rate.
    double_t max_rate = 1.0 / average_time;

    // Print to console.
    std::cout << "UKF with [" << ukf.n_variables() << "] variables and [" << ukf.n_observers() << "] observers at [" << n_runs << "] runs:" << std::endl;
    std::cout << std::fixed << std::setprecision(10) << average_time << " sec average" << std::endl;
    std::cout << max_rate << " Hz max rate" << std::endl;
}