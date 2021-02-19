#include <kalman_filter/ukfa.hpp>

#include <chrono>
#include <iostream>
#include <iomanip>

class model_t
    : public kalman_filter::ukfa_t
{
public:
    model_t()
        :ukfa_t(30,30)
    {

    }

private:
    void state_transition(const Eigen::VectorXd& xp, const Eigen::VectorXd& q, Eigen::VectorXd& x) const override
    {
        x = xp + q;
    }
    void observation(const Eigen::VectorXd& x, const Eigen::VectorXd& r, Eigen::VectorXd& z) const override
    {
        z = x + r;
    }
};

int32_t main(int32_t argc, char** argv)
{
    uint32_t n_runs = 10000;

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

    std::chrono::duration<double_t> delta = stop_time - start_time;

    std::cout << std::fixed << std::setprecision(10) << delta.count() / static_cast<double_t>(n_runs) << std::endl;
}