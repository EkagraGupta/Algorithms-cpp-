#include<iostream>
#include<Eigen/Eigen>

Eigen::VectorXf KalmanGain(10);
Eigen::VectorXf ProcessNoise(10);
Eigen::VectorXf EstimateUncertainity(10);
Eigen::VectorXf MeasurementNoise(10);
Eigen::VectorXf xhat(10);       // Predicted value vector

// True Value vector
Eigen::VectorXf x{
    {49.979, 50.025, 50, 50.003, 49.994, 50.002, 49.999, 50.006, 49.998, 49.991}
};
// Measured value vector
Eigen::VectorXf z{
    {49.95, 49.967, 50.1, 50.106, 49.992, 49.819, 49.933, 50.007, 50.023, 49.99}
};

/**
 * @brief Performs Kalman filtering for N_samples
 * 
 * @param K Kalman Gain
 * @param q Process Noise
 * @param p Estimate Uncertainity
 * @param r Measurement Noise
 * @param z Measurement vector
 * @param xhat Estimate Vector
 * @return auto 
 */
auto KalmanFilter(Eigen::VectorXf K, Eigen::VectorXf q, Eigen::VectorXf p, Eigen::VectorXf r, Eigen::VectorXf z, Eigen::VectorXf xhat)
{
    for (int n=1; n<10; n++)
        {
            K(n-1) = p(n-1)/(p(n-1) + r(n-1));
            r(n) = r(n-1);

            xhat(n) = xhat(n-1) + (K(n-1) * (z(n-1) - xhat(n-1)));

            p(n) = (1 - K(n-1)) * p(n-1);

            p(n) = p(n) + q(n-1);
        }
    return xhat;
        
}

auto absoluteError(Eigen::VectorXf TrueValue, Eigen::VectorXf EstimatedValue)
{
    return TrueValue - EstimatedValue;
}

int main()
{
    EstimateUncertainity(0) = 100.0*100.0;
    ProcessNoise(0) = 0.01*0.01;
    MeasurementNoise(0) = 0.1*0.1;
    xhat(0) = 10;

    xhat = KalmanFilter(KalmanGain, ProcessNoise, EstimateUncertainity, MeasurementNoise, z, xhat);

    std::cout<<xhat<<std::endl;
    std::cout<<absoluteError(x, xhat)<<std::endl;
    std::cin.get();
}