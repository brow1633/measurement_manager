#include "measurement_manager.hpp"
#include <iostream>
#include <chrono>

// Simple 1D position/velocity/acceleration state
struct KalmanState {
    double position = 0.0;
    double velocity = 0.0;
    double acceleration = 0.0;
    // ... covariance matrices, etc.
};

int main() {
    using namespace measurement_manager;
    using Duration = std::chrono::milliseconds;
    
    KalmanState current_state;
    
    // Reset callback - restore filter to a previous state
    auto reset = [&](const KalmanState& state) {
        current_state = state;
        // Also reset your covariance matrices, etc.
    };
    
    // Predict callback - advance filter by dt
    auto predict = [&](Duration dt) -> KalmanState {
        double dt_sec = dt.count() / 1000.0;
        
        current_state.position += current_state.velocity * dt_sec + 0.5 * current_state.acceleration * dt_sec;
        current_state.velocity += current_state.acceleration * dt_sec;
        // Update covariance matrices...
        
        return current_state;
    };
    
    // Create manager with 5-second buffer
    MeasurementManager<Duration, KalmanState> filter_mgr(current_state, Duration(0),
        reset, predict, Duration(5000)
    );
    
    // Add IMU measurement at t=1000ms
    filter_mgr.add_measurement(Duration(1000), [&](const KalmanState& state) {
        double imu_acceleration = 5.0;
        // Apply IMU 
        current_state.acceleration = (state.acceleration + imu_acceleration) / 2.0; // Simplified
        std::cout << "IMU Measurement Applied!" << std::endl;
        return true;
    });

    filter_mgr.update_to_time(Duration(1000));

    // Add GPS measurement at t=800ms (Delayed!)
    filter_mgr.add_measurement(Duration(800), [&](const KalmanState& state) {
        // Apply GPS position measurement
        double gps_position = 10.5; // From GPS sensor
        // ... Kalman update equations ...
        current_state.position = (state.position + gps_position) / 2.0; // Simplified
        std::cout << "GPS Measurement Applied!" << std::endl;
        return true;
    });
    
    filter_mgr.update_to_time(Duration(2000));
    
    std::cout << "Final position: " << current_state.position << std::endl;
    std::cout << "Final velocity: " << current_state.velocity << std::endl;
    std::cout << "Final acceleration: " << current_state.acceleration << std::endl;
    
    return 0;
}