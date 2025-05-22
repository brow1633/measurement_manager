// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Ethan Brown
#pragma once

#include <functional>
#include <memory>
#include <map>
#include <chrono>
#include <limits>
#include <type_traits>
#include <cmath>
#include <stdexcept>

namespace measurement_manager {

// Must declare with a std::chrono::duration type
template<typename DurationT, typename StateT>
class MeasurementManager;

// only this specialization is defined:
template<typename Rep, typename Period, typename StateT>
class MeasurementManager<std::chrono::duration<Rep,Period>, StateT> {
public:
    static_assert(std::is_default_constructible_v<StateT>, "State type must be default constructible");
    static_assert(std::is_copy_assignable_v<StateT>, "State type must be copyable");

    using DurationT = std::chrono::duration<Rep,Period>;

    using ResetFn = std::function<void(const StateT& /* new_state */)>;
    using PredictFn = std::function<StateT(DurationT /* dt */)>;
    using ApplyFn = std::function<bool(const StateT& /* current state */)>;

    using StateBuffer = std::map<DurationT, StateT>;
    using MeasurementBuffer = std::multimap<DurationT, ApplyFn>;

    /**
     * @brief Constructor for measurement manager.
     * @param init_state Initial state
     * @param init_time Initial time
     * @param reset Function to reset a filter to a given state
     * @param predict Function to predict a specified time delta.
     * @param max_buffer_length The maximum length to store states and measurements for.
     * 
     * @throws on invalid reset, predict functions or negative max_buffer_length.
     */
    MeasurementManager(const StateT& init_state, DurationT init_time, ResetFn reset, PredictFn predict, DurationT max_buffer_length) 
      : init_time_(init_time),
        reset_(reset),
        predict_(predict),
        max_buffer_length_(max_buffer_length),
        min_timestamp_(DurationT::max()),
        max_timestamp_(DurationT::min())
    {
        // Validate callback functions
        if (!reset_ || !predict_) {
            throw std::invalid_argument("Reset and predict callbacks must be valid functions");
        }
        
        // Validate buffer length
        if (max_buffer_length <= DurationT::zero()) {
            throw std::invalid_argument("Buffer length must be positive");
        }

        state_buffer_.emplace(init_time, init_state);
        reset_(init_state);
        current_filter_time_ = init_time;
    }

    /**
     * @brief Function to add a measurement to the measurement queue.
     * @param timestamp The timestamp of this measurement.
     * @param apply The callback to apply the measurement.
     * @return Whether the measurement was added. (If it is older than max_buffer_length it may not be added)
     */
    bool add_measurement(DurationT timestamp, ApplyFn apply) {
        // Don't add measurements without an apply function
        if (!apply) {
            return false; 
        }

        if (timestamp < init_time_) {
            return false;
        }

        // Don't add measurements past our desired buffer length
        if (max_timestamp_ > (DurationT::min() + max_buffer_length_) &&
            timestamp < (max_timestamp_ - max_buffer_length_)) {

            return false;
        }

        // If this timestamp is older than the oldest new measurement, update min_timestamp_
        if (timestamp < min_timestamp_) {
            min_timestamp_ = timestamp;
        }

        measurement_buffer_.emplace(timestamp, apply);
        return true;
    }

    /**
     * @brief Update the filter to a given time, incorporating all measurements in queue up to given time.
     * @param time The time to update the filter to.
     * @return Fully updated state of filter.
     * 
     * This function determines the measurement start time, which is the oldest measurement not 
     * yet integrated into the filter.  It then finds the newest state that is not newer than the oldest measurement.
     * 
     * It will reset the filter to the determined state, and remove all future states after this one from the state bufffer.
     * These will no longer be valid.
     * 
     * Then, we run the standard predict & update cycle for the filter up to the requested time.
     */
    StateT update_to_time(DurationT time) {
        StateT init_state;

        // This is an iterator pointing to the first measurement that we need to incorporate.
        auto meas_start = measurement_buffer_.begin();
        // This is the time we will start the predict/update from (i.e. the time of the starting state)
        DurationT start_time;

        if (state_buffer_.size() == 0) {
            // State is empty (first call)
            meas_start = measurement_buffer_.begin();
            start_time = meas_start->first;
        } else {
            // State buffer is stored in ascending order
            // This will return the first timestamp that isn't strictly less than min_timestamp_
            auto state_it = state_buffer_.lower_bound(min_timestamp_); 

            // If it's not exactly equal, we need to decrement
            if (state_it->first != min_timestamp_) {
                --state_it;
            }

            start_time = state_it->first;
            init_state = state_it->second;

            // Now we need to find the first measurement to be incorporated
            // lower bound naturally finds this
            meas_start = measurement_buffer_.lower_bound(start_time);

            // We need to erase all previously calculated states after this time
            auto next_state = std::next(state_it);
            if (next_state != state_buffer_.end()) {
                state_buffer_.erase(next_state, state_buffer_.end());
            }
            if (start_time != current_filter_time_) {
                reset_(init_state);
                current_filter_time_ = start_time;
            }
        }


        auto meas_it = meas_start;
        DurationT filter_time = start_time;

        // Loop through all measurements up to last measurement before our requested time
        for (; meas_it != measurement_buffer_.end(); ++meas_it) {
            if (meas_it->first > time) break; // We'd be passing our requested time

            // Prediction step
            DurationT dt = meas_it->first - filter_time;
            StateT state = predict_(dt);

            // Update step
            meas_it->second(state);

            // Update filter time and append calculated state
            filter_time = meas_it->first;
            state_buffer_.emplace(filter_time, state);
        }

        // Two cases:
        // 1. The previous loop perfectly brought us up to the requested time
        // 2. We are now some amount of time behind the request
        // For 1) We expect predict_() will give us the fully corrected state at current time (which previous loop does not recover)
        // For 2) We expect predict_() to update by dt and provide fully corrected state
        // We then put this into our state buffer
        DurationT dt = time - filter_time;
        StateT state = predict_(dt);
        state_buffer_.emplace(time, state);

        // The oldest measurement placed into our filter is, at oldest, the requested timestamp
        min_timestamp_ = time;

        // Maintain max_timestamp_ for buffer pruning
        if (time > max_timestamp_) max_timestamp_ = time;
        current_filter_time_ = time;

        // Prune old timestamps
        prune_buffers(time);
        return state;
    }

    /**
     * @brief Get current size of state buffer.
     * @return Current size.
     */
    size_t get_state_buffer_size() const { return state_buffer_.size(); }

    /**
     * @brief Get current size of measurement buffer.
     * @return Current size.
     */
    size_t get_measurement_buffer_size() const { return measurement_buffer_.size(); }

    /**
     * @brief Get current filter time.
     */
    DurationT get_current_filter_time() const { return current_filter_time_; }

protected:
    /**
     * @brief Prune an arbitrary buffer that is keyed with timestamps (i.e. a std::chrono::DurationT)
     * @param curr_time The current time reference.
     * @param buffer The buffer to prune
     */
    template<typename T> 
    void prune_buffer(DurationT curr_time, T& buffer) {
        static_assert(
            std::is_same_v<typename T::key_type, DurationT>,
            "Buffer::key_type must be DurationT"
        );
        auto it = buffer.lower_bound(curr_time-max_buffer_length_);

        if(it != buffer.begin()) {
            buffer.erase(buffer.begin(), it);
        }
    }

    /**
     * @brief Prune old measurements or states out of filter based on max_buffer_length
     * @param curr_time The current time reference.
     */
    void prune_buffers(DurationT curr_time) {
        prune_buffer(curr_time, measurement_buffer_);
        prune_buffer(curr_time, state_buffer_);
    }

    /**
     * @brief Buffer storing measurements keyed by timestamp
     */
    MeasurementBuffer measurement_buffer_;

    /**
     * @brief Buffer storing states keyed by timestamp
     */
    StateBuffer state_buffer_;

    /**
     * @brief Initial time passed into manager
     */
    DurationT init_time_;

    /**
     * @brief Function to reset the filter to a given state
     */
    ResetFn reset_;

    /**
     * @brief Function to predict the filter by dt.
     */
    PredictFn predict_;

    /**
     * @brief The maximum buffer length in seconds
     */
    DurationT max_buffer_length_;

    /**
     * @brief The oldest measurement timestamp that has not yet been incorporated into the filter.
     */
    DurationT min_timestamp_;

    /**
     * @brief The newest measurement timestamp that has been incorporated into the filter.
     */
    DurationT max_timestamp_;

    /**
     * @brief The last state reset to.
     */
    DurationT current_filter_time_;
}; // class MeasurementManager

} // namespace measurement_manager