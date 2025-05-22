#include <gtest/gtest.h>
#include <chrono>
#include <sstream>
#include <vector>
#include <iomanip>
#include "measurement_manager.hpp"

using namespace measurement_manager;

struct MockState {
    double t = 0.0;
    MockState() = default;
    MockState(double t) : t(t) {};
    MockState& operator=(const MockState&) = default;
};

template<typename DurationT>
class MeasurementManagerTestTemplate : public ::testing::Test {
protected:
    using ManagerType = MeasurementManager<DurationT, MockState>;
    using ResetFn = typename ManagerType::ResetFn;
    using PredictFn = typename ManagerType::PredictFn;
    using ApplyFn = typename ManagerType::ApplyFn;

    std::vector<std::string> applied;
    MockState state;
    std::unique_ptr<ManagerType> manager;

    // Helper to create duration from double seconds
    DurationT seconds(double s) {
        return std::chrono::duration_cast<DurationT>(std::chrono::duration<double>(s));
    }

    // We fill reset and predict functions with instrumentation to track what's occuring
    void SetUp() override {
        applied.clear();
        state = MockState();

        auto reset_fn = [this](const MockState& s) {
            std::ostringstream ss;
            ss << "reset" << std::fixed << std::setprecision(1) << s.t;
            applied.push_back(ss.str());
            state = s;
        };
        
        auto predict_fn = [this](DurationT dt) -> MockState {
            std::ostringstream ss;
            double dt_seconds = std::chrono::duration<double>(dt).count();
            ss << "p" << std::fixed << std::setprecision(1) << dt_seconds;
            applied.push_back(ss.str());
            state.t += dt_seconds;
            return state;
        };

        manager = std::make_unique<ManagerType>(
            state,
            seconds(0),
            reset_fn,
            predict_fn,
            seconds(10)
        );
        applied.clear();
    }

    // Helper to create apply functions
    ApplyFn CreateApplyFunction(const std::string& name) {
        return [this, name](const MockState& s) {
            applied.push_back(name);
            return true;
        };
    }

    void VerifySequence(const std::vector<std::string>& expected) {
        ASSERT_EQ(expected.size(), applied.size());
        for (size_t i = 0; i < expected.size(); i++) {
            EXPECT_EQ(expected[i], applied[i]) << "Mismatch at position " << i;
        }
    }

    void PrintApplied() {
        for (const auto& item : applied) {
            std::cout << item << std::endl;
        }
    }

    void VerifyApplied(const std::string& exp) {
        EXPECT_NE(std::find(applied.begin(), applied.end(), exp), applied.end());
    }

    void VerifyNotApplied(const std::string& exp) {
        EXPECT_EQ(std::find(applied.begin(), applied.end(), exp), applied.end());
    }
    
};

// Test with different duration types
using DurationTypes = ::testing::Types<
    std::chrono::milliseconds,
    std::chrono::microseconds,
    std::chrono::duration<double>,
    std::chrono::duration<float>
>;

TYPED_TEST_SUITE(MeasurementManagerTestTemplate, DurationTypes);

// Constructor tests
TYPED_TEST(MeasurementManagerTestTemplate, ConstructorValidation) {
    using ManagerType = typename TestFixture::ManagerType;
    using DurationT = TypeParam;
    // Test invalid reset function
    EXPECT_THROW({
        ManagerType mgr(this->state, this->seconds(0), nullptr, 
            [this](DurationT dt) -> MockState { return this->state; }, 
            this->seconds(10.0));
    }, std::invalid_argument);
    
    // Test invalid predict function
    EXPECT_THROW({
        ManagerType mgr(this->state, this->seconds(0),
            [this](const MockState& s) { this->state = s; }, 
            nullptr, 
            this->seconds(10.0));
    }, std::invalid_argument);
    
    // Test invalid buffer length
    EXPECT_THROW({
        ManagerType mgr(this->state, this->seconds(0),
            [this](const MockState& s) { this->state = s; }, 
            [this](DurationT dt) -> MockState { return this->state; }, 
            this->seconds(-1.0));
    }, std::invalid_argument);
    
    EXPECT_THROW({
        ManagerType mgr(this->state, this->seconds(0),
            [this](const MockState& s) { this->state = s; }, 
            [this](DurationT dt) -> MockState { return this->state; }, 
            DurationT::zero());
    }, std::invalid_argument);
}

TYPED_TEST(MeasurementManagerTestTemplate, ConstructorResetsFilter) {
    using ManagerType = typename TestFixture::ManagerType;
    using DurationT = TypeParam;
    bool called = false;

    // Filter should be reset to initial state
    ManagerType mgr(this->state, this->seconds(0),
        [&called](const MockState& s) { called = true; }, 
        [this](DurationT dt) -> MockState { return this->state; }, 
        this->seconds(10));

    EXPECT_TRUE(called);
}

TYPED_TEST(MeasurementManagerTestTemplate, AddMeasurementValidation) {
    // Test null apply function
    EXPECT_FALSE(this->manager->add_measurement(this->seconds(1.0), nullptr));

    // Test measurement prior to initial time
    EXPECT_FALSE(this->manager->add_measurement(this->seconds(-1.0), this->CreateApplyFunction("invalid")));
    
    // Test too old measurement
    EXPECT_TRUE(this->manager->add_measurement(this->seconds(1.0), this->CreateApplyFunction("m1")));
    this->manager->update_to_time(this->seconds(100.0));
    
    auto time_89 = this->seconds(89.0);
    auto time_91 = this->seconds(91.0);
    auto time_100 = this->seconds(100.0);
    auto buffer_length = this->seconds(10.0);

    // This should be rejected as it's outside the buffer window
    bool result_89 = this->manager->add_measurement(time_89, this->CreateApplyFunction("old"));
    EXPECT_FALSE(result_89);
    
    // This should be accepted as it's within the buffer window  
    bool result_91 = this->manager->add_measurement(time_91, this->CreateApplyFunction("valid"));
    EXPECT_TRUE(result_91);
}


// Test empty measurement buffer
TYPED_TEST(MeasurementManagerTestTemplate, EmptyMeasurementBuffer) {
    auto state = this->manager->update_to_time(this->seconds(1.0));

    EXPECT_EQ(this->state.t, 1.0);
    std::vector<std::string> expected = {"p1.0"};
    this->VerifySequence(expected);
}

TYPED_TEST(MeasurementManagerTestTemplate, PredictOnlyNoMeasurements) {
    auto final_state = this->manager->update_to_time(this->seconds(5.0));
    
    EXPECT_EQ(final_state.t, 5.0);
    std::vector<std::string> expected = {"p5.0"};
    this->VerifySequence(expected);
    this->PrintApplied();
}

TYPED_TEST(MeasurementManagerTestTemplate, DuplicateUpdateToSameTime) {
    this->manager->update_to_time(this->seconds(2.0));
    this->applied.clear();

    auto final_state = this->manager->update_to_time(this->seconds(2.0));
    // Expect no reset and zero prediction
    std::vector<std::string> expected = {"p0.0"};
    this->VerifySequence(expected);
}

// In-sequence measurements should propagate correctly
TYPED_TEST(MeasurementManagerTestTemplate, StaticApply) {
    this->manager->add_measurement(this->seconds(0.0), this->CreateApplyFunction("m1"));
    this->manager->add_measurement(this->seconds(1.0), this->CreateApplyFunction("m2"));

    auto final_state = this->manager->update_to_time(this->seconds(2.0));

    std::vector<std::string> expected = {"p0.0",  // Buffer will start at measurement time o we will predict 0.0
                                         "m1",    // Then we will incorporate the first measurement
                                         "p1.0",  // Then we will predict to the nex time (1s)
                                         "m2",    // Then we will incorporate the second measurement
                                         "p1.0"}; // Finally, predcit to our final time
    this->VerifySequence(expected);
}

// In-sequence measurements split by a call to update_to_time should propagate and reset correctly
TYPED_TEST(MeasurementManagerTestTemplate, DynamicApply) {
    // Same as above
    this->manager->add_measurement(this->seconds(0.0), this->CreateApplyFunction("m1"));
    this->manager->add_measurement(this->seconds(1.0), this->CreateApplyFunction("m2"));

    auto final_state = this->manager->update_to_time(this->seconds(2.0));

    // Add a new measurement at same time as last update
    this->manager->add_measurement(this->seconds(2.0), this->CreateApplyFunction("m3"));

    final_state = this->manager->update_to_time(this->seconds(2.0));

    std::vector<std::string> expected = {"p0.0", "m1", "p1.0", "m2", "p1.0", // Same as above
                                         "p0.0",     // Predict to measurement time (same as current state time)
                                         "m3",       // Now we incorporate measurement
                                         "p0.0"};    // Finally, predict to req. time
    this->VerifySequence(expected);
}

TYPED_TEST(MeasurementManagerTestTemplate, UpdateToExactTime) {
    // Test updating to exactly the time of a measurement
    this->manager->add_measurement(this->seconds(1.0), this->CreateApplyFunction("m1"));
    this->manager->add_measurement(this->seconds(2.0), this->CreateApplyFunction("m2"));
    this->manager->add_measurement(this->seconds(3.0), this->CreateApplyFunction("m3"));
    
    // Update to exactly the time of the last measurement
    auto final_state = this->manager->update_to_time(this->seconds(3.0));
    
    // Should have applied all measurements up to and including m3
    EXPECT_NE(std::find(this->applied.begin(), this->applied.end(), "m1"), this->applied.end());
    EXPECT_NE(std::find(this->applied.begin(), this->applied.end(), "m2"), this->applied.end());
    EXPECT_NE(std::find(this->applied.begin(), this->applied.end(), "m3"), this->applied.end());
    
    // The final predict should have dt=0 since we're exactly at the measurement time
    auto last_predict = *(this->applied.end() - 1);
    EXPECT_EQ(last_predict, "p0.0");
}

// Edge case tests for update_to_time
TYPED_TEST(MeasurementManagerTestTemplate, EdgeCaseNoStateForMeasurements) {
    // This tests the case where we have measurements but no state
    // to reset to before the first measurement
    this->manager->add_measurement(this->seconds(5.0), this->CreateApplyFunction("m1"));
    this->manager->add_measurement(this->seconds(6.0), this->CreateApplyFunction("m2"));
    this->manager->add_measurement(this->seconds(7.0), this->CreateApplyFunction("m3"));
    
    auto final_state = this->manager->update_to_time(this->seconds(8.0));
    
    // First measurement should initialize state
    EXPECT_TRUE(std::find(this->applied.begin(), this->applied.end(), "m1") != this->applied.end());
    
    // Subsequent measurements should be this->applied in order
    auto m1_pos = std::find(this->applied.begin(), this->applied.end(), "m1") - this->applied.begin();
    auto m2_pos = std::find(this->applied.begin(), this->applied.end(), "m2") - this->applied.begin();
    auto m3_pos = std::find(this->applied.begin(), this->applied.end(), "m3") - this->applied.begin();
    
    EXPECT_LT(m1_pos, m2_pos);
    EXPECT_LT(m2_pos, m3_pos);
}

TYPED_TEST(MeasurementManagerTestTemplate, EdgeCaseExactStateMatch) {
    // Test when state buffer has exactly the min_timestamp_
    this->manager->add_measurement(this->seconds(0.0), this->CreateApplyFunction("m1"));
    this->manager->update_to_time(this->seconds(0.0));
    
    this->manager->add_measurement(this->seconds(1.0), this->CreateApplyFunction("m2"));
    this->manager->update_to_time(this->seconds(1.0));
    
    // Add measurement exactly at a state timestamp
    this->manager->add_measurement(this->seconds(1.0), this->CreateApplyFunction("m3"));
    
    this->applied.clear(); // Clear to focus on just this update
    auto final_state = this->manager->update_to_time(this->seconds(2.0));
    
    // Should reset to the state at time 2.0 and apply m3
    this->VerifyApplied("m3");
}

TYPED_TEST(MeasurementManagerTestTemplate, MeasurementWithSameTimestamp) {
    // Test handling of measurements with identical timestamps
    
    // Create multiple measurements with the same timestamp
    this->manager->add_measurement(this->seconds(1.0), this->CreateApplyFunction("m1a"));
    this->manager->add_measurement(this->seconds(1.0), this->CreateApplyFunction("m1b"));
    this->manager->add_measurement(this->seconds(1.0), this->CreateApplyFunction("m1c"));
    
    // Update to a time after all measurements
    auto final_state = this->manager->update_to_time(this->seconds(2.0));
    
    // All three measurements should be applied
    EXPECT_NE(std::find(this->applied.begin(), this->applied.end(), "m1a"), this->applied.end());
    EXPECT_NE(std::find(this->applied.begin(), this->applied.end(), "m1b"), this->applied.end());
    EXPECT_NE(std::find(this->applied.begin(), this->applied.end(), "m1c"), this->applied.end());
    
    // Check that safe_find_buffer_key incremented the timestamp values
    auto m1a_pos = std::find(this->applied.begin(), this->applied.end(), "m1a") - this->applied.begin();
    auto m1b_pos = std::find(this->applied.begin(), this->applied.end(), "m1b") - this->applied.begin();
    auto m1c_pos = std::find(this->applied.begin(), this->applied.end(), "m1c") - this->applied.begin();
    
    // They should all appear in the order they were added
    EXPECT_LT(m1a_pos, m1b_pos);
    EXPECT_LT(m1b_pos, m1c_pos);
}

// Out-of-sequence measurements should cause filter to reset and propagate properly
TYPED_TEST(MeasurementManagerTestTemplate, BackwardsMeasurements) {
    this->manager->add_measurement(this->seconds(0.0), this->CreateApplyFunction("m1"));
    auto final_state = this->manager->update_to_time(this->seconds(1.0));

    this->manager->add_measurement(this->seconds(2.0), this->CreateApplyFunction("m2"));
    final_state = this->manager->update_to_time(this->seconds(3.0));

    this->manager->add_measurement(this->seconds(1.0), this->CreateApplyFunction("m3"));
    final_state = this->manager->update_to_time(this->seconds(4.0));

    std::vector<std::string> expected = {"p0.0",     // to first measurement
                                         "m1",       // First measurement
                                         "p1.0",     // To first update time
                                         "p1.0",     // To m2
                                         "m2",
                                         "p1.0",     // To 3.0s
                                         "reset1.0",
                                         "p0.0",     // No reset required, already at right time
                                         "m3",
                                         "p1.0",     // To m2
                                         "m2",
                                         "p2.0"};    // To final time requested
    this->VerifySequence(expected);
    EXPECT_EQ(final_state.t, 4.0);
}

TYPED_TEST(MeasurementManagerTestTemplate, NoRedundantResetIfSameTime) {
    this->manager->add_measurement(this->seconds(1.0), this->CreateApplyFunction("m1"));
    this->manager->update_to_time(this->seconds(2.0));
    this->applied.clear();

    // Add new measurement after that time
    this->manager->add_measurement(this->seconds(3.0), this->CreateApplyFunction("m2"));
    this->manager->update_to_time(this->seconds(4.0));

    // Ensure "reset2.0" or equivalent does not appear
    for (const auto& entry : this->applied) {
        EXPECT_EQ(entry.find("reset"), std::string::npos);
    }
}

// Edge case of measurement prior to first measurement
TYPED_TEST(MeasurementManagerTestTemplate, MeasurementBeforeFirst) {
    this->manager->add_measurement(this->seconds(1.0), this->CreateApplyFunction("m1"));
    auto final_state = this->manager->update_to_time(this->seconds(1.0));

    this->manager->add_measurement(this->seconds(0.0), this->CreateApplyFunction("m2"));
    final_state = this->manager->update_to_time(this->seconds(1.0));

    std::vector<std::string> expected = {"p1.0",
                                         "m1",
                                         "p0.0",
                                         "reset0.0",
                                         "p0.0",
                                         "m2",
                                         "p1.0",
                                         "m1",
                                         "p0.0"};
    this->VerifySequence(expected);
}

// Out-of-sequence measurements should cause invalidate states to be removed
TYPED_TEST(MeasurementManagerTestTemplate, OldStateRemoval) {
    // Similar to previous test, we perform a backwards measurement update
    // This backwards update test will remove the state entered with "update_to_time(3.0)"
    // As we don't stop at t=2.0s on the forward propagation anymore
    this->manager->add_measurement(this->seconds(0.0), this->CreateApplyFunction("m1"));
    auto final_state = this->manager->update_to_time(this->seconds(1.0));

    this->manager->add_measurement(this->seconds(2.0), this->CreateApplyFunction("m2"));
    final_state = this->manager->update_to_time(this->seconds(3.0));

    this->manager->add_measurement(this->seconds(1.0), this->CreateApplyFunction("m3"));
    final_state = this->manager->update_to_time(this->seconds(4.0));

    // This should cause a 'reset' to t=2.0s, which should be closest state reference
    this->manager->add_measurement(this->seconds(3.0), this->CreateApplyFunction("m4"));
    final_state = this->manager->update_to_time(this->seconds(4.0));


    this->VerifyNotApplied("reset3.0");
    this->VerifyApplied("reset2.0");

    EXPECT_EQ(this->state.t, 4.0);
}

// Test timestamp deduplication
TYPED_TEST(MeasurementManagerTestTemplate, TimestampDeduplication) {
    // Add two measurements with the same timestamp
    this->manager->add_measurement(this->seconds(1.0), this->CreateApplyFunction("m1"));
    this->manager->add_measurement(this->seconds(1.0), this->CreateApplyFunction("m2"));
    
    auto final_state = this->manager->update_to_time(this->seconds(2.0));
    
    // Both measurements should be applied
    std::vector<std::string> expected = {"p1.0",
                                         "m1",   // Added first
                                         "p0.0", // Very small prediction time
                                         "m2",   // Now m2
                                         "p1.0"};
    this->VerifySequence(expected);
}

// Test buffer pruning
TYPED_TEST(MeasurementManagerTestTemplate, BufferPruning) {
    // Add measurements spanning more than the buffer window
    this->manager->add_measurement(this->seconds(0.0), this->CreateApplyFunction("m1"));
    this->manager->add_measurement(this->seconds(5.0), this->CreateApplyFunction("m2"));
    this->manager->add_measurement(this->seconds(10.0), this->CreateApplyFunction("m3"));
    this->manager->add_measurement(this->seconds(15.0), this->CreateApplyFunction("m4"));
    this->manager->add_measurement(this->seconds(20.0), this->CreateApplyFunction("m5"));

    // Removal only happens when update_to_time is called
    auto final_state = this->manager->update_to_time(this->seconds(20.0));

    // We should let in one at 20.0-10.0 = 10.0
    EXPECT_TRUE(this->manager->add_measurement(this->seconds(10.0), this->CreateApplyFunction("edge")));
    // Now, if we add a measurement prior to 3.0s, it should reject it
    EXPECT_FALSE(this->manager->add_measurement(this->seconds(
                                                    std::nextafter(3.0, -std::numeric_limits<double>::infinity())), 
                                                this->CreateApplyFunction("old")));
    this->applied.clear();

    this->manager->update_to_time(this->seconds(20.0));

    std::vector<std::string> expected = {"reset10.0",
                                         "p0.0",
                                         "m3",      // Orig. t=10 meas
                                         "p0.0",
                                         "edge",    // New one comes after
                                         "p5.0",
                                         "m4",
                                         "p5.0",
                                         "m5",
                                         "p0.0"};

    this->VerifySequence(expected);

    EXPECT_EQ(this->manager->get_state_buffer_size(), 3); // @ 10, 15, 20
    EXPECT_EQ(this->manager->get_measurement_buffer_size(), 4); // m3, edge, m4, m5
}