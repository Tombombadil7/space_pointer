#include <unity.h>
#include <math.h>
#include "tracking_math.h"

void test_wraps_across_positive_boundary() {
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -179.0f, TrackingMath::wrapAngle(181.0f));
}

void test_wraps_across_negative_boundary() {
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 179.0f, TrackingMath::wrapAngle(-181.0f));
}

void test_full_revolution_is_zero() {
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, TrackingMath::wrapAngle(360.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, TrackingMath::wrapAngle(-720.0f));
}

void test_preserves_positive_and_negative_half_turns() {
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 180.0f, TrackingMath::wrapAngle(540.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -180.0f, TrackingMath::wrapAngle(-540.0f));
}

void test_rejects_non_finite_angles() {
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, TrackingMath::wrapAngle(NAN));
}

void test_chronological_event_pair_accepts_rise_before_set() {
    TEST_ASSERT_TRUE(TrackingMath::chronologicalEventPair(10.0, 10.5));
}

void test_chronological_event_pair_rejects_reversed_or_equal_times() {
    TEST_ASSERT_FALSE(TrackingMath::chronologicalEventPair(10.5, 10.0));
    TEST_ASSERT_FALSE(TrackingMath::chronologicalEventPair(10.0, 10.0));
}

void test_chronological_event_pair_rejects_non_finite_times() {
    TEST_ASSERT_FALSE(TrackingMath::chronologicalEventPair(NAN, 10.0));
    TEST_ASSERT_FALSE(TrackingMath::chronologicalEventPair(10.0, INFINITY));
}

void test_elevation_validation_accepts_mechanical_boundaries() {
    TEST_ASSERT_TRUE(TrackingMath::validElevation(0.0f));
    TEST_ASSERT_TRUE(TrackingMath::validElevation(90.0f));
    TEST_ASSERT_TRUE(TrackingMath::validElevation(45.0f));
}

void test_elevation_validation_rejects_out_of_range_and_non_finite_values() {
    TEST_ASSERT_FALSE(TrackingMath::validElevation(-0.01f));
    TEST_ASSERT_FALSE(TrackingMath::validElevation(90.01f));
    TEST_ASSERT_FALSE(TrackingMath::validElevation(NAN));
    TEST_ASSERT_FALSE(TrackingMath::validElevation(INFINITY));
}

void test_elevation_error_is_linear_not_circular() {
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 25.0f,
                             TrackingMath::elevationError(45.0f, 20.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -70.0f,
                             TrackingMath::elevationError(10.0f, 80.0f));
}

void test_validates_observer_coordinates_and_altitude() {
    TEST_ASSERT_TRUE(TrackingMath::validObserver(32.0, 34.8, 50.0));
    TEST_ASSERT_FALSE(TrackingMath::validObserver(91.0, 34.8, 50.0));
    TEST_ASSERT_FALSE(TrackingMath::validObserver(32.0, 181.0, 50.0));
    TEST_ASSERT_FALSE(TrackingMath::validObserver(32.0, 34.8, 50000.0));
}

void test_validates_norad_ids() {
    TEST_ASSERT_TRUE(TrackingMath::validNoradId(25544));
    TEST_ASSERT_FALSE(TrackingMath::validNoradId(0));
    TEST_ASSERT_FALSE(TrackingMath::validNoradId(1000000));
}

void test_accepts_only_supported_bodies() {
    TEST_ASSERT_TRUE(TrackingMath::supportedPlanetBody(10));
    TEST_ASSERT_TRUE(TrackingMath::supportedPlanetBody(3));
    TEST_ASSERT_TRUE(TrackingMath::supportedPlanetBody(4));
    TEST_ASSERT_TRUE(TrackingMath::supportedPlanetBody(5));
    TEST_ASSERT_FALSE(TrackingMath::supportedPlanetBody(0));
    TEST_ASSERT_FALSE(TrackingMath::supportedPlanetBody(1));
}


void test_proportional_controller_uses_nearest_step_rounding() {
    TEST_ASSERT_EQUAL_INT(1, TrackingMath::proportionalSteps(2.5f, 0.05f, 0.2f, 8));
    TEST_ASSERT_EQUAL_INT(-1, TrackingMath::proportionalSteps(2.5f, 0.05f, -0.2f, 8));
}

void test_proportional_controller_respects_deadband() {
    TEST_ASSERT_EQUAL_INT(0, TrackingMath::proportionalSteps(3.0f, 0.05f, 0.05f, 8));
    TEST_ASSERT_EQUAL_INT(0, TrackingMath::proportionalSteps(3.0f, 0.05f, -0.04f, 8));
}

void test_proportional_controller_caps_large_corrections() {
    TEST_ASSERT_EQUAL_INT(8, TrackingMath::proportionalSteps(3.0f, 0.05f, 90.0f, 8));
    TEST_ASSERT_EQUAL_INT(-8, TrackingMath::proportionalSteps(3.0f, 0.05f, -90.0f, 8));
}

void test_proportional_controller_ignores_non_finite_error() {
    TEST_ASSERT_EQUAL_INT(0, TrackingMath::proportionalSteps(3.0f, 0.05f, NAN, 8));
    TEST_ASSERT_EQUAL_INT(0, TrackingMath::proportionalSteps(3.0f, 0.05f, INFINITY, 8));
}

void test_observer_validation_accepts_documented_boundaries() {
    TEST_ASSERT_TRUE(TrackingMath::validObserver(-90.0, -180.0, -500.0));
    TEST_ASSERT_TRUE(TrackingMath::validObserver(90.0, 180.0, 10000.0));
}

void test_observer_validation_rejects_non_finite_values() {
    TEST_ASSERT_FALSE(TrackingMath::validObserver(NAN, 34.8, 50.0));
    TEST_ASSERT_FALSE(TrackingMath::validObserver(32.0, INFINITY, 50.0));
    TEST_ASSERT_FALSE(TrackingMath::validObserver(32.0, 34.8, NAN));
}


void test_wrap_angle_keeps_exact_half_turn_boundaries() {
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 180.0f, TrackingMath::wrapAngle(180.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -180.0f, TrackingMath::wrapAngle(-180.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 180.0f, TrackingMath::wrapAngle(900.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -180.0f, TrackingMath::wrapAngle(-900.0f));
}

void test_wrap_angle_handles_infinity() {
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, TrackingMath::wrapAngle(INFINITY));
}

void test_observer_validation_rejects_all_out_of_range_edges() {
    TEST_ASSERT_FALSE(TrackingMath::validObserver(-90.01, 0.0, 0.0));
    TEST_ASSERT_FALSE(TrackingMath::validObserver(90.01, 0.0, 0.0));
    TEST_ASSERT_FALSE(TrackingMath::validObserver(0.0, -180.01, 0.0));
    TEST_ASSERT_FALSE(TrackingMath::validObserver(0.0, 180.01, 0.0));
    TEST_ASSERT_FALSE(TrackingMath::validObserver(0.0, 0.0, -500.01));
    TEST_ASSERT_FALSE(TrackingMath::validObserver(0.0, 0.0, 10000.01));
}

void test_norad_validation_accepts_exact_range_edges() {
    TEST_ASSERT_TRUE(TrackingMath::validNoradId(1));
    TEST_ASSERT_TRUE(TrackingMath::validNoradId(999999));
    TEST_ASSERT_FALSE(TrackingMath::validNoradId(-1));
}

void test_proportional_controller_handles_zero_gain_and_zero_step_limit() {
    TEST_ASSERT_EQUAL_INT(0, TrackingMath::proportionalSteps(0.0f, 0.0f, 5.0f, 8));
    TEST_ASSERT_EQUAL_INT(0, TrackingMath::proportionalSteps(3.0f, 0.05f, 5.0f, 0));
}


void test_generation_readiness_starts_not_ready() {
    TrackingMath::GenerationReadiness readiness;
    TEST_ASSERT_FALSE(readiness.matches(1, 1));
}

void test_generation_readiness_matches_only_published_pair() {
    TrackingMath::GenerationReadiness readiness;
    readiness.publish(7, 12);
    TEST_ASSERT_TRUE(readiness.matches(7, 12));
    TEST_ASSERT_FALSE(readiness.matches(7, 13));
    TEST_ASSERT_FALSE(readiness.matches(8, 12));
}

void test_generation_readiness_clear_invalidates_previous_pair() {
    TrackingMath::GenerationReadiness readiness;
    readiness.publish(4, 9);
    readiness.clear();
    TEST_ASSERT_FALSE(readiness.matches(4, 9));
    TEST_ASSERT_EQUAL_UINT32(0, readiness.targetGeneration);
    TEST_ASSERT_EQUAL_UINT32(0, readiness.observerGeneration);
}

void test_generation_readiness_rejects_zero_generations() {
    TrackingMath::GenerationReadiness readiness;
    readiness.publish(0, 5);
    TEST_ASSERT_FALSE(readiness.matches(0, 5));
    readiness.publish(5, 0);
    TEST_ASSERT_FALSE(readiness.matches(5, 0));
    TEST_ASSERT_FALSE(readiness.matches(5, 5));
}

void test_generation_readiness_republish_replaces_both_generations() {
    TrackingMath::GenerationReadiness readiness;
    readiness.publish(1, 2);
    readiness.publish(3, 4);
    TEST_ASSERT_FALSE(readiness.matches(1, 2));
    TEST_ASSERT_TRUE(readiness.matches(3, 4));
}


/*
 * TLE data is tied to both the selected satellite and the observer location.
 * These tests model the checks used before SGP4 calculations are allowed.
 */
void test_tle_source_token_rejects_a_new_target_generation() {
    TrackingMath::GenerationReadiness tleSource;
    tleSource.publish(12, 4);
    TEST_ASSERT_TRUE(tleSource.matches(12, 4));
    TEST_ASSERT_FALSE(tleSource.matches(13, 4));
}

void test_tle_source_token_rejects_a_new_observer_generation() {
    TrackingMath::GenerationReadiness tleSource;
    tleSource.publish(12, 4);
    TEST_ASSERT_TRUE(tleSource.matches(12, 4));
    TEST_ASSERT_FALSE(tleSource.matches(12, 5));
}


/*
 * The web API keeps pointing/position telemetry and pass predictions on
 * separate generation pairs because pass calculations run less frequently.
 */
void test_telemetry_source_pair_rejects_old_target_or_observer() {
    TrackingMath::GenerationReadiness telemetrySource;
    telemetrySource.publish(20, 30);
    TEST_ASSERT_TRUE(telemetrySource.matches(20, 30));
    TEST_ASSERT_FALSE(telemetrySource.matches(21, 30));
    TEST_ASSERT_FALSE(telemetrySource.matches(20, 31));
}

void test_pass_prediction_source_pair_is_independent() {
    TrackingMath::GenerationReadiness telemetrySource;
    TrackingMath::GenerationReadiness passSource;
    telemetrySource.publish(8, 9);
    passSource.publish(8, 9);

    // A fresh pointing update does not make an old pass prediction current.
    telemetrySource.publish(8, 10);
    TEST_ASSERT_TRUE(telemetrySource.matches(8, 10));
    TEST_ASSERT_FALSE(passSource.matches(8, 10));

    // Once the pass calculation catches up, both groups describe the same
    // selected target and observer location.
    passSource.publish(8, 10);
    TEST_ASSERT_TRUE(passSource.matches(8, 10));
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_wraps_across_positive_boundary);
    RUN_TEST(test_wraps_across_negative_boundary);
    RUN_TEST(test_full_revolution_is_zero);
    RUN_TEST(test_preserves_positive_and_negative_half_turns);
    RUN_TEST(test_rejects_non_finite_angles);
    RUN_TEST(test_chronological_event_pair_accepts_rise_before_set);
    RUN_TEST(test_chronological_event_pair_rejects_reversed_or_equal_times);
    RUN_TEST(test_chronological_event_pair_rejects_non_finite_times);
    RUN_TEST(test_elevation_validation_accepts_mechanical_boundaries);
    RUN_TEST(test_elevation_validation_rejects_out_of_range_and_non_finite_values);
    RUN_TEST(test_elevation_error_is_linear_not_circular);
    RUN_TEST(test_validates_observer_coordinates_and_altitude);
    RUN_TEST(test_validates_norad_ids);
    RUN_TEST(test_accepts_only_supported_bodies);
    RUN_TEST(test_proportional_controller_uses_nearest_step_rounding);
    RUN_TEST(test_proportional_controller_respects_deadband);
    RUN_TEST(test_proportional_controller_caps_large_corrections);
    RUN_TEST(test_proportional_controller_ignores_non_finite_error);
    RUN_TEST(test_observer_validation_accepts_documented_boundaries);
    RUN_TEST(test_observer_validation_rejects_non_finite_values);
    RUN_TEST(test_wrap_angle_keeps_exact_half_turn_boundaries);
    RUN_TEST(test_wrap_angle_handles_infinity);
    RUN_TEST(test_observer_validation_rejects_all_out_of_range_edges);
    RUN_TEST(test_norad_validation_accepts_exact_range_edges);
    RUN_TEST(test_proportional_controller_handles_zero_gain_and_zero_step_limit);
    RUN_TEST(test_generation_readiness_starts_not_ready);
    RUN_TEST(test_generation_readiness_matches_only_published_pair);
    RUN_TEST(test_generation_readiness_clear_invalidates_previous_pair);
    RUN_TEST(test_generation_readiness_rejects_zero_generations);
    RUN_TEST(test_generation_readiness_republish_replaces_both_generations);
    RUN_TEST(test_tle_source_token_rejects_a_new_target_generation);
    RUN_TEST(test_tle_source_token_rejects_a_new_observer_generation);
    RUN_TEST(test_telemetry_source_pair_rejects_old_target_or_observer);
    RUN_TEST(test_pass_prediction_source_pair_is_independent);
    return UNITY_END();
}
