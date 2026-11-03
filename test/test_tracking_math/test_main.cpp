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

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_wraps_across_positive_boundary);
    RUN_TEST(test_wraps_across_negative_boundary);
    RUN_TEST(test_full_revolution_is_zero);
    RUN_TEST(test_preserves_positive_and_negative_half_turns);
    RUN_TEST(test_rejects_non_finite_angles);
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
    return UNITY_END();
}
