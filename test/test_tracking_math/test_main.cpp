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
    return UNITY_END();
}
