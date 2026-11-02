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
    return UNITY_END();
}
