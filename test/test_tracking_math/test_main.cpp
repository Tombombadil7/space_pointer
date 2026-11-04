#include <unity.h>
#include <math.h>
#include "tracking_math.h"
#include <astronomy.h>

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


/*
 * Exercise the actual Astronomy Engine library used by the firmware, not a
 * reimplementation of its equations. Equatorial-to-horizon results must stay
 * finite and within their documented angular ranges for every supported body.
 */
void assert_valid_planet_horizon(astro_body_t body) {
    astro_time_t time = Astronomy_MakeTime(2024, 4, 8, 18, 0, 0.0);
    const astro_observer_t observer = {0.0, 0.0, 0.0};
    const astro_equatorial_t equ = Astronomy_Equator(
        body, &time, observer, EQUATOR_OF_DATE, ABERRATION);
    TEST_ASSERT_EQUAL_INT(ASTRO_SUCCESS, equ.status);
    TEST_ASSERT_TRUE(isfinite(equ.ra));
    TEST_ASSERT_TRUE(isfinite(equ.dec));
    TEST_ASSERT_TRUE(isfinite(equ.dist));
    TEST_ASSERT_TRUE(equ.ra >= 0.0 && equ.ra < 24.0);
    TEST_ASSERT_TRUE(equ.dec >= -90.0 && equ.dec <= 90.0);
    TEST_ASSERT_TRUE(equ.dist > 0.0);

    const astro_horizon_t horizon = Astronomy_Horizon(
        &time, observer, equ.ra, equ.dec, REFRACTION_NORMAL);
    TEST_ASSERT_TRUE(isfinite(horizon.azimuth));
    TEST_ASSERT_TRUE(isfinite(horizon.altitude));
    TEST_ASSERT_TRUE(horizon.azimuth >= 0.0 && horizon.azimuth < 360.0);
    TEST_ASSERT_TRUE(horizon.altitude >= -90.0 && horizon.altitude <= 90.0);
}

void test_astronomy_engine_supported_bodies_produce_valid_horizon_coordinates() {
    assert_valid_planet_horizon(BODY_MOON);
    assert_valid_planet_horizon(BODY_MARS);
    assert_valid_planet_horizon(BODY_JUPITER);
    assert_valid_planet_horizon(BODY_SATURN);
}

/*
 * Reproduce the firmware's sequential rise-then-set search at the equator.
 * This catches invalid event status and reversed/equal event timestamps using
 * the real ephemeris and rise/set search implementation.
 */
void assert_chronological_planet_rise_set(astro_body_t body) {
    const astro_time_t start = Astronomy_MakeTime(2024, 4, 8, 0, 0, 0.0);
    const astro_observer_t observer = {0.0, 0.0, 0.0};
    const astro_search_result_t rise = Astronomy_SearchRiseSet(
        body, observer, DIRECTION_RISE, start, 1.0);
    TEST_ASSERT_EQUAL_INT(ASTRO_SUCCESS, rise.status);

    const astro_time_t afterRise = Astronomy_AddDays(rise.time, 1.0 / 86400.0);
    const astro_search_result_t set = Astronomy_SearchRiseSet(
        body, observer, DIRECTION_SET, afterRise, 1.0);
    TEST_ASSERT_EQUAL_INT(ASTRO_SUCCESS, set.status);
    TEST_ASSERT_TRUE(TrackingMath::chronologicalEventPair(
        rise.time.ut, set.time.ut));
}

void test_astronomy_engine_rise_set_search_is_chronological_for_supported_bodies() {
    assert_chronological_planet_rise_set(BODY_MOON);
    assert_chronological_planet_rise_set(BODY_MARS);
    assert_chronological_planet_rise_set(BODY_JUPITER);
    assert_chronological_planet_rise_set(BODY_SATURN);
}

/* Verify the J2000-to-Unix conversion constant used by firmware pass times. */
void test_astronomy_unix_epoch_conversion_matches_utc_epoch() {
    const astro_time_t unixEpoch = Astronomy_MakeTime(1970, 1, 1, 0, 0, 0.0);
    const double unixSeconds = unixEpoch.ut * 86400.0 + 946728000.0;
    TEST_ASSERT_TRUE(fabs(unixSeconds) <= 1.0);
}



void test_encoder_elevation_accepts_calibrated_mechanical_boundaries() {
    float elevation = NAN;
    TEST_ASSERT_TRUE(TrackingMath::encoderElevation(2048, 2048, 1, elevation));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, elevation);
    TEST_ASSERT_TRUE(TrackingMath::encoderElevation(3072, 2048, 1, elevation));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 90.0f, elevation);
    TEST_ASSERT_TRUE(TrackingMath::encoderElevation(1024, 2048, -1, elevation));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 90.0f, elevation);
}

void test_encoder_elevation_rejects_invalid_raw_values_and_calibration() {
    float elevation = 0.0f;
    TEST_ASSERT_FALSE(TrackingMath::encoderElevation(4096, 2048, 1, elevation));
    TEST_ASSERT_FALSE(TrackingMath::encoderElevation(2048, -1, 1, elevation));
    TEST_ASSERT_FALSE(TrackingMath::encoderElevation(2048, 4096, 1, elevation));
    TEST_ASSERT_FALSE(TrackingMath::encoderElevation(2048, 2048, 0, elevation));
    TEST_ASSERT_FALSE(TrackingMath::encoderElevation(2048, 2048, 2, elevation));
    TEST_ASSERT_FALSE(TrackingMath::encoderElevation(3073, 2048, 1, elevation));
    TEST_ASSERT_FALSE(TrackingMath::encoderElevation(1023, 2048, -1, elevation));
}

void test_encoder_elevation_exhaustively_checks_all_12_bit_readings() {
    float elevation = 0.0f;
    for (uint16_t raw = 0; raw <= 4095; ++raw) {
        const bool positiveDirection =
            TrackingMath::encoderElevation(raw, 2048, 1, elevation);
        TEST_ASSERT_EQUAL((raw >= 2048 && raw <= 3072), positiveDirection);

        const bool negativeDirection =
            TrackingMath::encoderElevation(raw, 2048, -1, elevation);
        TEST_ASSERT_EQUAL((raw >= 1024 && raw <= 2048), negativeDirection);
    }
}

void test_true_heading_wraps_east_and_west_declination() {
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.5f, TrackingMath::trueHeading(359.5f, 2.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 358.0f, TrackingMath::trueHeading(0.0f, -2.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, TrackingMath::trueHeading(180.0f, 180.0f));
}

void test_true_heading_rejects_non_finite_inputs() {
    TEST_ASSERT_TRUE(isnan(TrackingMath::trueHeading(NAN, 0.0f)));
    TEST_ASSERT_TRUE(isnan(TrackingMath::trueHeading(10.0f, INFINITY)));
}

void test_azimuth_error_uses_shortest_circular_direction() {
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.0f, TrackingMath::azimuthError(1.0f, 359.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -2.0f, TrackingMath::azimuthError(359.0f, 1.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 180.0f, TrackingMath::azimuthError(180.0f, 0.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -180.0f, TrackingMath::azimuthError(0.0f, 180.0f));
}



void test_finite_double_parser_accepts_valid_decimal_and_exponent_values() {
    double value = 0.0;
    TEST_ASSERT_TRUE(TrackingMath::parseFiniteDouble("32.5", value));
    TEST_ASSERT_TRUE(fabs(value - 32.5) < 0.000001);
    TEST_ASSERT_TRUE(TrackingMath::parseFiniteDouble("-1.25e2", value));
    TEST_ASSERT_TRUE(fabs(value + 125.0) < 0.000001);
    TEST_ASSERT_TRUE(TrackingMath::parseFiniteDouble(" 50", value));
    TEST_ASSERT_TRUE(fabs(value - 50.0) < 0.000001);
}

void test_finite_double_parser_rejects_partial_nonfinite_and_range_errors() {
    double value = 123.0;
    TEST_ASSERT_FALSE(TrackingMath::parseFiniteDouble(NULL, value));
    TEST_ASSERT_FALSE(TrackingMath::parseFiniteDouble("", value));
    TEST_ASSERT_FALSE(TrackingMath::parseFiniteDouble("12.5x", value));
    TEST_ASSERT_FALSE(TrackingMath::parseFiniteDouble("12.5 ", value));
    TEST_ASSERT_FALSE(TrackingMath::parseFiniteDouble("nan", value));
    TEST_ASSERT_FALSE(TrackingMath::parseFiniteDouble("inf", value));
    TEST_ASSERT_FALSE(TrackingMath::parseFiniteDouble("-inf", value));
    TEST_ASSERT_FALSE(TrackingMath::parseFiniteDouble("1e9999", value));
    TEST_ASSERT_FALSE(TrackingMath::parseFiniteDouble("1e-9999", value));
}

void test_long_integer_parser_accepts_only_complete_integer_strings() {
    long value = 0;
    TEST_ASSERT_TRUE(TrackingMath::parseLongInteger("25544", value));
    TEST_ASSERT_EQUAL_INT(25544, value);
    TEST_ASSERT_TRUE(TrackingMath::parseLongInteger("-1", value));
    TEST_ASSERT_EQUAL_INT(-1, value);
    TEST_ASSERT_TRUE(TrackingMath::parseLongInteger(" 42", value));
    TEST_ASSERT_EQUAL_INT(42, value);
}

void test_long_integer_parser_rejects_empty_partial_and_overflow_values() {
    long value = 0;
    TEST_ASSERT_FALSE(TrackingMath::parseLongInteger(NULL, value));
    TEST_ASSERT_FALSE(TrackingMath::parseLongInteger("", value));
    TEST_ASSERT_FALSE(TrackingMath::parseLongInteger("42.0", value));
    TEST_ASSERT_FALSE(TrackingMath::parseLongInteger("42x", value));
    TEST_ASSERT_FALSE(TrackingMath::parseLongInteger("42 ", value));
    TEST_ASSERT_FALSE(TrackingMath::parseLongInteger("999999999999999999999999999999999", value));
}



void test_proportional_controller_clamps_extreme_finite_inputs_before_rounding() {
    TEST_ASSERT_EQUAL_INT(8, TrackingMath::proportionalSteps(3.0e30f, 0.0f, 3.0e30f, 8));
    TEST_ASSERT_EQUAL_INT(-8, TrackingMath::proportionalSteps(3.0e30f, 0.0f, -3.0e30f, 8));
    TEST_ASSERT_EQUAL_INT(8, TrackingMath::proportionalSteps(1.0f, 0.0f, 8.5f, 8));
    TEST_ASSERT_EQUAL_INT(-8, TrackingMath::proportionalSteps(1.0f, 0.0f, -8.5f, 8));
}

void test_proportional_controller_rejects_invalid_gain_deadband_and_limit() {
    TEST_ASSERT_EQUAL_INT(0, TrackingMath::proportionalSteps(NAN, 0.05f, 1.0f, 8));
    TEST_ASSERT_EQUAL_INT(0, TrackingMath::proportionalSteps(INFINITY, 0.05f, 1.0f, 8));
    TEST_ASSERT_EQUAL_INT(0, TrackingMath::proportionalSteps(1.0f, NAN, 1.0f, 8));
    TEST_ASSERT_EQUAL_INT(0, TrackingMath::proportionalSteps(1.0f, -0.1f, 1.0f, 8));
    TEST_ASSERT_EQUAL_INT(0, TrackingMath::proportionalSteps(1.0f, 0.05f, 1.0f, -8));
}



void test_motor_readiness_requires_all_prerequisites_and_satellite_tle() {
    TEST_ASSERT_TRUE(TrackingMath::motorControlReady(
        true, true, true, true, false, false, true, true, 180.0f, 45.0f));
    TEST_ASSERT_TRUE(TrackingMath::motorControlReady(
        true, true, true, true, true, true, true, true, 180.0f, 45.0f));
    TEST_ASSERT_FALSE(TrackingMath::motorControlReady(
        true, true, true, true, true, false, true, true, 180.0f, 45.0f));
    TEST_ASSERT_FALSE(TrackingMath::motorControlReady(
        false, true, true, true, false, false, true, true, 180.0f, 45.0f));
    TEST_ASSERT_FALSE(TrackingMath::motorControlReady(
        true, false, true, true, false, false, true, true, 180.0f, 45.0f));
    TEST_ASSERT_FALSE(TrackingMath::motorControlReady(
        true, true, false, true, false, false, true, true, 180.0f, 45.0f));
    TEST_ASSERT_FALSE(TrackingMath::motorControlReady(
        true, true, true, false, false, false, true, true, 180.0f, 45.0f));
    TEST_ASSERT_FALSE(TrackingMath::motorControlReady(
        true, true, true, true, false, false, false, true, 180.0f, 45.0f));
    TEST_ASSERT_FALSE(TrackingMath::motorControlReady(
        true, true, true, true, false, false, true, false, 180.0f, 45.0f));
    TEST_ASSERT_FALSE(TrackingMath::motorControlReady(
        true, true, true, true, false, false, true, true, NAN, 45.0f));
    TEST_ASSERT_FALSE(TrackingMath::motorControlReady(
        true, true, true, true, false, false, true, true, 180.0f, 90.1f));
}

void test_motor_sensor_sample_accepts_calibrated_valid_boundaries() {
    TEST_ASSERT_TRUE(TrackingMath::validMotorSensorSample(
        true, true, 3, 3, 0.0f, 0));
    TEST_ASSERT_TRUE(TrackingMath::validMotorSensorSample(
        true, true, 3, 3, 359.999f, 4095));
}

void test_motor_sensor_sample_rejects_missing_or_invalid_sensor_data() {
    TEST_ASSERT_FALSE(TrackingMath::validMotorSensorSample(
        false, true, 3, 3, 180.0f, 2048));
    TEST_ASSERT_FALSE(TrackingMath::validMotorSensorSample(
        true, false, 3, 3, 180.0f, 2048));
    TEST_ASSERT_FALSE(TrackingMath::validMotorSensorSample(
        true, true, 2, 3, 180.0f, 2048));
    TEST_ASSERT_FALSE(TrackingMath::validMotorSensorSample(
        true, true, 3, 2, 180.0f, 2048));
    TEST_ASSERT_FALSE(TrackingMath::validMotorSensorSample(
        true, true, 3, 3, NAN, 2048));
    TEST_ASSERT_FALSE(TrackingMath::validMotorSensorSample(
        true, true, 3, 3, -0.1f, 2048));
    TEST_ASSERT_FALSE(TrackingMath::validMotorSensorSample(
        true, true, 3, 3, 360.0f, 2048));
    TEST_ASSERT_FALSE(TrackingMath::validMotorSensorSample(
        true, true, 3, 3, 180.0f, 4096));
}


int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_motor_readiness_requires_all_prerequisites_and_satellite_tle);
    RUN_TEST(test_motor_sensor_sample_accepts_calibrated_valid_boundaries);
    RUN_TEST(test_motor_sensor_sample_rejects_missing_or_invalid_sensor_data);
    RUN_TEST(test_proportional_controller_clamps_extreme_finite_inputs_before_rounding);
    RUN_TEST(test_proportional_controller_rejects_invalid_gain_deadband_and_limit);
    RUN_TEST(test_finite_double_parser_accepts_valid_decimal_and_exponent_values);
    RUN_TEST(test_finite_double_parser_rejects_partial_nonfinite_and_range_errors);
    RUN_TEST(test_long_integer_parser_accepts_only_complete_integer_strings);
    RUN_TEST(test_long_integer_parser_rejects_empty_partial_and_overflow_values);
    RUN_TEST(test_encoder_elevation_accepts_calibrated_mechanical_boundaries);
    RUN_TEST(test_encoder_elevation_rejects_invalid_raw_values_and_calibration);
    RUN_TEST(test_encoder_elevation_exhaustively_checks_all_12_bit_readings);
    RUN_TEST(test_true_heading_wraps_east_and_west_declination);
    RUN_TEST(test_true_heading_rejects_non_finite_inputs);
    RUN_TEST(test_azimuth_error_uses_shortest_circular_direction);
    RUN_TEST(test_astronomy_engine_supported_bodies_produce_valid_horizon_coordinates);
    RUN_TEST(test_astronomy_engine_rise_set_search_is_chronological_for_supported_bodies);
    RUN_TEST(test_astronomy_unix_epoch_conversion_matches_utc_epoch);
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
