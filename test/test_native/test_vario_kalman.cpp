#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

// Declares VarioKalman, its tuning constants, and the diagnostics API.
#include "Vario.h"

namespace
{

// Feed `count` samples of a constant climb of `rateMS`, `dtMs` apart,
// starting at (startAltM, startMs). Returns the time of the last sample.
// This only prepares input data; it does not reimplement the filter.
uint32_t feedLinearClimb(VarioKalman& kf, float startAltM, uint32_t startMs,
                         float rateMS, uint32_t dtMs, int count)
{
    uint32_t t = startMs;
    for (int i = 0; i < count; ++i)
    {
        const float altM = startAltM + rateMS * (static_cast<float>(i * dtMs) / 1000.0f);
        (void)kf.update(altM, t);
        t += dtMs;
    }
    return t - dtMs;
}

}  // namespace


// ---------------------------------------------------------
// Normal behaviour
// ---------------------------------------------------------

TEST(VarioKalman, StartsUninitialised)
{
    VarioKalman kf;

    EXPECT_FALSE(kf.initialised());
    EXPECT_FLOAT_EQ(kf.climbRateMS(), 0.0f);
}

TEST(VarioKalman, FirstSampleInitialisesAltitudeWithZeroRate)
{
    VarioKalman kf;

    EXPECT_TRUE(kf.update(2500.0f, 10000));

    EXPECT_TRUE(kf.initialised());
    EXPECT_NEAR(kf.altitudeM(), 2500.0f, 0.001f);
    EXPECT_FLOAT_EQ(kf.climbRateMS(), 0.0f);
}

TEST(VarioKalman, ConstantAltitudeGivesZeroRate)
{
    VarioKalman kf;

    feedLinearClimb(kf, 1000.0f, 10000, 0.0f, BARO_SAMPLE_MS, 50);

    EXPECT_NEAR(kf.climbRateMS(), 0.0f, 0.01f);
}

TEST(VarioKalman, SteadyClimbConvergesToTrueRate)
{
    VarioKalman kf;

    // 3 s of a clean 2 m/s climb.
    feedLinearClimb(kf, 1000.0f, 10000, 2.0f, BARO_SAMPLE_MS, 30);

    EXPECT_NEAR(kf.climbRateMS(), 2.0f, 0.05f);
}

TEST(VarioKalman, SteadyDescentConvergesToNegativeRate)
{
    VarioKalman kf;

    feedLinearClimb(kf, 1000.0f, 10000, -3.0f, BARO_SAMPLE_MS, 30);

    EXPECT_NEAR(kf.climbRateMS(), -3.0f, 0.05f);
}

TEST(VarioKalman, RealisticFlightAltitudeAndMillisValue)
{
    // ~2500 m and a large millis() value: guards against float precision
    // loss in the altitude / covariance arithmetic.
    VarioKalman kf;

    feedLinearClimb(kf, 2500.0f, 3600000UL, 3.0f, BARO_SAMPLE_MS, 40);

    EXPECT_NEAR(kf.climbRateMS(), 3.0f, 0.1f);
}

TEST(VarioKalman, IrregularSampleSpacingStillConverges)
{
    // loop() can be held up by the display or the SD card, so samples are
    // not always 100 ms apart. Spacing here cycles 100 / 250 / 80 / 400 ms.
    VarioKalman kf;
    const uint32_t spacingMs[] = {100, 250, 80, 400};
    uint32_t t = 10000;
    float elapsedS = 0.0f;

    for (int i = 0; i < 60; ++i)
    {
        (void)kf.update(1000.0f + 2.0f * elapsedS, t);
        const uint32_t dt = spacingMs[i % 4];
        t += dt;
        elapsedS += static_cast<float>(dt) / 1000.0f;
    }

    EXPECT_NEAR(kf.climbRateMS(), 2.0f, 0.1f);
}

TEST(VarioKalman, FollowsAChangeInClimbRate)
{
    // Sinking, then lift: the estimate must follow within a couple of seconds.
    VarioKalman kf;

    const uint32_t tLast = feedLinearClimb(kf, 1000.0f, 10000, -1.0f, BARO_SAMPLE_MS, 30);
    const float altAtSwitch = 1000.0f - 1.0f * (29 * BARO_SAMPLE_MS / 1000.0f);
    feedLinearClimb(kf, altAtSwitch, tLast + BARO_SAMPLE_MS, 2.0f, BARO_SAMPLE_MS, 20);

    EXPECT_NEAR(kf.climbRateMS(), 2.0f, 0.2f);
}


// ---------------------------------------------------------
// Rough air (regression: the gate must not reject real motion)
// ---------------------------------------------------------

namespace
{

// Number of samples the filter gated out while following a noise-free,
// physically real vertical-speed profile: 1.5 m/s mean, plus a 3 s and a
// 1.1 s sinusoid of the given amplitudes (m/s), plus optionally an
// instantaneous +/-3 m/s step every 10 s (a sharp thermal edge). Altitude is
// integrated finely so every sample is exact: anything rejected is real motion.
uint32_t rejectsFollowingRealMotion(float amp3sMS, float amp1p1sMS, bool sharpEdges)
{
    VarioKalman kf;
    double altM = 1000.0;
    uint32_t t = 10000;

    for (int i = 0; i < 600; ++i)
    {
        const double tS = i * 0.1;
        for (int k = 0; k < 10; ++k)
        {
            const double tm = tS + (k + 0.5) * 0.01;
            double vs = 1.5 + amp3sMS * std::sin(2.0 * M_PI * tm / 3.0)
                            + amp1p1sMS * std::sin(2.0 * M_PI * tm / 1.1);
            if (sharpEdges && std::fmod(tm, 20.0) > 10.0) vs += 3.0;
            altM += vs * 0.01;
        }
        (void)kf.update(static_cast<float>(altM), t);
        t += BARO_SAMPLE_MS;
    }
    return kf.rejectedCount();
}

}  // namespace

TEST(VarioKalman, ModerateTurbulenceIsNotRejected)
{
    // About 4 m/s^2 peak acceleration.
    EXPECT_EQ(rejectsFollowingRealMotion(2.0f, 1.0f, false), 0u);
}

TEST(VarioKalman, SevereTurbulenceIsNotRejectedAsOutliers)
{
    // Roughly 15 m/s^2 peak, far beyond the filter's own acceleration model.
    // A gate based on the filter's sigma alone threw away about one real
    // sample in six here and then coasted on stale data; the absolute
    // VARIO_KF_MIN_GATE_M floor prevents that.
    EXPECT_EQ(rejectsFollowingRealMotion(3.0f, 1.5f, false), 0u);
}

TEST(VarioKalman, SharpThermalEdgeIsNotRejected)
{
    // +/-3 m/s changes with no ramp: the sigma gate alone dropped samples
    // at every edge.
    EXPECT_EQ(rejectsFollowingRealMotion(2.0f, 1.0f, true), 0u);
}


// ---------------------------------------------------------
// Outlier handling
// ---------------------------------------------------------

TEST(VarioKalman, SingleLargeGlitchIsRejected)
{
    VarioKalman kf;
    const uint32_t tLast = feedLinearClimb(kf, 1000.0f, 10000, 0.0f, BARO_SAMPLE_MS, 40);

    // One corrupted read, 3 m off (the old +/-15 m/s gate let up to 1.5 m through).
    const bool used = kf.update(1003.0f, tLast + BARO_SAMPLE_MS);

    EXPECT_FALSE(used);
    EXPECT_EQ(kf.rejectedCount(), 1u);
    EXPECT_NEAR(kf.climbRateMS(), 0.0f, 0.05f);
}

TEST(VarioKalman, FilterRecoversNormallyAfterAGlitch)
{
    VarioKalman kf;
    uint32_t t = feedLinearClimb(kf, 1000.0f, 10000, 0.0f, BARO_SAMPLE_MS, 40);

    (void)kf.update(1003.0f, t + BARO_SAMPLE_MS);  // glitch
    feedLinearClimb(kf, 1000.0f, t + 2 * BARO_SAMPLE_MS, 0.0f, BARO_SAMPLE_MS, 20);

    EXPECT_NEAR(kf.climbRateMS(), 0.0f, 0.05f);
    EXPECT_NEAR(kf.altitudeM(), 1000.0f, 0.1f);
}

TEST(VarioKalman, RealStepIsAcceptedAfterAtMostTheRejectLimit)
{
    // The altitude genuinely jumps 10 m and stays there. Samples may be
    // gated for VARIO_KF_MAX_CONSECUTIVE_REJECTS, after which the filter
    // must follow the new altitude instead of rejecting forever.
    VarioKalman kf;
    uint32_t t = feedLinearClimb(kf, 1000.0f, 10000, 0.0f, BARO_SAMPLE_MS, 40);

    for (int i = 1; i <= 10; ++i)
    {
        (void)kf.update(1010.0f, t + i * BARO_SAMPLE_MS);
    }

    EXPECT_NEAR(kf.altitudeM(), 1010.0f, 0.5f);
    EXPECT_LE(kf.rejectedCount(), static_cast<uint32_t>(VARIO_KF_MAX_CONSECUTIVE_REJECTS) + 1u);
}


// ---------------------------------------------------------
// Degenerate input
// ---------------------------------------------------------

TEST(VarioKalman, DuplicateTimestampIsIgnored)
{
    VarioKalman kf;
    const uint32_t tLast = feedLinearClimb(kf, 1000.0f, 10000, 1.0f, BARO_SAMPLE_MS, 20);
    const float altBefore = kf.altitudeM();
    const float rateBefore = kf.climbRateMS();

    EXPECT_FALSE(kf.update(1500.0f, tLast));

    EXPECT_FLOAT_EQ(kf.altitudeM(), altBefore);
    EXPECT_FLOAT_EQ(kf.climbRateMS(), rateBefore);
}

TEST(VarioKalman, NonFiniteAltitudeIsIgnored)
{
    VarioKalman kf;
    const uint32_t tLast = feedLinearClimb(kf, 1000.0f, 10000, 1.0f, BARO_SAMPLE_MS, 20);
    const float altBefore = kf.altitudeM();

    EXPECT_FALSE(kf.update(NAN, tLast + BARO_SAMPLE_MS));
    EXPECT_FALSE(kf.update(INFINITY, tLast + 2 * BARO_SAMPLE_MS));

    EXPECT_FLOAT_EQ(kf.altitudeM(), altBefore);
    EXPECT_TRUE(std::isfinite(kf.climbRateMS()));
}

TEST(VarioKalman, NonFiniteFirstSampleLeavesFilterUninitialised)
{
    VarioKalman kf;

    EXPECT_FALSE(kf.update(NAN, 10000));

    EXPECT_FALSE(kf.initialised());
}

TEST(VarioKalman, LongGapRestartsTheFilter)
{
    // After more than VARIO_KF_MAX_GAP_MS with no samples, the old vertical
    // speed is meaningless: restart on the new altitude with a zero rate.
    VarioKalman kf;
    const uint32_t tLast = feedLinearClimb(kf, 1000.0f, 10000, 3.0f, BARO_SAMPLE_MS, 30);

    (void)kf.update(1400.0f, tLast + VARIO_KF_MAX_GAP_MS + 1000);

    EXPECT_NEAR(kf.altitudeM(), 1400.0f, 0.001f);
    EXPECT_FLOAT_EQ(kf.climbRateMS(), 0.0f);
}

TEST(VarioKalman, ResetClearsAllState)
{
    VarioKalman kf;
    feedLinearClimb(kf, 1000.0f, 10000, 2.0f, BARO_SAMPLE_MS, 30);

    kf.reset();

    EXPECT_FALSE(kf.initialised());
    EXPECT_FLOAT_EQ(kf.climbRateMS(), 0.0f);
    EXPECT_EQ(kf.rejectedCount(), 0u);
}

TEST(VarioKalman, SurvivesMillisWrapAround)
{
    // millis() rolls over after ~49.7 days. Start 500 ms before the wrap.
    VarioKalman kf;

    feedLinearClimb(kf, 1000.0f, 0xFFFFFFFFu - 500u, 2.0f, BARO_SAMPLE_MS, 30);

    EXPECT_NEAR(kf.climbRateMS(), 2.0f, 0.05f);
    EXPECT_EQ(kf.rejectedCount(), 0u);
}


// ---------------------------------------------------------
// Diagnostics wiring (through the real updateVario())
// ---------------------------------------------------------

TEST(VarioDiagnostics, FailedBaroReadsAreCounted)
{
    // The host stub's performReading() always fails, so each call to the
    // real updateVario() must be counted as one read failure.
    resetVarioDiagnostics();

    updateVario();
    updateVario();
    updateVario();

    EXPECT_EQ(varioDiagnostics().readFailures, 3u);
    EXPECT_EQ(varioDiagnostics().samples, 0u);
}

TEST(VarioDiagnostics, ResetClearsCounters)
{
    updateVario();

    resetVarioDiagnostics();

    EXPECT_EQ(varioDiagnostics().readFailures, 0u);
    EXPECT_EQ(varioDiagnostics().lateSamples, 0u);
    EXPECT_EQ(varioDiagnostics().maxGapMs, 0u);
}
