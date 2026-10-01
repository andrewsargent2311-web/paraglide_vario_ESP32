#include <gtest/gtest.h>

#include <initializer_list>

// Declares computeClimbRateLeastSquares(), CLIMB_WINDOW_N and the
// altWindow / timeWindow / windowCount / windowIndex globals.
#include "Vario.h"

namespace
{

struct Sample
{
    float         altitudeM;
    unsigned long timeMs;
};

// Append one sample to the ring buffer.
// Vario.cpp exposes no API for this (updateVario() needs a live BMP580),
// so the test writes the globals directly. This only prepares input data;
// it does not reimplement the regression.
void pushSample(const Sample& s)
{
    altWindow[windowIndex]  = s.altitudeM;
    timeWindow[windowIndex] = s.timeMs;

    windowIndex = (windowIndex + 1) % CLIMB_WINDOW_N;

    if (windowCount < CLIMB_WINDOW_N)
    {
        ++windowCount;
    }
}

void loadSamples(std::initializer_list<Sample> samples)
{
    for (const Sample& s : samples)
    {
        pushSample(s);
    }
}

} // namespace


class VarioClimbRate : public ::testing::Test
{
protected:
    void SetUp() override
    {
        for (int i = 0; i < CLIMB_WINDOW_N; ++i)
        {
            altWindow[i]  = 0.0f;
            timeWindow[i] = 0;
        }

        windowCount = 0;
        windowIndex = 0;
    }
};


// ---------------------------------------------------------
// Normal behaviour
// ---------------------------------------------------------

TEST_F(VarioClimbRate, ConstantAltitudeIsZero)
{
    loadSamples({
        {100.0f, 0}, {100.0f, 1000}, {100.0f, 2000}, {100.0f, 3000}, {100.0f, 4000}
    });

    EXPECT_NEAR(computeClimbRateLeastSquares(), 0.0f, 0.001f);
}

TEST_F(VarioClimbRate, SteadyClimbProducesPositiveRate)
{
    // 2 m every second.
    loadSamples({
        {100.0f, 0}, {102.0f, 1000}, {104.0f, 2000}, {106.0f, 3000}, {108.0f, 4000}
    });

    EXPECT_NEAR(computeClimbRateLeastSquares(), 2.0f, 0.001f);
}

TEST_F(VarioClimbRate, SteadyDescentProducesNegativeRate)
{
    loadSamples({
        {108.0f, 0}, {106.0f, 1000}, {104.0f, 2000}, {102.0f, 3000}, {100.0f, 4000}
    });

    EXPECT_NEAR(computeClimbRateLeastSquares(), -2.0f, 0.001f);
}

TEST_F(VarioClimbRate, TwoSamplesGiveTheirSlope)
{
    loadSamples({
        {100.0f, 0}, {103.0f, 1000}
    });

    EXPECT_NEAR(computeClimbRateLeastSquares(), 3.0f, 0.001f);
}

TEST_F(VarioClimbRate, UsesAllSamplesNotJustEndpoints)
{
    // Flat, then a jump on the last sample.
    // A "last minus first" shortcut would give (108 - 100) / 4 = 2.0 m/s.
    // The least-squares fit through all five points is 1.6 m/s
    // (hand-calculated: (5*1032 - 10*508) / (5*30 - 10*10) = 80 / 50).
    loadSamples({
        {100.0f, 0}, {100.0f, 1000}, {100.0f, 2000}, {100.0f, 3000}, {108.0f, 4000}
    });

    EXPECT_NEAR(computeClimbRateLeastSquares(), 1.6f, 0.001f);
}

TEST_F(VarioClimbRate, NoisyClimbStillProducesCorrectTrend)
{
    // Roughly 1 m/s with +/- 0.2 m of noise.
    // Hand-calculated least-squares slope: 1.01 m/s.
    loadSamples({
        {100.0f, 0}, {101.1f, 1000}, {101.9f, 2000}, {103.2f, 3000}, {104.0f, 4000}
    });

    EXPECT_NEAR(computeClimbRateLeastSquares(), 1.01f, 0.01f);
}

TEST_F(VarioClimbRate, RealisticFlightSamplingAtAltitude)
{
    // Production conditions: BARO_SAMPLE_MS (100 ms) spacing, a full window
    // of CLIMB_WINDOW_N samples, ~2500 m altitude, and a large millis()
    // value. Climbing at 3 m/s = 0.3 m per sample.
    // Guards against float precision loss in the regression sums.
    const unsigned long startMs = 3600000UL;

    for (int i = 0; i < CLIMB_WINDOW_N; ++i)
    {
        pushSample({2500.0f + 0.3f * static_cast<float>(i),
                    startMs + static_cast<unsigned long>(i) * BARO_SAMPLE_MS});
    }

    EXPECT_NEAR(computeClimbRateLeastSquares(), 3.0f, 0.1f);
}


// ---------------------------------------------------------
// Degenerate input
// ---------------------------------------------------------

TEST_F(VarioClimbRate, EmptyWindowReturnsZero)
{
    EXPECT_FLOAT_EQ(computeClimbRateLeastSquares(), 0.0f);
}

TEST_F(VarioClimbRate, SingleSampleReturnsZero)
{
    loadSamples({
        {100.0f, 1000}
    });

    EXPECT_FLOAT_EQ(computeClimbRateLeastSquares(), 0.0f);
}

TEST_F(VarioClimbRate, IdenticalTimestampsReturnZero)
{
    loadSamples({
        {100.0f, 1000}, {102.0f, 1000}, {104.0f, 1000}
    });

    EXPECT_FLOAT_EQ(computeClimbRateLeastSquares(), 0.0f);
}


// ---------------------------------------------------------
// Circular-buffer behaviour
// ---------------------------------------------------------

TEST_F(VarioClimbRate, FullWindowAfterWrapAroundUsesOnlyNewestSamples)
{
    // Two wildly wrong samples go in first. Pushing CLIMB_WINDOW_N more
    // samples overwrites them and wraps windowIndex. The result must reflect
    // only the newest CLIMB_WINDOW_N samples (a clean 2 m/s climb), so any
    // stale data or wrong oldest-sample index would show up as a bad slope.
    loadSamples({
        {5000.0f, 0}, {-5000.0f, 500}
    });

    for (int i = 0; i < CLIMB_WINDOW_N; ++i)
    {
        pushSample({100.0f + 2.0f * static_cast<float>(i),
                    1000UL + static_cast<unsigned long>(i) * 1000UL});
    }

    ASSERT_EQ(windowCount, CLIMB_WINDOW_N);
    ASSERT_EQ(windowIndex, 2);   // wrapped past the end of the buffer

    EXPECT_NEAR(computeClimbRateLeastSquares(), 2.0f, 0.001f);
}

// ---------------------------------------------------------
// Averaged display climb rate (updateClimbRateAverage)
// ---------------------------------------------------------

class VarioClimbRateAverage : public ::testing::Test
{
protected:
    void SetUp() override { resetClimbRateAverage(); }
};

TEST_F(VarioClimbRateAverage, SingleSampleIsItself)
{
    updateClimbRateAverage(2.5f, 10000);

    EXPECT_FLOAT_EQ(currentClimbRateAvgMS, 2.5f);
}

TEST_F(VarioClimbRateAverage, SteadyRateAveragesToThatRate)
{
    for (int i = 0; i < 30; ++i)
    {
        updateClimbRateAverage(1.2f, 10000UL + static_cast<unsigned long>(i) * BARO_SAMPLE_MS);
    }

    EXPECT_NEAR(currentClimbRateAvgMS, 1.2f, 0.001f);
}

TEST_F(VarioClimbRateAverage, AlternatingNoiseIsSmoothedOut)
{
    // Fast value flips between +1 and -1 every sample (a bouncing vario).
    // The averaged value must stay close to zero.
    for (int i = 0; i < 40; ++i)
    {
        updateClimbRateAverage((i % 2 == 0) ? 1.0f : -1.0f,
                               10000UL + static_cast<unsigned long>(i) * BARO_SAMPLE_MS);
    }

    EXPECT_NEAR(currentClimbRateAvgMS, 0.0f, 0.1f);
}

TEST_F(VarioClimbRateAverage, SamplesOlderThanTheWindowAreIgnored)
{
    // 3 s of 0 m/s, then 1 s of 4 m/s at 100 ms spacing. With a 1.5 s
    // window the old zeros must have dropped out of most of the average,
    // so it should be well above the 1.0 m/s a 4 s mean would give.
    unsigned long t = 10000;

    for (int i = 0; i < 30; ++i, t += BARO_SAMPLE_MS)
    {
        updateClimbRateAverage(0.0f, t);
    }
    for (int i = 0; i < 10; ++i, t += BARO_SAMPLE_MS)
    {
        updateClimbRateAverage(4.0f, t);
    }

    EXPECT_GT(currentClimbRateAvgMS, 1.5f);
    EXPECT_LT(currentClimbRateAvgMS, 4.0f);
}

TEST_F(VarioClimbRateAverage, StepResponseSettlesWithinTheWindow)
{
    // After DISPLAY_CLIMB_AVG_MS of a new steady rate, the average has
    // fully moved to it.
    unsigned long t = 10000;

    for (int i = 0; i < 20; ++i, t += BARO_SAMPLE_MS)
    {
        updateClimbRateAverage(-1.0f, t);
    }
    for (unsigned long elapsed = 0; elapsed <= DISPLAY_CLIMB_AVG_MS; elapsed += BARO_SAMPLE_MS, t += BARO_SAMPLE_MS)
    {
        updateClimbRateAverage(3.0f, t);
    }

    EXPECT_NEAR(currentClimbRateAvgMS, 3.0f, 0.3f);
}

TEST_F(VarioClimbRateAverage, SurvivesMillisWrapAround)
{
    // millis() rolls over after ~49.7 days; unsigned subtraction must cope.
    unsigned long t = 0xFFFFFFFFUL - 500UL;

    for (int i = 0; i < 10; ++i, t += BARO_SAMPLE_MS)
    {
        updateClimbRateAverage(2.0f, t);
    }

    EXPECT_NEAR(currentClimbRateAvgMS, 2.0f, 0.001f);
}
