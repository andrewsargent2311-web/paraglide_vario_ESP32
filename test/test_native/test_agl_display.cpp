#include <gtest/gtest.h>

#include "AglDisplay.h"

TEST(AglDisplayBuffer, HasNoReadingBeforeFirstValidSample)
{
    AglDisplayBuffer buffer;
    float displayedAglM = 0.0f;

    EXPECT_FALSE(buffer.update(false, 0.0f, 1000UL, displayedAglM));
}

TEST(AglDisplayBuffer, HoldsLastReadingForFiveSecondsAfterSourceLoss)
{
    AglDisplayBuffer buffer;
    float displayedAglM = 0.0f;

    ASSERT_TRUE(buffer.update(true, 125.0f, 1000UL, displayedAglM));
    EXPECT_FLOAT_EQ(displayedAglM, 125.0f);

    EXPECT_TRUE(buffer.update(false, 0.0f, 6000UL, displayedAglM));
    EXPECT_FLOAT_EQ(displayedAglM, 125.0f);
    EXPECT_FALSE(buffer.update(false, 0.0f, 6001UL, displayedAglM));
}

TEST(AglDisplayBuffer, ResumesWithFreshReadingAfterSourceRecovery)
{
    AglDisplayBuffer buffer;
    float displayedAglM = 0.0f;

    ASSERT_TRUE(buffer.update(true, 125.0f, 1000UL, displayedAglM));
    ASSERT_TRUE(buffer.update(false, 0.0f, 2000UL, displayedAglM));
    ASSERT_TRUE(buffer.update(true, 132.0f, 2500UL, displayedAglM));

    EXPECT_FLOAT_EQ(displayedAglM, 132.0f);
}

TEST(AglDisplayBuffer, HoldTimerHandlesMillisRollover)
{
    AglDisplayBuffer buffer;
    float displayedAglM = 0.0f;
    const unsigned long sampleAt = 0xFFFFFF00UL;

    ASSERT_TRUE(buffer.update(true, 125.0f, sampleAt, displayedAglM));
    EXPECT_TRUE(buffer.update(false, 0.0f, sampleAt + 5000UL, displayedAglM));
    EXPECT_FALSE(buffer.update(false, 0.0f, sampleAt + 5001UL, displayedAglM));
}
