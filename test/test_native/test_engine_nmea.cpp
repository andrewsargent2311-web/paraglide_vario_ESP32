#include <gtest/gtest.h>

#include <string.h>

#include "EngineNmea.h"

namespace {

const char* const kVerifiedSentences[] = {
    "$EMS,999999,9999,9999,9999,99,18000,55000,99,310000*7C",
    "$EMS,999999,9999,9999,9999,99,18000,99,99,310000*4C",
    "$EMS,999999,9999,9999,9999,99,99,99,99,0*47",
    "$EMS,101325,450,1200,250,2200,18000,55000,8500,310000*70",
    "$PEMBAT,85*2E",
};

struct ReadingCollector {
  EngineReading readings[8];
  size_t count;
  EngineNmeaCounters counters;
};

void collectLine(const char* line, size_t len, void* context) {
  ReadingCollector* collector = static_cast<ReadingCollector*>(context);
  EngineReading reading = {};
  if (parseEngineSentence(line, len, reading, collector->counters) &&
      collector->count < sizeof(collector->readings) / sizeof(collector->readings[0])) {
    collector->readings[collector->count++] = reading;
  }
}

void feedSentence(EngineNmeaLineAssembler& assembler, const char* sentence,
                  size_t chunkSize, ReadingCollector& collector, bool crlf = true) {
  uint8_t framed[EngineNmeaLineAssembler::kLineCapacity];
  const size_t sentenceLen = strlen(sentence);
  memcpy(framed, sentence, sentenceLen);
  size_t framedLen = sentenceLen;
  if (crlf) framed[framedLen++] = '\r';
  framed[framedLen++] = '\n';

  for (size_t offset = 0U; offset < framedLen;) {
    const size_t remaining = framedLen - offset;
    const size_t count = chunkSize < remaining ? chunkSize : remaining;
    assembler.feed(framed + offset, count, collectLine, &collector);
    offset += count;
  }
}

EngineReading parseValid(const char* sentence) {
  EngineReading reading = {};
  EXPECT_TRUE(parseEngineSentence(sentence, strlen(sentence), reading));
  return reading;
}

}  // namespace

TEST(EngineNmea, ParsesVerifiedMeterVectors) {
  const EngineReading first = parseValid(kVerifiedSentences[0]);
  EXPECT_TRUE(first.rpmUpdated);
  EXPECT_FALSE(first.rpmUnavailable);
  EXPECT_NEAR(first.rpm, 3100.0f, 0.01f);
  EXPECT_TRUE(first.chtUpdated);
  EXPECT_FALSE(first.chtFault);
  EXPECT_NEAR(first.chtC, 180.0f, 0.01f);
  EXPECT_TRUE(first.egtUpdated);
  EXPECT_FALSE(first.egtFault);
  EXPECT_NEAR(first.egtC, 550.0f, 0.01f);

  const EngineReading second = parseValid(kVerifiedSentences[1]);
  EXPECT_NEAR(second.chtC, 180.0f, 0.01f);
  EXPECT_FALSE(second.chtFault);
  EXPECT_TRUE(second.egtFault);
  EXPECT_FLOAT_EQ(second.egtC, 0.0f);
  EXPECT_NEAR(second.rpm, 3100.0f, 0.01f);

  const EngineReading third = parseValid(kVerifiedSentences[2]);
  EXPECT_TRUE(third.chtFault);
  EXPECT_TRUE(third.egtFault);
  EXPECT_FLOAT_EQ(third.chtC, 0.0f);
  EXPECT_FLOAT_EQ(third.egtC, 0.0f);
  EXPECT_FALSE(third.rpmUnavailable);
  EXPECT_FLOAT_EQ(third.rpm, 0.0f);

  const EngineReading fourth = parseValid(kVerifiedSentences[3]);
  EXPECT_NEAR(fourth.chtC, 180.0f, 0.01f);
  EXPECT_NEAR(fourth.egtC, 550.0f, 0.01f);
  EXPECT_NEAR(fourth.rpm, 3100.0f, 0.01f);
  EXPECT_FALSE(fourth.chtFault);
  EXPECT_FALSE(fourth.egtFault);

  const EngineReading fifth = parseValid(kVerifiedSentences[4]);
  EXPECT_TRUE(fifth.batteryUpdated);
  EXPECT_EQ(fifth.batteryPercent, 85U);
  EXPECT_FALSE(fifth.rpmUpdated);
}

TEST(EngineNmea, ReassemblesVerifiedVectorsAcrossNotificationChunkSizes) {
  const size_t chunkSizes[] = {1U, 7U, 20U};
  for (size_t sizeIndex = 0U; sizeIndex < sizeof(chunkSizes) / sizeof(chunkSizes[0]);
       ++sizeIndex) {
    EngineNmeaLineAssembler assembler;
    ReadingCollector collector = {};
    for (size_t sentenceIndex = 0U;
         sentenceIndex < sizeof(kVerifiedSentences) / sizeof(kVerifiedSentences[0]);
         ++sentenceIndex) {
      feedSentence(assembler, kVerifiedSentences[sentenceIndex], chunkSizes[sizeIndex],
                   collector);
    }
    ASSERT_EQ(collector.count, 5U);
    EXPECT_NEAR(collector.readings[0].rpm, 3100.0f, 0.01f);
    EXPECT_TRUE(collector.readings[1].egtFault);
    EXPECT_FLOAT_EQ(collector.readings[2].rpm, 0.0f);
    EXPECT_NEAR(collector.readings[3].chtC, 180.0f, 0.01f);
    EXPECT_EQ(collector.readings[4].batteryPercent, 85U);
  }
}

TEST(EngineNmea, EmitsBackToBackSentencesAndAcceptsCrLfOrBareLf) {
  EngineNmeaLineAssembler assembler;
  ReadingCollector collector = {};
  const char both[] =
      "$EMS,999999,9999,9999,9999,99,18000,55000,99,310000*7C\r\n"
      "$PEMBAT,85*2E\n";
  assembler.feed(reinterpret_cast<const uint8_t*>(both), sizeof(both) - 1U,
                 collectLine, &collector);
  ASSERT_EQ(collector.count, 2U);
  EXPECT_TRUE(collector.readings[0].rpmUpdated);
  EXPECT_TRUE(collector.readings[1].batteryUpdated);
}

TEST(EngineNmea, RejectsBadChecksumAndMalformedSentences) {
  EngineNmeaCounters counters = {};
  EngineReading reading = {};
  const char badChecksum[] = "$PEMBAT,85*2F";
  EXPECT_FALSE(parseEngineSentence(badChecksum, sizeof(badChecksum) - 1U, reading, counters));
  EXPECT_EQ(counters.checksumRejected, 1U);

  const char missingChecksum[] = "$PEMBAT,85";
  EXPECT_FALSE(parseEngineSentence(missingChecksum, sizeof(missingChecksum) - 1U,
                                   reading, counters));
  const char wrongFieldCount[] = "$EMS,1,2,3,4,5,6,7,8*53";
  EXPECT_FALSE(parseEngineSentence(wrongFieldCount, sizeof(wrongFieldCount) - 1U,
                                   reading, counters));
  const char nonNumericField[] = "$EMS,1,2,3,4,5,x,7,8,9*08";
  EXPECT_FALSE(parseEngineSentence(nonNumericField, sizeof(nonNumericField) - 1U,
                                   reading, counters));
  const char integerOverflow[] = "$EMS,2147483648,2,3,4,5,6,7,8,9*72";
  EXPECT_FALSE(parseEngineSentence(integerOverflow, sizeof(integerOverflow) - 1U,
                                   reading, counters));
  EXPECT_EQ(counters.malformedRejected, 4U);
}

TEST(EngineNmea, AcceptsLowercaseChecksumAndIgnoresUnknownIdentifiers) {
  EngineReading reading = {};
  const char lowercase[] = "$PEMBAT,85*2e";
  ASSERT_TRUE(parseEngineSentence(lowercase, sizeof(lowercase) - 1U, reading));
  EXPECT_EQ(reading.batteryPercent, 85U);

  EngineNmeaCounters counters = {};
  const char unknown[] = "$UNKNOWN,42*62";
  EXPECT_FALSE(parseEngineSentence(unknown, sizeof(unknown) - 1U, reading, counters));
  EXPECT_EQ(counters.checksumRejected, 0U);
  EXPECT_EQ(counters.malformedRejected, 0U);
}

TEST(EngineNmea, MapsUnavailableRpmAndTemperatureSentinels) {
  const char rpmUnavailable[] =
      "$EMS,999999,9999,9999,9999,99,99,99,99,9999*77";
  const EngineReading unavailable = parseValid(rpmUnavailable);
  EXPECT_TRUE(unavailable.rpmUpdated);
  EXPECT_TRUE(unavailable.rpmUnavailable);
  EXPECT_FLOAT_EQ(unavailable.rpm, 0.0f);

  const char realNinetyNine[] = "$EMS,1,2,3,4,5,9900,9900,8,100*4F";
  const EngineReading real = parseValid(realNinetyNine);
  EXPECT_FALSE(real.chtFault);
  EXPECT_FALSE(real.egtFault);
  EXPECT_NEAR(real.chtC, 99.0f, 0.01f);
  EXPECT_NEAR(real.egtC, 99.0f, 0.01f);

  const char negativeTemperature[] = "$EMS,1,2,3,4,5,9900,-500,8,100*57";
  const EngineReading negative = parseValid(negativeTemperature);
  EXPECT_NEAR(negative.chtC, 99.0f, 0.01f);
  EXPECT_NEAR(negative.egtC, -5.0f, 0.01f);
  EXPECT_FALSE(negative.egtFault);
}

TEST(EngineNmea, ResynchronizesAtDollarAndRecoversAfterOverflow) {
  EngineNmeaLineAssembler assembler;
  ReadingCollector collector = {};
  const char resync[] = "$EMS,partial$PEMBAT,85*2E\n";
  assembler.feed(reinterpret_cast<const uint8_t*>(resync), sizeof(resync) - 1U,
                 collectLine, &collector);
  ASSERT_EQ(collector.count, 1U);
  EXPECT_EQ(collector.readings[0].batteryPercent, 85U);

  uint8_t tooLong[99];
  tooLong[0] = '$';
  for (size_t i = 1U; i < 97U; ++i) tooLong[i] = 'A';
  tooLong[97] = '\n';
  tooLong[98] = '$';
  assembler.feed(tooLong, sizeof(tooLong), collectLine, &collector);
  const char recovery[] = "PEMBAT,85*2E\n";
  assembler.feed(reinterpret_cast<const uint8_t*>(recovery), sizeof(recovery) - 1U,
                 collectLine, &collector);
  EXPECT_EQ(assembler.overflowCount(), 1U);
  ASSERT_EQ(collector.count, 2U);
  EXPECT_EQ(collector.readings[1].batteryPercent, 85U);
}

TEST(EngineNmea, DropsLeadingGarbageAndHandlesBatteryBounds) {
  EngineNmeaLineAssembler assembler;
  ReadingCollector collector = {};
  const uint8_t prefix[] = {0x01U, 0xFFU, 'x', '\r'};
  assembler.feed(prefix, sizeof(prefix), collectLine, &collector);
  feedSentence(assembler, "$PEMBAT,0*13", 20U, collector);
  feedSentence(assembler, "$PEMBAT,100*12", 20U, collector, false);
  feedSentence(assembler, "$PEMBAT,101*13", 20U, collector);
  feedSentence(assembler, "$PEMBAT,-1*3F", 20U, collector);
  ASSERT_EQ(collector.count, 2U);
  EXPECT_EQ(collector.readings[0].batteryPercent, 0U);
  EXPECT_EQ(collector.readings[1].batteryPercent, 100U);
  EXPECT_EQ(collector.counters.malformedRejected, 2U);
}
