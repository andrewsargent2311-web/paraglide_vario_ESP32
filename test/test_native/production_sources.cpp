// Production code under test for the native environment.
//
// env:native builds everything under test/test_native and none of the
// project's src_dir, so main.cpp and the hardware-only files never enter
// the build. Each production file the tests need is pulled in here with
// one #include. (-I$PROJECT_SRC_DIR in platformio.ini makes these resolve.)
//
// The production files are compiled as-is: nothing is copied or edited.
// When a new component gets tests, add its .cpp on a new line below.
//
// Note: these all compile as ONE translation unit, so a file-local symbol
// or macro in one production file can clash with another. If that happens
// it shows up as a compile error, not a silent bug.

#include "Vario.cpp"
#include "Gps.cpp"
#include "Utils.cpp"
#include "OpenAirScanner.cpp"
#include "SdCard.cpp"
#include "settings.cpp"
#include "IgcRecorder.cpp"
