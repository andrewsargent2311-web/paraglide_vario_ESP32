# AI Assistant Ruleset — GoogleTest for C++ / PlatformIO

## Purpose

Use this ruleset when adding **host-side GoogleTest unit tests** to an existing C++ project, particularly a PlatformIO ESP32 project.

The goal is simple:

> Add tests as separate C++ files that exercise the existing production code, without unnecessarily restructuring or duplicating the production code.

This ruleset covers **GoogleTest unit testing only**.

Out of scope for now:
- static analysis
- clang-tidy
- GCC warning configuration
- sanitizers
- code coverage
- CI/CD
- hardware/ESP32 test execution
- performance testing
- fuzz testing
- runtime assertion strategy

---

## Core principles

### 1. Production code and test code are separate

Never put GoogleTest code into the production `.cpp` files.

Typical structure:

```text
project/
├── platformio.ini
├── paraglide_vario_ESP32/
│   ├── main.cpp
│   ├── Vario.cpp
│   └── Vario.h
│
└── test/
    └── test_vario/
        └── test_vario.cpp
```

The test file is a **consumer of the production component**.

Do not create a second copy of the production implementation for testing.

---

### 2. Do not refactor production code merely to make testing easier

The existing architecture is the source of truth.

Do not:
- split a `.cpp` into additional `.cpp` files just because a function is testable
- move functions into new "Math", "Utils", or "Test" libraries without being asked
- rename public functions or variables
- change APIs solely to fit GoogleTest
- rewrite working code into a different architecture
- duplicate production algorithms inside the tests

A function being easy to test does **not** automatically mean it deserves to become its own component.

The rule is:

> Test the existing component first. Refactor only when the user explicitly asks for refactoring.

---

### 3. Tests must exercise the real production code

A test must call the actual production function/class being tested.

Good:

```cpp
TEST(Vario, CalculatesSteadyClimb)
{
    // Arrange
    ...

    // Act
    float result = computeClimbRateLeastSquares();

    // Assert
    EXPECT_NEAR(result, 2.0f, 0.01f);
}
```

Bad:

```cpp
TEST(Vario, CalculatesSteadyClimb)
{
    // Re-implement the same calculation here.
    float expected = (108.0f - 100.0f) / 4.0f;
    ...
}
```

The test can calculate an expected value where appropriate, but it must never reproduce the implementation it is supposed to verify.

---

## Workflow when files are provided

When the user gives source files, follow this process.

### Step 1 — Inspect the supplied files

Read the relevant `.h` and `.cpp` files before writing tests.

Identify:
- public functions/classes
- public inputs and outputs
- important constants
- global state
- dependencies
- hardware dependencies
- functions that can execute on the host
- functions that are tightly coupled to hardware

Do not assume symbols, types, constants, or interfaces that are not present in the supplied files.

### Step 2 — Identify the testable surface

Prioritize functions that have deterministic inputs and outputs.

Examples:

```text
input → calculation → result
input → parser → structure
input → state transition → state
samples → filter → filtered value
```

A function that talks directly to an ESP32 peripheral may not be suitable for the initial native test suite.

That does not mean it should immediately be refactored.

### Step 3 — Decide what tests belong in the first test file

Prefer a small, useful test suite over dozens of trivial tests.

Normally cover:

1. normal/expected behaviour
2. important boundary values
3. important invalid or degenerate input
4. a representative regression case where there is known behaviour worth protecting

Only add cases that are meaningful for the actual code.

### Step 4 — Create the test file

Use PlatformIO's GoogleTest layout:

```text
test/
└── test_<component>/
    └── test_<component>.cpp
```

Keep the test source separate from production source.

### Step 5 — Compile and run the native tests

The intended test environment is:

```ini
[env:native]
platform = native
test_framework = googletest
```

Run:

```bash
pio test -e native
```

Do not claim tests pass unless they were actually run successfully.

If the native build fails, report the actual blocking dependency or compiler error rather than inventing a workaround.

---

## GoogleTest conventions

### Test naming

Use:

```cpp
TEST(ComponentName, BehaviourBeingTested)
```

Examples:

```cpp
TEST(VarioClimbRate, ConstantAltitudeIsZero)
TEST(VarioClimbRate, SteadyClimbProducesExpectedRate)
TEST(VarioClimbRate, SteadyDescentProducesNegativeRate)
```

The name should describe **behaviour**, not implementation details.

---

### Arrange / Act / Assert

Prefer this structure:

```cpp
TEST(Vario, SteadyClimb)
{
    // Arrange
    ...

    // Act
    const float result = ...;

    // Assert
    EXPECT_NEAR(result, 2.0f, 0.01f);
}
```

Do not add comments when the structure is already obvious, but use the pattern consistently for non-trivial tests.

---

### EXPECT vs ASSERT

Use `EXPECT_*` when later assertions can still provide useful information.

Use `ASSERT_*` when failure means continuing the test would be invalid.

Example:

```cpp
ASSERT_TRUE(object.isValid());

EXPECT_EQ(object.value(), 42);
```

---

### Floating-point comparisons

Do not use exact equality for calculations involving floating-point arithmetic unless exact equality is specifically guaranteed.

Prefer:

```cpp
EXPECT_NEAR(actual, expected, tolerance);
```

Example:

```cpp
EXPECT_NEAR(result, 2.0f, 0.01f);
```

Choose tolerances based on the actual algorithm and expected precision. Do not use an arbitrarily tiny tolerance just to make a test look strict.

---

## Test fixtures

Use a fixture only when multiple tests genuinely share setup.

Simple tests should remain simple:

```cpp
TEST(Vario, ...)
```

Use a fixture when appropriate:

```cpp
class VarioTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        ...
    }
};

TEST_F(VarioTest, ...)
{
    ...
}
```

Do not introduce fixtures, helper classes, or abstractions just for style.

---

## Test-side helpers

Test helper functions are allowed.

For example:

```cpp
static void resetTestState()
{
    ...
}
```

A helper may prepare test data or reset state, but it must not duplicate the production algorithm.

Good:

```cpp
static void addSample(float altitude, unsigned long time)
{
    ...
}
```

Bad:

```cpp
static float calculateExpectedClimbRate(...)
{
    // Reimplementation of the production regression algorithm.
}
```

---

## Testing global state

Existing code may use globals.

Do not immediately redesign the production code because of this.

Where the existing API permits it, tests may:
- reset test state
- populate exposed test inputs
- call the real production function
- inspect the real output

For example:

```cpp
extern float altWindow[];
extern unsigned long timeWindow[];
extern int windowCount;
extern int windowIndex;
```

may be used by a test **when those symbols are genuinely part of the existing implementation and there is no cleaner existing interface**.

However, treat this as a characteristic of the current code, not as a reason to spread global access throughout future components.

---

## Hardware dependencies

The native test suite runs on the development computer.

Do not require:
- an ESP32
- BMP/BME/SHT sensors
- GPS hardware
- I2C
- SPI
- UART hardware
- Bluetooth hardware
- display hardware
- GPIO

A component that directly depends on hardware may therefore require a test seam, fake, or existing abstraction.

Do not invent a large mocking framework unless the supplied code actually requires it.

When a component cannot reasonably be tested natively in its current form:

1. identify the exact dependency preventing the test
2. test the hardware-independent behaviour that is already accessible
3. clearly identify what remains untested
4. do not silently rewrite production architecture

---

## What makes a useful unit test?

A useful test should fail when the production code is wrong.

For a calculation, test cases might include:

```text
normal positive result
normal negative result
zero result
boundary/limit
empty input
single input
degenerate input
representative noisy input
```

Do not blindly create every conceivable permutation.

Aim for tests that protect actual behaviour.

---

## Avoid brittle tests

Tests should verify externally meaningful behaviour rather than incidental implementation details.

Prefer:

```cpp
EXPECT_NEAR(calculateClimbRate(...), 2.0f, 0.01f);
```

over checking every internal accumulator:

```cpp
EXPECT_FLOAT_EQ(sumT, ...);
EXPECT_FLOAT_EQ(sumTA, ...);
```

unless those internals are themselves part of the intended contract.

The test should survive a reasonable internal refactor that preserves behaviour.

---

## Regression tests

When the user provides a bug, known failure, or previously observed incorrect result, create a test that reproduces the issue first where practical.

The desired progression is:

```text
known bug
    ↓
test demonstrates failure
    ↓
production fix
    ↓
test passes
```

Keep that test permanently as protection against regression.

---

## Do not hide failures

Never:
- weaken a test just to make it pass
- remove a failing test without explanation
- change expected values to match the current implementation without checking the intended behaviour
- silently skip a test because it exposes a bug

When expected behaviour is ambiguous, identify the ambiguity rather than inventing an answer.

---

## Output expectations

When asked to add tests, provide:

1. the test file(s) to create or modify
2. any minimal PlatformIO configuration required
3. the command used to run the tests
4. a brief explanation of what the tests cover
5. any test limitations caused by hardware coupling or missing source files

Do not make unrelated production-code changes.

---

## Default decision rule

When in doubt, prefer:

> **Smallest change to production code + separate GoogleTest file + tests against observable behaviour.**

The primary objective is to establish a reliable safety net around the existing codebase, not to redesign the codebase.

## No invented interfaces

Every symbol used in a generated test must come from one of:

- a supplied production source/header file
- GoogleTest
- the C++ standard library
- an explicitly documented test helper created by the assistant

If a required symbol cannot be verified from the supplied files, do not invent it.
State which file is required.

## Mandatory pre-test analysis

Before writing any test code:

1. Inspect every header and source file supplied by the user.
2. Identify the exact types of every dependency used by the production code.
3. Never invent a class, typedef, object, function, or API.
4. Never assume a production object can be replaced by a mock object.
5. Determine how the production component can actually be linked into the native test.
6. If native compilation is impossible with the supplied files, STOP and explain exactly why.
7. Do not create mock hardware classes unless the existing production code uses an interface/abstraction that supports substitution.
8. Do not modify production code merely to accommodate a test unless explicitly authorised.
9. Before generating tests, provide a short "Testability Analysis" identifying:
   - functions that can be tested natively
   - functions that cannot
   - required production files
   - external dependencies
   - how the test will call the real production code
10. Only then generate the test files.
