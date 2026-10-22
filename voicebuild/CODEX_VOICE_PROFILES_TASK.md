# Codex task: selectable voice profiles for the ADS-B voice alerts (paraglide_vario_ESP32)

## Before you start
1. Read `skills/flightcomputer-power-of-ten.skill` (it is a zip: unzip it and read `SKILL.md` and `references/rules.md`) and follow it. In particular: no heap allocation after initialisation, bounded loops, bounded buffers, every return value checked.
2. Make only the changes listed here. Do not refactor unrelated code, rename existing public functions, or change the clip vocabulary: `VoiceClips.h` (60 clips, ids 0..59) must NOT change.
3. `voicebuild/` already exists in the repo (a Piper-based generator that produces profile folders). Do not modify it and do not generate or commit any `.PCM` files yourself.
4. After each task, confirm `pio run -e esp32-s3` compiles. At the end also run `pio test -e native` (existing tests must still pass; no new host tests are needed because `VoiceAlert.cpp` depends on SD_MMC/PSRAM).

## Goal
Let the user keep several voices on the SD card and pick one from the menu, persisted across reboots. The current single voice must keep working unchanged on existing cards.

## SD card layout (new)
```
/VOICE/V000.PCM ... V059.PCM          <- existing flat layout = profile "Default" (still supported)
/VOICE/US_RYAN/V000.PCM ... V059.PCM  <- new: one sub-folder per profile
/VOICE/ANOTHER/V000.PCM ...
```
- A profile is valid if its folder contains `V000.PCM` when discovered (full validation happens when loading).
- Profile folder names are limited to `[A-Za-z0-9_-]`, 1 to 12 characters. Ignore folders whose names don't match (this keeps path building safe). Max 8 profiles.
- Non-`.PCM` files (e.g. `manifest.txt`) are ignored.

## Facts about the existing code you must respect
- Clips are raw headerless 16 kHz mono s16le PCM at `/VOICE/V%03d.PCM`. `loadVoiceClips()` in `VoiceAlert.cpp` currently `ps_malloc`s one buffer per clip at boot and is called from `setup()` in `FlightComputer.cpp` after `SD_MMC.begin()`.
- `voiceServiceChunk()` is called from `i2sToneService()` in `Buzzer.cpp`, which runs in its OWN pinned FreeRTOS audio task (see `FlightComputer.cpp`, around `xTaskCreatePinnedToCore` near line 439), NOT from `loop()`. So the audio task reads the clip buffers concurrently with anything the menu or background task does. A reload must never free or overwrite clip memory while the audio task could be reading it.
- `voicePlaySequence()` is called from `loop()`/menu code on Core 1. `backgroundTask()` (`BackgroundTask.cpp`, Core 0) already does SD work and takes `sdMutex` (200 ms timeout) around SD access (see the DEM lookup there). The SD card is shared with the IGC logger, so every SD access at runtime must hold `sdMutex`.
- Settings persistence pattern: see `selectedDemFile` / the `"demFile"` Preferences key in `settings.cpp` / `settings.h`, and `loadSettings()` / `saveSettings()`.
- Menu pattern: `menu.cpp`. Under `MENU_SCREEN_ADSB_SETTINGS` item 3 ("Alarm Sound") cycles in place. The item count is returned by `getMenuItemCount()` (currently `return 8;` for ADSB_SETTINGS), labels are built in the `MENU_SCREEN_ADSB_SETTINGS` case near line 452, and selection is handled near line 761. The file-listing pattern is `scanMapFiles()`.

## Task 1: VoiceAlert.cpp / VoiceAlert.h: profile-aware loading with one fixed pool
Replace per-clip `ps_malloc` with ONE fixed PSRAM pool, allocated once the first time `loadVoiceClips()` runs at boot (so there is no allocation or fragmentation at runtime):
- `#define VOICE_POOL_BYTES (2u * 1024u * 1024u)` (existing voice is about 1.25 MB; check the README/hardware notes for the PSRAM size and keep this conservative). If the pool allocation fails, voice alerts are disabled exactly as today.
- `g_clips[id]` becomes a pointer into the pool plus a sample count.
- Add to `VoiceAlert.h`:
  ```cpp
  #define VOICE_PROFILE_NAME_MAX 13   // 12 chars + NUL
  #define VOICE_MAX_PROFILES 8
  // Profile "" (empty string) = the flat /VOICE/V###.PCM layout, shown as "Default".
  uint8_t listVoiceProfiles(char names[][VOICE_PROFILE_NAME_MAX], uint8_t maxNames); // takes sdMutex; returns count
  const char* voiceActiveProfile();          // name currently loaded ("" = default); valid only if voiceClipsLoaded()
  bool requestVoiceProfile(const char* name); // queues a background reload; returns false if one is already pending/running or the name is invalid
  bool voiceProfileLoading();                 // true while a reload is pending or running
  bool voiceLastReloadOk();                   // result of the most recent reload
  void voiceProfileService();                 // called from backgroundTask() each pass; performs a pending reload
  ```
- `loadVoiceClips()` (boot, synchronous, called before the background task exists) loads the profile named by the persisted setting (Task 2). Fallback order if that profile is missing or invalid: Default (flat) layout, then the first discovered valid profile, then voice disabled. Log the outcome to Serial (`[Voice] ...`), keeping the existing `Loaded 60 voice clips into PSRAM` style message and adding the profile name.
- Path building: `/VOICE/V%03d.PCM` for Default, `/VOICE/%s/V%03d.PCM` for a named profile, using bounded `snprintf` into a buffer sized for the longest case.
- Loading a profile (shared by boot and runtime reload) is two passes:
  1. **Validate, no changes to live state:** for all 60 ids, the file opens, size > 0, size is even, and the running total fits in `VOICE_POOL_BYTES`. If validation fails, reject the profile, leave the currently loaded voice untouched, log why.
  2. **Load:** read the clips into the pool back to back, recording each clip's offset and sample count.
- Runtime reload (`voiceProfileService()`, called from `backgroundTask()`):
  - Do the SD access with `sdMutex` held PER FILE (take with the same 200 ms timeout as the DEM lookup; retry a file a small bounded number of times, e.g. 3, then fail). Do not hold `sdMutex` across all 60 files, so the IGC logger is never starved.
  - Concurrency with the audio task (REQUIRED invariants): before touching the pool, stop playback and mark clips unavailable (`g_playing = false`, `g_clipsLoaded = false`) and make sure the audio task cannot be mid-read of the pool when you start overwriting it. Use a small FreeRTOS mutex or critical section around the clip reads in `voiceServiceChunk()`; that function must still never block: use a zero-timeout try-lock and output silence if it can't take it. Only set `g_clipsLoaded = true` again after the pool is completely loaded. Mark the shared flags `volatile` (or use atomics) as the existing code style allows.
  - If a read fails after validation passed (the pool is partly overwritten), try ONCE to reload the previously active profile; if that also fails, leave voice disabled (`voiceClipsLoaded()` false) and log it.
  - `voiceProfileLoading()` is true from the moment `requestVoiceProfile()` accepts a request until `voiceProfileService()` finishes it.
- `voicePlaySequence()` must keep its current behaviour (silent no-op if clips aren't loaded). Keep `voiceClipsLoaded()` semantics.

## Task 2: Settings persistence (settings.h / settings.cpp)
- Add `extern char voiceProfile[VOICE_PROFILE_NAME_MAX];` (default `""` = Default). Include whatever header is needed for the constant without creating an include cycle; if that's awkward, define `VOICE_PROFILE_NAME_MAX` in `settings.h` and have `VoiceAlert.h` use it.
- Load/save with a Preferences key `"voiceProf"` (<= 15 chars), copying the bounded `strncpy` + terminator pattern used for `selectedDemFile`. Validate on load: if the stored name fails the `[A-Za-z0-9_-]{0,12}` check, use `""`.
- The setting is only saved after a profile has loaded successfully (see Task 4), never on a failed attempt.

## Task 3: Boot hookup (FlightComputer.cpp)
- `loadVoiceClips()` stays where it is (after `SD_MMC.begin()`, `loadSettings()` having already run, so `voiceProfile` is valid). Confirm that `loadSettings()` runs before it; if not, report rather than reorder boot code.

## Task 4: Menu item (menu.h / menu.cpp)
- Add a new LAST item to `MENU_SCREEN_ADSB_SETTINGS` at index 8 (append, do not renumber existing items): label `Voice: <name>`, where an empty name is shown as `Default`, and while `voiceProfileLoading()` is true show `Voice: Loading...`. Change `getMenuItemCount()` for that screen from 8 to 9. Update the menu tree comment at the top of `menu.h` to mention it.
- On select: call `listVoiceProfiles()` (it takes `sdMutex` itself, with the same busy handling as `scanMapFiles()`: if the SD is busy, log and do nothing), find the currently active profile in the list, and request the NEXT one (wrapping). If there is only one profile, play the short "no change" feedback tone and do nothing. If `voiceProfileLoading()` is already true, ignore the press.
- When `voiceProfileLoading()` turns false and `voiceLastReloadOk()` is true, copy the new name into `voiceProfile` and call `saveSettings()`. Do this from an existing per-pass menu/loop hook on Core 1 (not from the background task) so `saveSettings()` and the settings globals are only touched from one core. If there is no suitable existing hook, add the smallest possible check in the menu update code and explain it in your summary. If the reload failed, leave `voiceProfile` unchanged and play a low error tone (`playFeedbackTone`).
- No automatic preview is needed: the existing **Alert Volume > Test** item already plays the example sentence with whichever profile is loaded.

## Task 5: Background task hookup (BackgroundTask.cpp)
- Call `voiceProfileService()` once per pass of the `backgroundTask()` loop, outside the interval-gated blocks, and only `if (sdCardOK)`. It must return immediately when no reload is pending.

## Task 6: README
Update `README.md`: in the SD card step, document the profile layout above (flat `VOICE/` = Default; `VOICE/<NAME>/` sub-folders for extra voices), the new `Voice:` item under ADSB Settings, that switching takes a couple of seconds, that all 60 clips must be present in a profile folder or it is skipped, and that new profiles are generated with `voicebuild/` (see `voicebuild/README.md`). Adjust the sentences that say the clips are always loaded from `/VOICE/V###.PCM`. Keep the existing style.

## Do not change (but mention in your summary)
- `triggerAdsbAlert()` in `Buzzer.cpp` silently does nothing in Voice mode if `voiceClipsLoaded()` is false. That now also covers the 1-2 s while a reload is in progress, or if a reload fails. Do NOT change this behaviour. In your summary, recommend whether it should fall back to the siren instead.

## Verify
- `pio run -e esp32-s3` compiles with no new warnings.
- `pio test -e native` still passes.
- Serial output on boot with the existing flat card still shows the voice loading (default profile), with no behaviour change.

## Manual checks for the user (on the device)
1. Existing card, only flat `/VOICE/`: boots, Alert Volume > Test speaks as before; menu shows `Voice: Default`; selecting it does nothing but the feedback tone.
2. Add `/VOICE/US_RYAN/` (60 clips): menu cycles Default -> US_RYAN -> Default; shows `Loading...` briefly; Test speaks with the new voice; the choice survives a reboot.
3. Remove one clip from `/VOICE/US_RYAN/`: selecting it is rejected with the error tone and the previous voice keeps working.
4. Switch voices during a flight-log recording: the IGC file keeps logging with no gaps.
5. Pull the SD card / corrupt a clip while on the new voice: device keeps running, voice alerts disabled, no crash.

## When done
Summarise changes per file, list any decisions you made (pool size, hook used in Task 4), and anything you noticed but did not change.
