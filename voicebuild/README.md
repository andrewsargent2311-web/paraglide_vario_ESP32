# voicebuild

Generates a voice profile (the 60 alert clips `V000.PCM`..`V059.PCM`) with
[Piper](https://github.com/rhasspy/piper) neural TTS. See the docstring at the
top of `build_voice_profile.py` for all options.

## Make a US-accent profile

```bash
python -m venv .venv && source .venv/bin/activate      # Windows: .venv\Scripts\activate
pip install -r voicebuild/requirements.txt

# download a clear US voice (internet needed)
python -m piper.download_voices en_US-ryan-high --data-dir voicebuild/voices

python voicebuild/build_voice_profile.py \
    --profile US_RYAN \
    --model en_US-ryan-high \
    --data-dir voicebuild/voices \
    --match-loudness "paraglide_vario_ESP32/Goes into SD card/VOICE"
```

Output goes to `paraglide_vario_ESP32/Goes into SD card/VOICE/US_RYAN/`. Listen to
`US_RYAN_preview.wav` (written next to it) before copying anything to the SD card.

Good US voices to try (compare them on the Piper samples page first):
`en_US-ryan-high` (male, crisp), `en_US-lessac-high` (female/neutral, very clear),
`en_US-amy-medium`, `en_US-hfc_female-medium`. Check each voice's licence before
committing its clips to a public repo.

If a word sounds wrong, add a line to a TSV (`<id><TAB><respelled text>`) and
re-run with `--overrides fix.tsv --only <id>`.
