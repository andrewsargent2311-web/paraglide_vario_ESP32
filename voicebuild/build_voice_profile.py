#!/usr/bin/env python3
"""
build_voice_profile.py -- generate a complete set of voice-alert clips
(V000.PCM .. V059.PCM) for the paraglide vario using a Piper neural TTS voice.

Output format (must match VoiceAlert.cpp): raw, headerless, 16 kHz, mono,
signed 16-bit little-endian PCM, one file per clip, named V###.PCM where ###
is the VoiceClipId from VoiceClips.h.

The phrase for every clip comes from the existing manifest.txt, so the clip
count, order and wording stay identical to the current voice and
VoiceClips.h does NOT need to change.

QUICK START (run from the repo root)
------------------------------------
  pip install -r voicebuild/requirements.txt

  # Get a voice (needs internet). piper-tts 1.3 or newer:
  python -m piper.download_voices en_US-ryan-high --data-dir voicebuild/voices

  python voicebuild/build_voice_profile.py \
      --profile US_RYAN \
      --model en_US-ryan-high \
      --data-dir voicebuild/voices

  -> writes  paraglide_vario_ESP32/Goes into SD card/VOICE/US_RYAN/V000.PCM ...
  -> writes  .../VOICE/US_RYAN_preview.wav  (the example alert sentence, to
             listen to on your computer; NOT needed on the SD card)

Copy the US_RYAN folder onto the SD card as /VOICE/US_RYAN/ .

Useful options
--------------
  --match-loudness DIR   level the new clips to the same loudness as the
                         existing voice (e.g. "paraglide_vario_ESP32/Goes into SD card/VOICE")
  --only 0-9,50-59       regenerate just some clips (re-run after tweaking text)
  --overrides FILE.tsv   per-clip text overrides: "<id><TAB><text to speak>"
                         (the manifest/enum text is left alone -- use this to
                         respell a word if the TTS says it badly)
  --piper-args "..."     extra raw flags passed to piper, e.g. "--length-scale 1.1"
                         (slower speech; flag spelling differs between piper versions)
"""

import argparse
import math
import os
import re
import shlex
import subprocess
import sys
import tempfile
import wave
from fractions import Fraction

import numpy as np
from scipy.signal import resample_poly

SAMPLE_RATE = 16000          # must match VOICE_CLIP_SAMPLE_RATE in VoiceClips.h
EXPECTED_CLIPS = 60          # must match VOICE_CLIP_COUNT in VoiceClips.h
GAP_MS = 90                  # VOICE_INTER_CLIP_GAP_MS in VoiceAlert.h (preview only)

TRIM_THRESHOLD_REL = 0.02    # ~ -34 dB below the clip's own peak counts as "speech"
LEAD_PAD_MS = 15
TAIL_PAD_MS = 60
FADE_IN_MS = 3
FADE_OUT_MS = 15
PEAK_CEILING = 0.89          # about -1 dBFS
DEFAULT_TARGET_RMS_DBFS = -22.0
MAX_CLIP_SECONDS = 3.0

PREVIEW_SEQUENCE = [
    "PHRASE_AIRCRAFT_ON_YOUR", "CLOCK_2", "PHRASE_FLYING", "COMPASS_NORTH_WEST",
    "PHRASE_AT", "NUM_2", "NUM_THOUSAND", "NUM_4", "NUM_HUNDRED", "PHRASE_FEET",
    "NUM_1", "NUM_THOUSAND", "NUM_2", "NUM_HUNDRED", "PHRASE_FEET_ABOVE_YOU",
    "PHRASE_AND_IS", "NUM_8", "NUM_HUNDRED", "PHRASE_METERS_AWAY",
]


# --------------------------------------------------------------------------
# Inputs
# --------------------------------------------------------------------------

def parse_manifest(path):
    """manifest.txt lines: id<TAB>NAME<TAB>bytes<TAB>spoken text"""
    clips = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.rstrip("\r\n")
            if not line.strip():
                continue
            parts = line.split("\t")
            if len(parts) < 4:
                raise SystemExit(f"Bad manifest line (need 4 tab-separated fields): {line!r}")
            clips.append((int(parts[0]), parts[1], "\t".join(parts[3:])))
    clips.sort(key=lambda c: c[0])
    return clips


def check_against_header(clips, header_path):
    """Cross-check the manifest against VoiceClips.h so they can't drift."""
    if not os.path.exists(header_path):
        print(f"[warn] {header_path} not found -- skipping header cross-check")
        return
    pat = re.compile(r'^\s*VOICE_([A-Z0-9_]+)\s*=\s*(\d+),\s*//\s*"(.*)"\s*$')
    hdr = {}
    with open(header_path, "r", encoding="utf-8") as f:
        for line in f:
            m = pat.match(line)
            if m:
                hdr[int(m.group(2))] = (m.group(1), m.group(3))
    man = {c[0]: (c[1], c[2]) for c in clips}
    if set(hdr) != set(man):
        raise SystemExit(
            "manifest.txt and VoiceClips.h disagree on which clip ids exist.\n"
            f"  only in header:   {sorted(set(hdr) - set(man))}\n"
            f"  only in manifest: {sorted(set(man) - set(hdr))}")
    for cid in sorted(man):
        if hdr[cid][0] != man[cid][0] or hdr[cid][1] != man[cid][1]:
            raise SystemExit(
                f"Clip {cid} differs: header={hdr[cid]!r} manifest={man[cid]!r}")
    print(f"[ok] manifest matches VoiceClips.h ({len(man)} clips)")


def parse_overrides(path):
    out = {}
    if not path:
        return out
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.rstrip("\r\n")
            if not line.strip() or line.lstrip().startswith("#"):
                continue
            cid, _, text = line.partition("\t")
            out[int(cid)] = text
    return out


def parse_id_ranges(spec):
    ids = set()
    for part in spec.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            a, b = part.split("-", 1)
            ids.update(range(int(a), int(b) + 1))
        else:
            ids.add(int(part))
    return ids


# --------------------------------------------------------------------------
# Piper
# --------------------------------------------------------------------------

def synth_piper(text, model, data_dir, extra_args, wav_path):
    cmd = [sys.executable, "-m", "piper", "-m", model, "-f", wav_path]
    if data_dir:
        cmd += ["--data-dir", data_dir]
    cmd += extra_args
    res = subprocess.run(cmd, input=(text + "\n").encode("utf-8"),
                         stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if res.returncode != 0 or not os.path.exists(wav_path):
        raise SystemExit(
            f"piper failed for text {text!r}\n  command: {' '.join(shlex.quote(c) for c in cmd)}\n"
            f"  stderr: {res.stderr.decode('utf-8', 'replace')[-800:]}")


# --------------------------------------------------------------------------
# DSP
# --------------------------------------------------------------------------

def read_wav_mono_float(path):
    with wave.open(path, "rb") as w:
        n_ch, width, sr, n = w.getnchannels(), w.getsampwidth(), w.getframerate(), w.getnframes()
        raw = w.readframes(n)
    if width != 2:
        raise SystemExit(f"Expected 16-bit WAV from piper, got {width * 8}-bit")
    x = np.frombuffer(raw, dtype="<i2").astype(np.float32) / 32768.0
    if n_ch > 1:
        x = x.reshape(-1, n_ch).mean(axis=1)
    return x, sr


def to_16k(x, sr):
    if sr == SAMPLE_RATE:
        return x
    frac = Fraction(SAMPLE_RATE, sr).limit_denominator(1000)
    return resample_poly(x, frac.numerator, frac.denominator).astype(np.float32)


def active_mask(x):
    peak = float(np.max(np.abs(x))) if x.size else 0.0
    if peak <= 0:
        return np.zeros_like(x, dtype=bool), peak
    return np.abs(x) > peak * TRIM_THRESHOLD_REL, peak


def trim_and_fade(x):
    mask, peak = active_mask(x)
    if not mask.any():
        raise SystemExit("A clip came out silent -- check the voice/model")
    idx = np.nonzero(mask)[0]
    lead = int(SAMPLE_RATE * LEAD_PAD_MS / 1000)
    tail = int(SAMPLE_RATE * TAIL_PAD_MS / 1000)
    start = max(0, idx[0] - lead)
    end = min(len(x), idx[-1] + 1 + tail)
    y = x[start:end].copy()
    fi = min(len(y), int(SAMPLE_RATE * FADE_IN_MS / 1000))
    fo = min(len(y), int(SAMPLE_RATE * FADE_OUT_MS / 1000))
    if fi > 0:
        y[:fi] *= np.linspace(0.0, 1.0, fi, dtype=np.float32)
    if fo > 0:
        y[-fo:] *= np.linspace(1.0, 0.0, fo, dtype=np.float32)
    return y


def active_rms(x):
    mask, _ = active_mask(x)
    seg = x[mask] if mask.any() else x
    return float(np.sqrt(np.mean(seg.astype(np.float64) ** 2)))


def dbfs(v):
    return 20.0 * math.log10(v) if v > 1e-9 else -999.0


def existing_loudness_dbfs(dir_path):
    """Median active-RMS (dBFS) of an existing VOICE folder's V###.PCM clips."""
    vals = []
    for i in range(EXPECTED_CLIPS):
        p = os.path.join(dir_path, f"V{i:03d}.PCM")
        if os.path.exists(p):
            x = np.fromfile(p, dtype="<i2").astype(np.float32) / 32768.0
            if x.size:
                vals.append(dbfs(active_rms(x)))
    if not vals:
        raise SystemExit(f"--match-loudness: no V###.PCM clips found in {dir_path}")
    return float(np.median(vals))


def level_clip(x, target_rms_dbfs):
    rms = active_rms(x)
    peak = float(np.max(np.abs(x)))
    want = 10 ** (target_rms_dbfs / 20.0) / max(rms, 1e-9)
    limit = PEAK_CEILING / max(peak, 1e-9)
    peak_limited = want > limit
    return x * min(want, limit), peak_limited


def to_pcm16(x):
    return np.clip(np.round(x * 32767.0), -32768, 32767).astype("<i2")


# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------

def main():
    here = os.path.dirname(os.path.abspath(__file__))
    repo = os.path.dirname(here)
    default_voice_dir = os.path.join(repo, "paraglide_vario_ESP32", "Goes into SD card", "VOICE")

    ap = argparse.ArgumentParser(description="Generate a vario voice-clip profile with Piper TTS.")
    ap.add_argument("--profile", required=True,
                    help="Profile name, becomes the folder name (A-Z 0-9 _ -, max 12 chars), e.g. US_RYAN")
    ap.add_argument("--model", required=True,
                    help="Piper voice name (e.g. en_US-ryan-high) or path to a .onnx model")
    ap.add_argument("--data-dir", default=None, help="Folder holding downloaded Piper voices")
    ap.add_argument("--manifest", default=os.path.join(default_voice_dir, "manifest.txt"))
    ap.add_argument("--header", default=os.path.join(repo, "paraglide_vario_ESP32", "VoiceClips.h"))
    ap.add_argument("--out-dir", default=default_voice_dir,
                    help="Parent folder; the profile folder is created inside it")
    ap.add_argument("--match-loudness", default=None, metavar="DIR",
                    help="Level clips to match the median loudness of an existing VOICE folder")
    ap.add_argument("--target-rms", type=float, default=DEFAULT_TARGET_RMS_DBFS,
                    help=f"Target speech RMS in dBFS if --match-loudness not given (default {DEFAULT_TARGET_RMS_DBFS})")
    ap.add_argument("--overrides", default=None, help="TSV of '<id><TAB><text>' spoken-text overrides")
    ap.add_argument("--only", default=None, help="Only (re)generate these ids, e.g. 0-9,50")
    ap.add_argument("--piper-args", default="", help="Extra raw args for piper (quoted string)")
    ap.add_argument("--no-preview", action="store_true", help="Skip writing the preview WAV")
    args = ap.parse_args()

    if not re.fullmatch(r"[A-Za-z0-9_-]{1,12}", args.profile):
        raise SystemExit("--profile must be 1-12 chars of A-Z a-z 0-9 _ - (it is shown on the device menu)")
    profile = args.profile.upper()

    clips = parse_manifest(args.manifest)
    if len(clips) != EXPECTED_CLIPS or [c[0] for c in clips] != list(range(EXPECTED_CLIPS)):
        raise SystemExit(f"manifest must contain ids 0..{EXPECTED_CLIPS - 1} exactly once each "
                         f"(found {len(clips)})")
    check_against_header(clips, args.header)

    overrides = parse_overrides(args.overrides)
    only = parse_id_ranges(args.only) if args.only else None
    extra = shlex.split(args.piper_args)

    target = existing_loudness_dbfs(args.match_loudness) if args.match_loudness else args.target_rms
    print(f"[info] target speech loudness: {target:.1f} dBFS RMS")

    out_dir = os.path.join(args.out_dir, profile)
    os.makedirs(out_dir, exist_ok=True)

    print(f"\n{'id':>3}  {'name':<26} {'sec':>5}  {'rms':>6}  {'peak':>6}  note")
    pcm_by_name = {}
    total_bytes = 0
    problems = 0

    with tempfile.TemporaryDirectory() as tmp:
        for cid, name, text in clips:
            pcm_path = os.path.join(out_dir, f"V{cid:03d}.PCM")

            if only is not None and cid not in only:
                if os.path.exists(pcm_path):
                    pcm = np.fromfile(pcm_path, dtype="<i2")
                    pcm_by_name[name] = pcm
                    total_bytes += pcm.nbytes
                continue

            spoken = overrides.get(cid, text)
            wav_path = os.path.join(tmp, f"{cid:03d}.wav")
            synth_piper(spoken, args.model, args.data_dir, extra, wav_path)

            x, sr = read_wav_mono_float(wav_path)
            x = to_16k(x, sr)
            x = trim_and_fade(x)
            x, peak_limited = level_clip(x, target)
            pcm = to_pcm16(x)
            pcm.tofile(pcm_path)

            pcm_by_name[name] = pcm
            total_bytes += pcm.nbytes

            secs = len(pcm) / SAMPLE_RATE
            notes = []
            if peak_limited:
                notes.append("peak-limited")
            if secs > MAX_CLIP_SECONDS:
                notes.append("LONG")
                problems += 1
            if overrides.get(cid):
                notes.append("override")
            print(f"{cid:>3}  {name:<26} {secs:5.2f}  {dbfs(active_rms(x)):6.1f}  "
                  f"{dbfs(float(np.max(np.abs(x)))):6.1f}  {' '.join(notes)}")

    # manifest alongside the clips (same format as the existing one; the firmware ignores it)
    with open(os.path.join(out_dir, "manifest.txt"), "w", encoding="utf-8", newline="\n") as f:
        for cid, name, text in clips:
            p = os.path.join(out_dir, f"V{cid:03d}.PCM")
            size = os.path.getsize(p) if os.path.exists(p) else 0
            f.write(f"{cid}\t{name}\t{size}\t{text}\n")

    print(f"\n[done] {profile}: total {total_bytes / 1e6:.2f} MB of PCM "
          f"(existing voice is about 1.25 MB) -> {out_dir}")

    if not args.no_preview:
        missing = [n for n in PREVIEW_SEQUENCE if n not in pcm_by_name]
        if missing:
            print(f"[warn] preview skipped, clips not generated: {missing}")
        else:
            gap = np.zeros(int(SAMPLE_RATE * GAP_MS / 1000), dtype="<i2")
            seq = []
            for n in PREVIEW_SEQUENCE:
                seq.append(pcm_by_name[n])
                seq.append(gap)
            audio = np.concatenate(seq).astype("<i2")
            prev = os.path.join(args.out_dir, f"{profile}_preview.wav")
            with wave.open(prev, "wb") as w:
                w.setnchannels(1)
                w.setsampwidth(2)
                w.setframerate(SAMPLE_RATE)
                w.writeframes(audio.tobytes())
            print(f"[done] preview sentence -> {prev}")

    if problems:
        print(f"[warn] {problems} clip(s) flagged LONG -- listen to them; consider an override or a different voice")


if __name__ == "__main__":
    main()
