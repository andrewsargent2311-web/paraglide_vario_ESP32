#!/usr/bin/env python3
"""
dem_to_agldem.py -- Convert one or more LINZ DEM GeoTIFF tiles into a single
compact binary grid ("ADEM" format) that an ESP32 can seek/read directly
off an SD card to compute height-above-ground (AGL).

Handles the "lower North Island split across several 50km x 50km LINZ
tiles" case: each tile is downscaled *and* reprojected straight onto one
shared WGS84 grid, then pasted into a single output array. Tiles are
never mosaicked at native 8m resolution (that would mean holding the
whole clipped region in memory at full resolution, which does not scale
past a single tile) -- each tile is shrunk to the target resolution
before it's placed on the shared grid, so memory use only depends on the
*output* size, not the combined input size.

Why a custom binary format instead of just copying GeoTIFFs onto the SD
card?
  - Reading a real GeoTIFF (esp. compressed/tiled/COG) needs libtiff/GDAL,
    which is far too heavy for an ESP32.
  - The vario only ever needs "elevation at this lat/lon", so a flat,
    fixed-cell-size grid of int16 metres that can be found with pure
    arithmetic (no parsing, no per-tile file switching) is exactly what's
    needed. One f.seek() gets you to the right byte, anywhere in the
    whole combined area.

Pipeline, per tile:
  1. Open the source GeoTIFF (any CRS -- LINZ DEMs are NZTM2000/EPSG:2193).
  2. Reproject + downsample it in one step (average resampling) straight
     onto the shared output grid's exact pixel lattice, so adjoining
     tiles line up with no seams and no separate "downsample then warp"
     pass is needed.
  3. Paste the tile's contribution into the shared output array.

After all tiles are placed:
  4. Write a small fixed header + rows*cols int16 elevation grid (metres,
     nodata = -32768) -- row 0 = northernmost row, col 0 = westernmost.

Usage -- two ways to run this:

  1. Easiest: just run it with no arguments and answer the questions.
       python3 dem_to_agldem.py
     It will ask you for the folder your .tif tiles are in, where to save
     the result, and what resolution to use (just press Enter to accept
     the sensible defaults it suggests).

  2. From a terminal, with everything specified up front:
       python3 dem_to_agldem.py --output DEM.ADEM --resolution 30 tiles/*.tif

  --resolution   target ground sample distance in metres (default 30).
                 Bigger = smaller file, coarser terrain. 30-50m is plenty
                 for AGL on a paraglider/paramotor -- you don't need 8m.
"""

import argparse
import glob
import math
import struct
import sys
from pathlib import Path

try:
    import numpy as np
    import rasterio
    from rasterio.enums import Resampling
    from rasterio.transform import Affine, from_origin
    from rasterio.warp import reproject, transform_bounds
except ImportError as e:
    print(f"Missing required package: {e}\n")
    print("This script needs numpy and rasterio installed. Install them with:\n")
    print("    pip install rasterio --break-system-packages\n")
    print("(drop --break-system-packages if you're on Windows -- that flag "
          "is only needed on some Linux setups)")
    if len(sys.argv) == 1:
        try:
            input("\nPress Enter to close this window...")
        except EOFError:
            pass
    sys.exit(1)

MAGIC = b"ADEM"
VERSION = 1
NODATA = -32768

# Must match the #pragma pack(1) struct DemHeader on the ESP32 side
# byte-for-byte: 4s H H d d d d I I h  (little-endian, no padding)
HEADER_FMT = "<4sHHddddIIh"
HEADER_SIZE = struct.calcsize(HEADER_FMT)

METRES_PER_DEG_LAT = 111_320.0  # close enough everywhere; lat degrees barely vary in length


def expand_inputs(raw_args):
    """Expand any glob patterns (mainly for Windows, where the shell
    doesn't do it) and de-duplicate while preserving order."""
    paths = []
    for arg in raw_args:
        if any(ch in arg for ch in "*?["):
            matches = sorted(glob.glob(arg))
            if not matches:
                print(f"Warning: pattern '{arg}' matched no files", file=sys.stderr)
            paths.extend(matches)
        else:
            paths.append(arg)
    seen = set()
    unique = []
    for p in paths:
        if p not in seen:
            seen.add(p)
            unique.append(p)
    return unique


def compute_shared_grid(tile_paths, resolution_m):
    """Union the WGS84 bounds of every tile and lay out one shared grid
    at the target resolution that all tiles will be reprojected onto."""
    west = south = math.inf
    east = north = -math.inf

    for path in tile_paths:
        with rasterio.open(path) as src:
            b = transform_bounds(src.crs, "EPSG:4326", *src.bounds)
            west, south = min(west, b[0]), min(south, b[1])
            east, north = max(east, b[2]), max(north, b[3])
            print(f"  {path}: CRS={src.crs}  native res={src.res[0]:.1f}m  "
                  f"WGS84 bounds={tuple(round(v, 4) for v in b)}")

    mean_lat = (south + north) / 2.0
    deg_lat = resolution_m / METRES_PER_DEG_LAT
    deg_lon = resolution_m / (METRES_PER_DEG_LAT * math.cos(math.radians(mean_lat)))

    cols = max(1, math.ceil((east - west) / deg_lon))
    rows = max(1, math.ceil((north - south) / deg_lat))

    transform = from_origin(west, north, deg_lon, deg_lat)

    size_mb = (rows * cols * 2) / (1024 * 1024)
    print(f"\nCombined area: {(east - west) * METRES_PER_DEG_LAT / 1000:.0f}km x "
          f"{(north - south) * METRES_PER_DEG_LAT / 1000:.0f}km  "
          f"-> grid {rows} x {cols} @ ~{resolution_m}m  (~{size_mb:.1f} MB)")
    if size_mb > 200:
        print("  This is a big file for an SD-card lookup on an ESP32 -- "
              "consider a coarser --resolution (e.g. 50 or 90) unless you "
              "specifically need finer terrain detail.")

    return transform, rows, cols, west, north, deg_lat, deg_lon


def paste_tile(path, dst, dst_transform, dst_rows, dst_cols):
    """Reproject+downsample one tile straight onto the shared grid's pixel
    lattice, then blend its valid pixels into dst (which starts as NaN)."""
    with rasterio.open(path) as src:
        # Figure out exactly which pixel window of the shared grid this
        # tile can possibly touch, so we only reproject a small chunk --
        # not the whole (potentially huge) combined-area array.
        b = transform_bounds(src.crs, "EPSG:4326", *src.bounds)
        west, south, east, north = b

        inv = ~dst_transform
        col0f, row0f = inv * (west, north)
        col1f, row1f = inv * (east, south)
        col0 = max(0, math.floor(min(col0f, col1f)) - 1)
        row0 = max(0, math.floor(min(row0f, row1f)) - 1)
        col1 = min(dst_cols, math.ceil(max(col0f, col1f)) + 1)
        row1 = min(dst_rows, math.ceil(max(row0f, row1f)) + 1)

        if col1 <= col0 or row1 <= row0:
            print(f"  {path}: no overlap with output grid, skipping")
            return

        window_transform = dst_transform * Affine.translation(col0, row0)
        window_shape = (row1 - row0, col1 - col0)

        tile_dst = np.full(window_shape, np.nan, dtype=np.float32)
        reproject(
            source=rasterio.band(src, 1),
            destination=tile_dst,
            src_transform=src.transform,
            src_crs=src.crs,
            src_nodata=src.nodata,
            dst_transform=window_transform,
            dst_crs="EPSG:4326",
            dst_nodata=np.nan,
            resampling=Resampling.average,
        )

        valid = ~np.isnan(tile_dst)
        target_slice = dst[row0:row1, col0:col1]
        target_slice[valid] = tile_dst[valid]
        n_valid = int(valid.sum())
        print(f"  {path}: pasted {n_valid} cells into rows [{row0}:{row1}) "
              f"cols [{col0}:{col1})")


def write_adem(path, grid, origin_lat, origin_lon, cell_size_lat, cell_size_lon):
    rows, cols = grid.shape

    int_grid = np.where(np.isnan(grid), NODATA, np.round(grid)).astype(np.int16)

    header = struct.pack(
        HEADER_FMT,
        MAGIC, VERSION, 0,
        origin_lat, origin_lon,
        cell_size_lat, cell_size_lon,
        rows, cols,
        NODATA,
    )

    with open(path, "wb") as f:
        f.write(header)
        f.write(int_grid.tobytes())  # row-major, matches ESP32 read order

    valid = int_grid[int_grid != NODATA]
    n_nodata = int_grid.size - valid.size
    size_kb = (HEADER_SIZE + int_grid.nbytes) / 1024
    print(f"\nWrote {path}")
    print(f"  grid: {rows} rows x {cols} cols  ({size_kb:.1f} KB)")
    print(f"  origin (NW corner): lat={origin_lat:.6f} lon={origin_lon:.6f}")
    print(f"  cell size: {cell_size_lat:.8f} deg lat x {cell_size_lon:.8f} deg lon")
    if valid.size:
        print(f"  elevation range: {valid.min()} .. {valid.max()} m "
              f"({n_nodata} nodata cells)")
    else:
        print("  WARNING: entire grid is nodata -- check your tiles/bounds")


def prompt_interactive():
    """Used when the script is double-clicked / run with no arguments --
    ask for a folder instead of expecting command-line flags, and keep
    the console window open at the end so you can read the output."""
    print("No command-line arguments given -- running in interactive mode.\n")

    folder = input("Folder containing your LINZ .tif tiles: ").strip().strip('"')
    if not folder:
        folder = "."

    pattern = str(Path(folder) / "*.tif")
    tile_paths = sorted(glob.glob(pattern))
    if not tile_paths:
        # Try upper-case extension too -- common on Windows downloads
        tile_paths = sorted(glob.glob(str(Path(folder) / "*.TIF")))
    if not tile_paths:
        print(f"\nNo .tif files found in: {folder}")
        return None

    print(f"\nFound {len(tile_paths)} tile(s) in that folder.")

    default_out = str(Path(folder) / "DEM.ADEM")
    out_path = input(f"\nOutput file [{default_out}]: ").strip().strip('"')
    if not out_path:
        out_path = default_out

    res_str = input("Resolution in metres [30]: ").strip()
    resolution = float(res_str) if res_str else 30.0

    class Args:
        pass
    args = Args()
    args.tiles = tile_paths
    args.output = out_path
    args.resolution = resolution
    return args


def main():
    if len(sys.argv) == 1:
        # Double-clicked, or run with no arguments -- ask instead of
        # printing a usage error and exiting immediately.
        args = prompt_interactive()
        if args is None:
            input("\nPress Enter to exit...")
            return
    else:
        ap = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
        ap.add_argument("tiles", nargs="+",
                         help="One or more LINZ DEM GeoTIFFs, or glob pattern(s) like tiles/*.tif")
        ap.add_argument("--output", "-o", required=True, help="Output .ADEM file for the SD card")
        ap.add_argument("--resolution", type=float, default=30.0,
                         help="Target resolution in metres (default: 30)")
        args = ap.parse_args()

    tile_paths = expand_inputs(args.tiles)
    if not tile_paths:
        print("No input tiles found.", file=sys.stderr)
        if len(sys.argv) == 1:
            input("\nPress Enter to exit...")
        sys.exit(1)

    print(f"Found {len(tile_paths)} tile(s):")
    transform, rows, cols, west, north, deg_lat, deg_lon = compute_shared_grid(
        tile_paths, args.resolution
    )

    dst = np.full((rows, cols), np.nan, dtype=np.float32)

    print("\nReprojecting + pasting tiles onto shared grid:")
    for path in tile_paths:
        paste_tile(path, dst, transform, rows, cols)

    write_adem(args.output, dst, north, west, deg_lat, deg_lon)
    print("\nCopy this file to the SD card root (e.g. as /DEM.ADEM) and "
          "point DEM_FILE at it in the firmware.")

    if len(sys.argv) == 1:
        try:
            input("\nDone -- press Enter to close this window...")
        except EOFError:
            pass


if __name__ == "__main__":
    try:
        main()
    except Exception:
        # If anything blows up (missing rasterio, bad path, whatever) on
        # Windows the console window otherwise flashes and vanishes before
        # you can read why. Print the real error and hold the window open
        # instead, but only when double-clicked (len(sys.argv) == 1) --
        # if run from a terminal you already see the traceback normally,
        # so don't also swallow the non-zero exit code there.
        import traceback
        traceback.print_exc()
        if len(sys.argv) == 1:
            try:
                input("\nSomething went wrong (see error above). "
                      "Press Enter to close this window...")
            except EOFError:
                pass
        sys.exit(1)
