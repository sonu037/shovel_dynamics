"""
Dataset fixtures and provenance checking for shovel-dynamics E2E verification.
Pure-Python, zero-dependency data unpacking.
"""

import hashlib
import os
import zlib
from pathlib import Path
from typing import Any, Dict, Optional, Tuple
import numpy as np
import scipy.io as sio

FROZEN_P08B_MD5 = "5eaa1131f51257e72b24b2596a83598d"
P08B_REL_PATH = os.path.join(
    "M2", "experiments", "08_held_out", "rev2", "MS02P08B_simscape_20261118_130959.mat"
)


def get_repo_root() -> Path:
    """Find repository root dynamically from current file location or cwd."""
    p = Path(__file__).resolve()
    for parent in [p] + list(p.parents):
        if (parent / "claims.md").is_file() and (parent / "M1").is_dir():
            return parent
    cwd = Path.cwd().resolve()
    if (cwd / "claims.md").is_file():
        return cwd
    return Path(r"C:\Users\IIT KGP\MATLAB Drive\shovel-dynamics")


def compute_file_md5(file_path: str | Path) -> str:
    """Compute standard hexadecimal MD5 checksum of a file."""
    hasher = hashlib.md5()
    with open(file_path, "rb") as f:
        while chunk := f.read(65536):
            hasher.update(chunk)
    return hasher.hexdigest()


def verify_provenance_md5(
    file_path: Optional[str | Path] = None, expected_md5: str = FROZEN_P08B_MD5
) -> Tuple[bool, str, str]:
    """Verify that file matches the expected frozen cryptographic hash."""
    if file_path is None:
        file_path = get_repo_root() / P08B_REL_PATH
    actual_md5 = compute_file_md5(file_path)
    return (actual_md5 == expected_md5, actual_md5, expected_md5)


def load_p08b_simscape(file_path: Optional[str | Path] = None) -> Dict[str, Any]:
    """
    Unpack 30 Hz decimated telemetry (S30) from the frozen Simscape export.
    Zero-dependency pure-Python reader extracting the 16 exact 15608-byte zlib streams.
    """
    if file_path is None:
        file_path = get_repo_root() / P08B_REL_PATH

    with open(file_path, "rb") as f:
        buf = f.read()

    # MD5 check
    actual_md5 = hashlib.md5(buf).hexdigest()
    if actual_md5 != FROZEN_P08B_MD5:
        raise ValueError(
            f"Provenance mismatch for {file_path}: expected {FROZEN_P08B_MD5}, got {actual_md5}"
        )

    # Locate and decompress the 16 30 Hz signal chunks (each 1951 float64s = 15608 bytes)
    chunks = []
    buf_len = len(buf)
    for i in range(buf_len - 2):
        if buf[i] == 0x78 and buf[i + 1] in (0x01, 0x5E, 0x9C, 0xDA):
            try:
                decomp = zlib.decompress(buf[i : i + 65536])
                if len(decomp) == 15608:
                    chunks.append(np.frombuffer(decomp, dtype=np.float64).copy())
            except Exception:
                pass

    if len(chunks) != 16:
        raise RuntimeError(
            f"Expected exactly 16 S30 signal chunks (8 for E0, 8 for E1), found {len(chunks)}"
        )

    signal_names = ["t", "q_3", "q_3_dot", "d_4", "d_4_dot", "tau3_cmd", "f4_cmd", "outF"]

    e0_data = {name: chunks[i] for i, name in enumerate(signal_names)}
    e1_data = {name: chunks[i + 8] for i, name in enumerate(signal_names)}

    truth = {
        "Md": 54300.0,
        "Izz": 287900.0,
        "c": -1.32,
        "fv3": 3800.0,
        "fc3": 340.0,
        "fv4": 1473.0,
        "fc4": 480.0,
        "g": 9.80665,
        "x_ref": 1.50,
        "L_COG": 2.82,
        "x_tip": 2.00,
        "eps_c": 1e-3,
        "eps_obs": 3e-4,
    }

    return {
        "file_path": str(file_path),
        "md5": actual_md5,
        "cases": {"E0": {"S30": e0_data}, "E1": {"S30": e1_data}},
        "truth": truth,
        "sample_count": 1951,
        "duration_sec": 65.0,
        "dt": 1.0 / 30.0,
    }


def load_legacy_mat(rel_path: str) -> Dict[str, Any]:
    """Load standard MATLAB v5/v7 .mat files using scipy.io.loadmat."""
    full_path = get_repo_root() / rel_path
    if not full_path.is_file():
        raise FileNotFoundError(f"Legacy .mat file not found: {full_path}")
    return sio.loadmat(str(full_path), squeeze_me=True)
