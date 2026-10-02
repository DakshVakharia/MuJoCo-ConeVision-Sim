"""Procedural, seamlessly tiling ground textures (PNG) generated with numpy + OpenCV."""
from pathlib import Path

import numpy as np


def _fft_noise(size, rng, beta):
    """Periodic 1/f^beta noise in [-1, 1] (tiles seamlessly)."""
    f = np.fft.fftfreq(size)
    fx, fy = np.meshgrid(f, f)
    rad = np.sqrt(fx ** 2 + fy ** 2)
    rad[0, 0] = 1.0
    spec = (rng.standard_normal((size, size)) + 1j * rng.standard_normal((size, size))) / rad ** beta
    spec[0, 0] = 0
    n = np.real(np.fft.ifft2(spec))
    return n / (np.abs(n).max() + 1e-9)


def make_texture(kind, path, size=1024, seed=1):
    """Write texture `kind` (asphalt | grass) to `path`."""
    import cv2
    rng = np.random.default_rng(seed)
    coarse = _fft_noise(size, rng, 1.6)
    fine = _fft_noise(size, rng, 0.6)
    if kind == "asphalt":
        base = 0.30 + 0.05 * coarse + 0.04 * fine
        speck = (rng.random((size, size)) > 0.985) * rng.uniform(0.1, 0.25, (size, size))
        g = np.clip(base + speck, 0, 1)
        img = np.stack([g * 0.98, g, g * 1.03], axis=-1)
    elif kind == "grass":
        base = 0.5 + 0.28 * coarse + 0.22 * fine
        img = np.stack([0.15 + 0.10 * base, 0.27 + 0.20 * base, 0.07 + 0.05 * base], axis=-1)
    else:
        raise ValueError(f"unknown procedural texture '{kind}'")
    img = np.clip(img, 0, 1)
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(path), (img[..., ::-1] * 255).astype(np.uint8))   # RGB -> BGR
    return Path(path)


def write_palette(colors, path, swatch=8):
    """Texture with one flat swatch per colour in a single row (colour k at u = (k + 0.5) / n).

    Meshes give every vertex of a triangle the same texcoord, so texture derivatives are zero and
    the GPU samples mip level 0: colours stay exact at any distance.
    """
    import cv2
    img = np.zeros((swatch, len(colors) * swatch, 3), np.uint8)
    for k, c in enumerate(colors):
        img[:, k * swatch:(k + 1) * swatch] = (np.array(c[::-1], float) * 255).round().astype(np.uint8)
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(path), img)
    return Path(path)
