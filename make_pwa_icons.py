#!/usr/bin/env python3
"""Generate the PWA icon set in docs/icons/ from a vector description.

Re-run after changing the artwork below:
    pip install pillow
    python make_pwa_icons.py

Outputs (all referenced by docs/manifest.webmanifest or docs/index.html):
    icon-192.png  icon-512.png            normal icons, art fills the canvas
    icon-192-maskable.png  icon-512-maskable.png
                                          art inside the 80% safe zone, for
                                          Android adaptive-icon cropping
    apple-touch-icon.png (180)            iOS home screen, must be opaque
    favicon-32.png                        browser tab
"""
import os
from PIL import Image, ImageDraw

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "docs", "icons")

BG      = (14, 14, 14)     # #0e0e0e - dashboard chrome
EDGE    = (51, 51, 51)     # #333    - card border
GREEN   = (0, 255, 0)      # #0f0    - phosphor green, the dashboard accent
DIM     = (0, 160, 0)
FLAME   = (255, 136, 0)    # #f80    - boost phase colour
SKY     = (68, 170, 255)   # #4af    - link colour, used for the signal arcs

SS = 8  # supersample factor; art is drawn at SS*size then box-filtered down


def draw_icon(size, scale=1.0, rounded=True):
    """Render one icon. `scale` shrinks the artwork for the maskable safe zone."""
    S = size * SS
    img = Image.new("RGB", (S, S), BG)
    d = ImageDraw.Draw(img)

    if rounded:
        # Rounded-square plate with a hairline border, matching .card in style.css.
        r = int(S * 0.22)
        d.rounded_rectangle([0, 0, S - 1, S - 1], radius=r, fill=BG,
                            outline=EDGE, width=max(1, int(S * 0.012)))

    cx, cy = S / 2, S / 2
    u = S * scale / 100.0          # 1 unit = 1% of the (scaled) canvas

    def P(pts):
        return [(cx + x * u, cy + y * u) for x, y in pts]

    # --- signal arcs: three widening arcs behind the nose, telemetry uplink ---
    ax, ay = cx - 2 * u, cy - 14 * u   # arcs radiate from just behind the nose
    for i, rad in enumerate((24, 33, 42)):
        bb = [ax - rad * u, ay - rad * u, ax + rad * u, ay + rad * u]
        d.arc(bb, start=196, end=254, fill=(SKY, (46, 118, 178), (34, 84, 128))[i],
              width=max(1, int(u * 3.4)))

    # --- exhaust flame ---
    d.polygon(P([(0, 36), (-8.5, 17), (0, 23), (8.5, 17)]), fill=FLAME)

    # --- fins ---
    d.polygon(P([(-7, 6), (-19, 22), (-19, 8), (-7, -4)]), fill=DIM)
    d.polygon(P([(7, 6), (19, 22), (19, 8), (7, -4)]), fill=DIM)

    # --- body: pointed nose, straight sides, flat base ---
    d.polygon(P([(0, -46), (8.5, -22), (8.5, 20), (-8.5, 20), (-8.5, -22)]),
              fill=GREEN)

    # --- porthole ---
    r = 4.6 * u
    d.ellipse([cx - r, cy - 13 * u - r, cx + r, cy - 13 * u + r], fill=BG)

    return img.resize((size, size), Image.LANCZOS)


def save(img, name):
    path = os.path.join(OUT, name)
    img.save(path, "PNG", optimize=True)
    print("%-28s %5d bytes" % (name, os.path.getsize(path)))


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    save(draw_icon(192, scale=0.9), "icon-192.png")
    save(draw_icon(512, scale=0.9), "icon-512.png")
    # Maskable icons are cropped to a circle/squircle by the launcher; keep the
    # art inside the inner 80% and let the plate bleed to the edges.
    save(draw_icon(192, scale=0.72, rounded=False), "icon-192-maskable.png")
    save(draw_icon(512, scale=0.72, rounded=False), "icon-512-maskable.png")
    save(draw_icon(180, scale=0.9), "apple-touch-icon.png")
    save(draw_icon(32, scale=0.94), "favicon-32.png")
