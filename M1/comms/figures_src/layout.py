"""
Shapely-backed label placer for schematic figures.

Obstacles are true polygons (not bounding boxes), so a label may sit in the
concave gap beside a diagonal beam - something an AABB test forbids.
Placement: for each label, walk a ring of candidate offsets (preferred
directions first) and take the first whose text rectangle is disjoint from
every obstacle, every previously placed label, and every leader line.
"""
import numpy as np
from matplotlib.text import Text
from shapely.geometry import Polygon as SPoly, LineString, box as sbox
from shapely.ops import unary_union

def text_bbox(t, ren):
    """Text rectangle only - excludes the annotation's leader arrow."""
    return Text.get_window_extent(t, renderer=ren)

class Placer:
    def __init__(self, ax, fig, ren, pad=6.0):
        self.ax, self.fig, self.ren, self.pad = ax, fig, ren, pad
        self.specs, self.placed, self.leaders = [], [], []
        self.obstacles = None

    def add(self, xy, text, color, size=11.5, leader=True, prefer=None, weight="normal"):
        self.specs.append(dict(xy=np.asarray(xy, float), text=text, color=color,
                               size=size, leader=leader, prefer=prefer or [], weight=weight))

    def freeze_obstacles(self, skip_wide=0.55):
        """Snapshot geometry as shapely polygons/lines in display coordinates."""
        ax, ren = self.ax, self.ren
        W = ax.get_window_extent().width
        geoms = []
        for p in ax.patches:
            path = p.get_path().transformed(p.get_transform())
            for poly in path.to_polygons():
                if len(poly) >= 3:
                    g = SPoly(poly)
                    if g.is_valid and g.area > 90:
                        geoms.append(g)
        for l in ax.lines:
            d = l.get_xydata()
            if len(d) < 2:
                continue
            disp = ax.transData.transform(d)
            if disp[:, 0].max() - disp[:, 0].min() > skip_wide * W:
                continue                      # ground / datum lines may be crossed
            geoms.append(LineString(disp).buffer(max(l.get_linewidth(), 1.0) * 1.2))
        self.obstacles = unary_union(geoms) if geoms else None

    def _rect(self, cx, cy, w, h, ha):
        x0 = cx if ha == "left" else (cx - w if ha == "right" else cx - w / 2)
        return sbox(x0 - self.pad, cy - h / 2 - self.pad, x0 + w + self.pad, cy + h / 2 + self.pad)

    def solve(self, radii=(52, 76, 104, 136, 172, 212, 258, 310, 370, 440),
              angles=tuple(np.arange(0, 360, 9))):
        ax, fig, ren = self.ax, self.fig, self.ren
        report = []
        for sp in self.specs:
            t = ax.annotate(sp["text"], xy=sp["xy"], xytext=(0, 0), textcoords="offset pixels",
                            ha="center", va="center", color=sp["color"], fontsize=sp["size"],
                            fontweight=sp["weight"], zorder=40,
                            arrowprops=dict(arrowstyle="-", color=sp["color"], lw=0.8,
                                            shrinkA=0, shrinkB=4) if sp["leader"] else None)
            fig.canvas.draw()
            bb = text_bbox(t, ren); w, h = bb.width, bb.height
            ax0, ay0 = ax.transData.transform(sp["xy"])

            cands = []
            for px, py in sp["prefer"]:
                v = np.array([px, py], float); v /= np.linalg.norm(v)
                cands += [v * r for r in radii]
            for r in radii:
                for a in np.radians(angles):
                    cands.append(np.array([np.cos(a), np.sin(a)]) * r)

            chosen = None
            # pass 1 also keeps leaders clear of other labels; pass 2 drops that nicety
            for strict in (True, False):
                for dxy in cands:
                    ha = "left" if dxy[0] > 16 else ("right" if dxy[0] < -16 else "center")
                    rect = self._rect(ax0 + dxy[0], ay0 + dxy[1], w, h, ha)
                    if self.obstacles is not None and rect.intersects(self.obstacles):
                        continue
                    if any(rect.intersects(r0) for r0 in self.placed):
                        continue
                    if strict:
                        lead = LineString([(ax0, ay0), rect.centroid.coords[0]])
                        if any(lead.crosses(r0) for r0 in self.placed):
                            continue
                        # a leader that runs across the machine reads as clutter:
                        # allow only a short stub inside geometry near its own anchor
                        if self.obstacles is not None:
                            inside = lead.intersection(self.obstacles)
                            if not inside.is_empty and inside.length > 26.0:
                                continue
                    chosen = (dxy, ha, rect); break
                if chosen is not None:
                    break

            if chosen is None:
                chosen = (np.array([0.0, 250.0]), "center",
                          self._rect(ax0, ay0 + 250, w, h, "center"))
                report.append(sp["text"][:40])
            t.xyann = tuple(chosen[0]); t.set_ha(chosen[1])
            self.placed.append(chosen[2])
        fig.canvas.draw()
        return report

def audit(ax, fig, ren, obstacles):
    """Independent check: text-text overlap, text-on-geometry, clipping."""
    from matplotlib.text import Text
    T = [(t.get_text()[:34], text_bbox(t, ren)) for t in ax.texts if t.get_text().strip()]
    R = [(n, sbox(b.x0, b.y0, b.x1, b.y1)) for n, b in T]
    tt = [(R[i][0], R[j][0]) for i in range(len(R)) for j in range(i + 1, len(R))
          if R[i][1].intersects(R[j][1])]
    tg = [n for n, r in R if obstacles is not None and r.intersects(obstacles)]
    x0, x1 = ax.get_xlim(); y0, y1 = ax.get_ylim(); clip = 0
    for p in ax.patches:
        bb = p.get_extents().transformed(ax.transData.inverted())
        if bb.x0 < x0 or bb.x1 > x1 or bb.y0 < y0 or bb.y1 > y1:
            clip += 1
    return tt, tg, clip


class Callouts:
    """
    Numbered-disc annotation for dense machine drawings.

    Each part gets a small numbered disc placed in clear space near it, joined by a
    short leader. The names live in a key beside the figure. Discs are tiny and
    uniform, so placement always succeeds and the drawing never carries long text.
    """
    def __init__(self, ax, fig, ren, r_px=11.0, pad=3.0):
        self.ax, self.fig, self.ren = ax, fig, ren
        self.r, self.pad = r_px, pad
        self.items, self.placed = [], []
        self.obstacles = None

    def add(self, xy, name, prefer=None):
        self.items.append(dict(xy=np.asarray(xy, float), name=name, prefer=prefer or []))

    def freeze(self, placer_obstacles):
        self.obstacles = placer_obstacles

    def solve(self, radii=(24, 32, 42, 54, 68, 84, 104, 128), angles=tuple(np.arange(0, 360, 8)),
              face="#FFFFFF", edge="#1F3864", txt="#1F3864"):
        from matplotlib.patches import Circle as _C
        ax, fig, ren = self.ax, self.fig, self.ren
        inv = ax.transData.inverted()
        missed = []
        for i, it in enumerate(self.items, start=1):
            a = ax.transData.transform(it["xy"])
            cands = []
            for px, py in it["prefer"]:
                v = np.array([px, py], float); v /= np.linalg.norm(v)
                cands += [v * r for r in radii]
            for r in radii:
                for t in np.radians(angles):
                    cands.append(np.array([np.cos(t), np.sin(t)]) * r)
            spot = None
            for strict in (True, False):
                for d in cands:
                    c = a + d
                    disc = sbox(c[0]-self.r-self.pad, c[1]-self.r-self.pad,
                                c[0]+self.r+self.pad, c[1]+self.r+self.pad)
                    if self.obstacles is not None and disc.intersects(self.obstacles):
                        continue
                    if any(disc.intersects(p) for p in self.placed):
                        continue
                    if strict:
                        lead = LineString([tuple(a), tuple(c)])
                        if any(lead.crosses(p) for p in self.placed):
                            continue
                    spot = (c, disc); break
                if spot: break
            if spot is None:
                c = a + np.array([0.0, 150.0]); spot = (c, sbox(c[0]-self.r, c[1]-self.r,
                                                                c[0]+self.r, c[1]+self.r))
                missed.append(it["name"])
            c, disc = spot
            self.placed.append(disc)
            cd = inv.transform(c); ad = it["xy"]
            v = c - a; L = np.linalg.norm(v)
            if L > self.r + 3:
                stop = inv.transform(c - v / L * (self.r + 1.5))
                ax.annotate("", xy=tuple(stop), xytext=tuple(ad),
                            arrowprops=dict(arrowstyle="-", color=edge, lw=0.8,
                                            shrinkA=0, shrinkB=0), zorder=38)
            rad_data = abs(inv.transform((c[0]+self.r, c[1]))[0] - cd[0])
            ax.add_patch(_C(cd, rad_data, fc=face, ec=edge, lw=1.2, zorder=40))
            ax.text(cd[0], cd[1], str(i), color=txt, fontsize=10.5, ha="center", va="center",
                    zorder=41, fontweight="bold")
        return missed

    def key_lines(self):
        return [f"{i}   {it['name']}" for i, it in enumerate(self.items, start=1)]
