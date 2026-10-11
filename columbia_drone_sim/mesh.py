"""Small binary/ASCII STL reader. The original mesh is never modified."""
from pathlib import Path
import struct
import numpy as np


def load_stl(path):
    raw = Path(path).read_bytes()
    n = struct.unpack_from('<I', raw, 80)[0] if len(raw) >= 84 else 0
    if n and len(raw) == 84 + 50 * n:
        dtype = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
        triangles = np.frombuffer(raw, dtype=dtype, offset=84, count=n)['vertices'].astype(float)
    else:
        vertices = [list(map(float, line.split()[1:4])) for line in raw.decode('ascii').splitlines()
                    if line.strip().startswith('vertex ')]
        triangles = np.array(vertices, dtype=float).reshape(-1, 3, 3)
    if not len(triangles) or not np.isfinite(triangles).all():
        raise ValueError('STL is empty or has non-finite coordinates')
    return triangles


def prepare_mesh(config, base):
    c = config['mesh']
    tri = (load_stl(Path(base) / c['path']) - np.array(c['body_origin_stl_units'])) * c['units_to_m']
    for axis, angle in enumerate(np.deg2rad(c['rotation_deg_xyz'])):
        co, si = np.cos(angle), np.sin(angle)
        R = np.eye(3)
        i, j = (axis + 1) % 3, (axis + 2) % 3
        R[i, i] = R[j, j] = co
        R[i, j], R[j, i] = -si, si
        tri = tri @ R.T
    # Vertex clustering, not random face removal: produces a lightweight closed-ish
    # display mesh. This is display-only and never used for mass/aerodynamics.
    grid = c.get('display_vertex_grid_m', 0)
    if grid > 0:
        flat = tri.reshape(-1, 3)
        _, inverse = np.unique(np.round(flat / grid).astype(int), axis=0, return_inverse=True)
        count = np.bincount(inverse)
        centers = np.stack([np.bincount(inverse, weights=flat[:, k]) / count for k in range(3)], axis=1)
        faces = inverse.reshape(-1, 3)
        valid = (faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2]) & (faces[:, 0] != faces[:, 2])
        faces = faces[valid]
        _, idx = np.unique(np.sort(faces, axis=1), axis=0, return_index=True)
        tri = centers[faces[np.sort(idx)]]
    return tri
