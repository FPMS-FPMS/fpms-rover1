"""Render a scene of meshes to PNGs: iso / side / top / front views (VTK, offscreen)
plus planar sections (trimesh) drawn with matplotlib.
scene = list of dicts: {path or mesh, color (r,g,b 0-1), opacity, name}
"""
import vtk, numpy as np, trimesh, os, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from vtk.util.numpy_support import numpy_to_vtk, numpy_to_vtkIdTypeArray


def _poly(mesh):
    pts = vtk.vtkPoints(); pts.SetData(numpy_to_vtk(np.ascontiguousarray(mesh.vertices, dtype=np.float64), deep=True))
    f = np.hstack([np.full((len(mesh.faces), 1), 3), mesh.faces]).astype(np.int64).ravel()
    cells = vtk.vtkCellArray(); cells.SetCells(len(mesh.faces), numpy_to_vtkIdTypeArray(f, deep=True))
    p = vtk.vtkPolyData(); p.SetPoints(pts); p.SetPolys(cells)
    n = vtk.vtkPolyDataNormals(); n.SetInputData(p); n.SetFeatureAngle(35); n.Update()
    return n.GetOutput()


VIEWS = {  # camera direction (from target to camera), view-up
    'iso':      ((1.0, -1.1, 0.85), (0, 0, 1)),
    'iso_rear': ((-1.0, 1.1, 0.85), (0, 0, 1)),
    'side':     ((0, -1, 0), (0, 0, 1)),
    'top':      ((0, 0, 1), (1, 0, 0)),
    'front':    ((1, 0, 0), (0, 0, 1)),
}


def _load(s):
    return s['mesh'] if 'mesh' in s else trimesh.load(s['path'])


def render_views(scene, outdir, prefix, views=('iso', 'iso_rear', 'side', 'top', 'front'), size=(1400, 1000)):
    os.makedirs(outdir, exist_ok=True)
    meshes = [_load(s) for s in scene]
    allb = np.vstack([m.bounds for m in meshes]); lo, hi = allb.min(0), allb.max(0)
    c = (lo + hi) / 2; rad = np.linalg.norm(hi - lo) / 2
    out = []
    for v in views:
        d, up = VIEWS[v]
        r = vtk.vtkRenderer(); r.SetBackground(0.96, 0.96, 0.97); r.UseDepthPeelingOn(); r.SetMaximumNumberOfPeels(8)
        w = vtk.vtkRenderWindow(); w.SetOffScreenRendering(1); w.AddRenderer(r); w.SetSize(*size); w.SetAlphaBitPlanes(1); w.SetMultiSamples(0)
        for s, m in zip(scene, meshes):
            mp = vtk.vtkPolyDataMapper(); mp.SetInputData(_poly(m))
            a = vtk.vtkActor(); a.SetMapper(mp)
            pr = a.GetProperty(); pr.SetColor(*s.get('color', (0.7, 0.7, 0.75))); pr.SetOpacity(s.get('opacity', 1.0))
            pr.SetSpecular(0.15); pr.SetDiffuse(0.85); pr.SetAmbient(0.18)
            r.AddActor(a)
        cam = r.GetActiveCamera(); d = np.array(d, float); d /= np.linalg.norm(d)
        cam.SetFocalPoint(*c); cam.SetPosition(*(c + d * rad * 6)); cam.SetViewUp(*up)
        # fit: project the bounding-box corners onto the screen axes
        u = np.array(up, float); u = u - d * (u @ d); u /= np.linalg.norm(u); rt = np.cross(u, d)
        corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]) - c
        hh = np.ptp(corners @ u) / 2; ww = np.ptp(corners @ rt) / 2
        cam.ParallelProjectionOn(); cam.SetParallelScale(max(hh, ww * size[1] / size[0]) * 1.06)
        r.ResetCameraClippingRange()
        lk = vtk.vtkLightKit(); lk.AddLightsToRenderer(r)
        t = vtk.vtkTextActor(); t.SetInput(f'{prefix}  {v}   (X fwd, Y left, Z up)'); t.GetTextProperty().SetFontSize(20)
        t.GetTextProperty().SetColor(0.2, 0.2, 0.2); t.SetPosition(10, 10); r.AddActor2D(t)
        w.Render()
        f = vtk.vtkWindowToImageFilter(); f.SetInput(w); f.Update()
        pw = vtk.vtkPNGWriter(); fn = os.path.join(outdir, f'{prefix}_{v}.png'); pw.SetFileName(fn); pw.SetInputConnection(f.GetOutputPort()); pw.Write()
        out.append(fn)
    return out


def section(scene, outdir, prefix, normal, origin, label, xlim=None, ylim=None):
    """Plane section. normal 'x' -> plot Y-Z seen from the front, 'y' -> X-Z seen from the right, 'z' -> X-Y plan."""
    ax_n = {'x': [1, 0, 0], 'y': [0, 1, 0], 'z': [0, 0, 1]}[normal]
    pick = {'x': (1, 2), 'y': (0, 2), 'z': (0, 1)}[normal]
    names = {'x': ('Y (left +)', 'Z'), 'y': ('X (forward +)', 'Z'), 'z': ('X (forward +)', 'Y (left +)')}[normal]
    fig, ax = plt.subplots(figsize=(11, 8))
    for s in scene:
        m = _load(s)
        sec = m.section(plane_origin=origin, plane_normal=ax_n)
        col = s.get('color', (0.4, 0.4, 0.4)); solid = s.get('opacity', 1) > 0.6
        if sec is not None:
            for ent in sec.entities:
                p = sec.vertices[ent.points][:, pick]
                if solid:
                    ax.fill(p[:, 0], p[:, 1], color=col, alpha=0.55, lw=0)
                ax.plot(p[:, 0], p[:, 1], color=col, lw=1.2 if solid else 1.0, ls='-' if solid else '--')
        ax.plot([], [], color=col, label=s.get('name', ''), ls='-' if solid else '--')
    if normal == 'x':
        ax.invert_xaxis()   # seen from the front: +Y (left) appears on the right
    ax.set_aspect('equal'); ax.grid(alpha=0.3); ax.set_xlabel(names[0]); ax.set_ylabel(names[1])
    if xlim: ax.set_xlim(*xlim)
    if ylim: ax.set_ylim(*ylim)
    ax.legend(fontsize=7, loc='upper left', bbox_to_anchor=(1.01, 1))
    ax.set_title(f'{prefix}: {label}')
    fig.tight_layout()
    fn = os.path.join(outdir, f'{prefix}_sec_{label.split()[0]}_{label.split()[-1].replace("=", "")}.png')
    fig.savefig(fn, dpi=100); plt.close(fig)
    return fn
