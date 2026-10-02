"""Camera rendering (RGB + segmentation) with reusable MuJoCo renderers."""
import mujoco
import numpy as np


class CameraRenderer:
    """One GL context, two MuJoCo render contexts: RGB (multisampled) and segmentation (no MSAA).

    A single mujoco.Renderer owns the GL context + RGB mjrContext + scene; a second mjrContext
    without multisampling is created in the same GL context for id rendering (ids must not be
    blended, and it is faster). Using two separate GL contexts was ~3x slower because of
    context switching. Segmentation decoding is done here in numpy.
    """

    def __init__(self, model, cam_cfg, msaa_samples=None):
        self.model = model
        self.cam = cam_cfg["name"]
        self.width = int(cam_cfg["width"])
        self.height = int(cam_cfg["height"])
        orig = int(model.vis.quality.offsamples)
        try:
            if msaa_samples is not None:
                model.vis.quality.offsamples = int(msaa_samples)
            self._r = mujoco.Renderer(model, self.height, self.width)
            model.vis.quality.offsamples = 0
            self._r._gl_context.make_current()
            self._seg_ctx = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_100)
            mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, self._seg_ctx)
        finally:
            model.vis.quality.offsamples = orig
        self._rect = mujoco.MjrRect(0, 0, self.width, self.height)
        self._rgb_buf = np.empty((self.height, self.width, 3), np.uint8)
        self._seg_buf = np.empty((self.height, self.width, 3), np.uint8)
        self._code4 = np.zeros((self.height, self.width, 4), np.uint8)

    def render_rgb(self, data):
        """-> HxWx3 uint8 (RGB). The returned array is a fresh copy."""
        r = self._r
        r.update_scene(data, camera=self.cam)
        r._gl_context.make_current()
        mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, r._mjr_context)
        r.render(out=self._rgb_buf)
        return self._rgb_buf.copy()

    def render_segmentation(self, data):
        """-> HxW int32 geom id per pixel, -1 where there is no geom (sky/background)."""
        r = self._r
        r.update_scene(data, camera=self.cam)
        sc = r._scene
        F = sc.flags
        orig = F.copy()
        F[mujoco.mjtRndFlag.mjRND_SEGMENT] = True
        F[mujoco.mjtRndFlag.mjRND_IDCOLOR] = True
        r._gl_context.make_current()
        mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, self._seg_ctx)
        mujoco.mjr_render(self._rect, sc, self._seg_ctx)
        mujoco.mjr_readPixels(self._seg_buf, None, self._rect, self._seg_ctx)
        np.copyto(F, orig)
        self._code4[..., :3] = self._seg_buf            # RGB -> little-endian uint32 id code
        code = self._code4.view(np.uint32)[::-1, :, 0]   # GL origin is bottom-left
        n = sc.ngeom
        lut = np.full(n + 1, -1, np.int32)               # segid+1 -> geom id; 0 = background
        for g in sc.geoms[:n]:
            if g.segid != -1 and g.objtype == mujoco.mjtObj.mjOBJ_GEOM:
                lut[g.segid + 1] = g.objid
        return lut[code]

    def close(self):
        self._r.close()
