"""One camera frame plus whatever the robot knew when it was taken."""
from dataclasses import dataclass, field
import io
import time

import numpy as np
from PIL import Image, ImageOps

VLM_MAX_SIDE = 1568   # long side sent to the vision model
SHARPNESS_SIDE = 480  # long side used for the blur measure


@dataclass
class Frame:
    image: Image.Image
    name: str = ""
    stamp: float = field(default_factory=time.time)
    pose: tuple = None      # robot (x, y, yaw) in the map frame, if localisation is available
    points: object = None   # lidar points, Nx3 array in the lidar frame, if available
    depth: object = None    # depth image in metres aligned to `image`, HxW array, if available
    camera: object = None   # the colour camera's Calibration, when the source knows it
    pose_frame: str = None  # "map": pose from localisation in a saved map; "odometry": a frame of this run only

    @classmethod
    def from_bytes(cls, data, **kwargs):
        image = Image.open(io.BytesIO(data))
        image.load()
        return cls(ImageOps.exif_transpose(image).convert("RGB"), **kwargs)

    @classmethod
    def from_file(cls, path, **kwargs):
        with open(path, "rb") as handle:
            return cls.from_bytes(handle.read(), name=kwargs.pop("name", str(path).rsplit("/", 1)[-1]), **kwargs)

    def jpeg(self, max_side=VLM_MAX_SIDE, quality=85):
        image = self.image
        if max(image.size) > max_side:
            image = image.copy()
            image.thumbnail((max_side, max_side))
        buffer = io.BytesIO()
        image.save(buffer, "JPEG", quality=quality)
        return buffer.getvalue()

    def sharpness(self):
        """Variance of the Laplacian on a small grey copy: low values mean motion blur."""
        grey = self.image.convert("L")
        grey.thumbnail((SHARPNESS_SIDE, SHARPNESS_SIDE))
        g = np.asarray(grey, dtype=np.float32)
        laplacian = g[1:-1, :-2] + g[1:-1, 2:] + g[:-2, 1:-1] + g[2:, 1:-1] - 4 * g[1:-1, 1:-1]
        return float(laplacian.var())
