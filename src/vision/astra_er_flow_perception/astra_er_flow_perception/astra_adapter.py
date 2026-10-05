import numpy as np


class AstraAdapter:
    """Register a color image into the Astra depth-image coordinate system."""

    def __init__(
        self,
        depth_intrinsics,
        color_intrinsics,
        R_depth_to_color,
        t_depth_to_color,
        depth_width=640,
        depth_height=480,
        color_width=640,
        color_height=480,
        depth_scale=0.001,
    ):
        self.depth_width = int(depth_width)
        self.depth_height = int(depth_height)
        self.color_width = int(color_width)
        self.color_height = int(color_height)
        self.depth_scale = float(depth_scale)

        self.fx_d = float(depth_intrinsics["fx"])
        self.fy_d = float(depth_intrinsics["fy"])
        self.cx_d = float(depth_intrinsics["cx"])
        self.cy_d = float(depth_intrinsics["cy"])

        self.fx_c = float(color_intrinsics["fx"])
        self.fy_c = float(color_intrinsics["fy"])
        self.cx_c = float(color_intrinsics["cx"])
        self.cy_c = float(color_intrinsics["cy"])

        self.R_dc = np.asarray(R_depth_to_color, dtype=np.float32).reshape(3, 3)
        self.t_dc = np.asarray(t_depth_to_color, dtype=np.float32).reshape(3)

        u, v = np.meshgrid(
            np.arange(self.depth_width, dtype=np.float32),
            np.arange(self.depth_height, dtype=np.float32),
        )
        self.x_ray = (u - self.cx_d) / self.fx_d
        self.y_ray = (v - self.cy_d) / self.fy_d

    def depth_pixel_to_color_pixel(self, u_d, v_d, depth_raw):
        if depth_raw <= 0:
            return None

        z_d = float(depth_raw) * self.depth_scale
        x_d = (float(u_d) - self.cx_d) * z_d / self.fx_d
        y_d = (float(v_d) - self.cy_d) * z_d / self.fy_d

        p_d = np.array([x_d, y_d, z_d], dtype=np.float32)
        p_c = self.R_dc @ p_d + self.t_dc

        x_c, y_c, z_c = p_c
        if z_c <= 1e-6:
            return None

        u_c = self.fx_c * x_c / z_c + self.cx_c
        v_c = self.fy_c * y_c / z_c + self.cy_c

        if not (0 <= u_c < self.color_width and 0 <= v_c < self.color_height):
            return None
        return float(u_c), float(v_c)

    def align_color_to_depth(self, depth_image, color_image):
        if depth_image.shape[:2] != (self.depth_height, self.depth_width):
            raise ValueError(
                f"Expected depth {self.depth_width}x{self.depth_height}, "
                f"got {depth_image.shape[1]}x{depth_image.shape[0]}"
            )
        if color_image.shape[:2] != (self.color_height, self.color_width):
            raise ValueError(
                f"Expected color {self.color_width}x{self.color_height}, "
                f"got {color_image.shape[1]}x{color_image.shape[0]}"
            )

        z_d = depth_image.astype(np.float32) * self.depth_scale
        valid_depth = z_d > 0

        x_d = self.x_ray * z_d
        y_d = self.y_ray * z_d
        points_d = np.stack((x_d, y_d, z_d), axis=-1)

        points_c = points_d @ self.R_dc.T + self.t_dc
        x_c = points_c[..., 0]
        y_c = points_c[..., 1]
        z_c = points_c[..., 2]

        valid_z = z_c > 1e-6
        safe_z = np.where(valid_z, z_c, 1.0)

        u_c = self.fx_c * x_c / safe_z + self.cx_c
        v_c = self.fy_c * y_c / safe_z + self.cy_c

        u_ci = np.rint(u_c).astype(np.int32)
        v_ci = np.rint(v_c).astype(np.int32)

        valid = (
            valid_depth & valid_z
            & (u_ci >= 0) & (u_ci < self.color_width)
            & (v_ci >= 0) & (v_ci < self.color_height)
        )

        aligned_color = np.zeros(
            (self.depth_height, self.depth_width, 3), dtype=color_image.dtype
        )
        aligned_color[valid] = color_image[v_ci[valid], u_ci[valid]]
        valid_mask = valid.astype(np.uint8) * 255
        return aligned_color, valid_mask
