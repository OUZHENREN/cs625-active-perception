#!/usr/bin/env python3
"""Phase-2b: single-shot ICP registration against the CAD template.

Why ICP instead of centroid + PCA: the workpiece is an open metal frame with
holes and specular patches, so first/second-order statistics drift with every
mask.  Registration uses the whole surface, and returns fitness / inlier RMSE
that double as a confidence score for the active-perception loop.

    colour + aligned depth -> YOLO-seg mask -> masked cloud (range gated)
      -> voxel downsample + normals
      -> PCA initial guess
      -> point-to-plane ICP against the template
      -> 6D pose + fitness + inlier RMSE
"""

from __future__ import annotations

import argparse
import json
import pathlib
import sys
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32MultiArray

sys.stdout.reconfigure(line_buffering=True)


class IcpPose(Node):
    def __init__(self, weights: str, template: str, conf: float,
                 min_range: float, max_range: float, voxel: float) -> None:
        super().__init__("icp_pose")
        import open3d as o3d
        from ultralytics import YOLO

        # The cell publishes several unconnected TF trees (tool0 is not even in
        # the robot tree), so a camera <- base_link lookup resolves to a wrong
        # tree and silently yields a pose metres away (measured: fitness 0.36,
        # xyz [-0.37, -0.92, 0.38] for a part that was 0.5 m in front of the
        # camera).  Use the same constant the rest of the chain uses:
        #     T_base_cam = T_base_tcp @ T_tcp_cam
        self.t_tcp_cam = None
        calib = pathlib.Path(__file__).resolve().parents[1] / "config/handeye/tcp_to_camera.json"
        if calib.is_file():
            self.t_tcp_cam = np.array(json.loads(calib.read_text())["T_tcp_cam"], dtype=float)
        self.tcp = None

        self.o3d = o3d
        self.model = YOLO(weights)
        self.conf = conf
        self.min_range = min_range
        self.max_range = max_range
        self.voxel = voxel
        self.bridge = CvBridge()
        self.color = None
        self.depth = None
        self.info = None
        self.done = False
        self.clip_radius = 0.35
        self.dump_projection = ""

        mesh = o3d.io.read_triangle_mesh(template)
        if len(mesh.triangles) > 0:
            sampled = mesh.sample_points_uniformly(number_of_points=30000)
            template_points = np.asarray(sampled.points)
        else:
            # RVS's own template (Model/box.ply) is a SCANNED POINT CLOUD, not a
            # CAD mesh: 182k points with no triangles.  Using it is the only way
            # for this pose to be directly comparable with the S3 message RVS
            # sends, because it fixes the same model frame RVS reports in.
            cloud_template = o3d.io.read_point_cloud(template)
            template_points = np.asarray(cloud_template.points)
            if len(template_points) == 0:
                raise SystemExit(f"cannot read template: {template}")
            if len(template_points) > 30000:
                pick = np.random.default_rng(0).choice(
                    len(template_points), 30000, replace=False)
                template_points = template_points[pick]
            self.get_logger().info(
                f"template is a point cloud: {len(template_points)} sampled points")
        # The senior's collision meshes (box_collision.STL / slot_collision.STL)
        # are stored in MILLIMETRES while the CAD exports are metres; ICP on a
        # 1000x template can only fail.  Normalise on the sampled extent.
        extent = float(np.linalg.norm(template_points.max(axis=0) - template_points.min(axis=0)))
        self.template_scale = 1.0
        if extent > 10.0:
            self.template_scale = 0.001
            template_points = template_points * 0.001
            sampled.scale(0.001, np.zeros(3))
        # Keep the template in ITS OWN frame.  The senior pipeline reports the
        # pose of the collision-model ORIGIN, which is not the centroid: their
        # box_collision.STL spans x[-0.038,0.076] y[-0.436,0.006] z[-0.459,0.006],
        # i.e. the origin sits on the corner/edge that RVS uses as the locating
        # point.  Centroid-centring the template (the previous version) silently
        # reported a pose ~0.2-0.3 m away from theirs - which is exactly the
        # "never matches RVS" symptom.  Instead of moving the template, offset
        # the INIT by the template centroid, which is algebraically the same
        # alignment but keeps the output in the model frame.
        self.template_centroid = template_points.mean(axis=0)
        # Built once: rebuilding a KDTree over 60k points for every candidate was
        # a large part of the 35 s spent in registration.
        self.template_tree = cKDTree(template_points)
        self.template_projection_points = template_points[::5]
        # --- anchor (branch) discriminator -------------------------------------
        # The part's face is almost square, so a 90/180 deg in-plane rotation fits
        # the observed surface just as well (measured: coverage 86 % vs 87 %, the
        # pose sits 0.3 m away).  The ONLY asymmetric feature is the cut corner:
        # of the four front-face corners, one is chamfered and the opposite is
        # square.  Check the model's SQUARE corner patch -- under a wrong branch it
        # lands where the real part is cut, so it has no observed support.
        corners = []
        low = template_points.min(axis=0); high = template_points.max(axis=0)
        for x in (low[0], high[0]):
            for y in (low[1], high[1]):
                for z in (low[2], high[2]):
                    corner = np.array([x, y, z])
                    distance, _ = self.template_tree.query(corner)
                    corners.append((float(distance), corner))
        # front face = the two corners with the smallest corner-to-cloud distance
        corners.sort(key=lambda item: item[0])
        anchor_corner = corners[0][1]
        patch = template_points[np.linalg.norm(template_points - anchor_corner, axis=1) < 0.06]
        self.anchor_patch = patch
        self.get_logger().info(
            f"anchor square corner {np.round(anchor_corner, 3).tolist()} "
            f"({corners[0][0] * 1000:.1f} mm gap vs {corners[-1][0] * 1000:.1f} mm), "
            f"patch {len(patch)} pts")
        self.get_logger().info(
            f"template largest extent {extent:.3f} (model units) -> scale {self.template_scale}; "
            f"model-frame centroid {np.round(self.template_centroid, 3).tolist()}")
        self.template = o3d.geometry.PointCloud()
        self.template.points = o3d.utility.Vector3dVector(template_points)
        self.template.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=30))

        self.create_subscription(Image, "/camera/color/image_raw", self.on_color, qos_profile_sensor_data)
        self.create_subscription(Image, "/camera/depth/image_raw", self.on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "/camera/color/camera_info", self.on_info, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "/tcp_pose_broadcaster/pose",
                                 self.on_tcp, 10)
        self.pose_pub = self.create_publisher(PoseStamped, "/perception/target_pose", 10)
        self.quality_pub = self.create_publisher(
            Float32MultiArray, "/perception/pose_quality", 10)
        self.get_logger().info("waiting for one colour+depth frame pair ...")

    def observed_silhouette(self, points_camera, height, width):
        """Silhouette of the CLEANED part cloud -- not the YOLO mask.

        The YOLO mask swallows the table under the part, which inflated the union
        and made every candidate score ~0.24, so it could not separate two poses
        0.25 m apart.
        """

        import cv2
        z = points_camera[:, 2]
        keep = z > 0.05
        u = self.info.k[0] * points_camera[keep, 0] / z[keep] + self.info.k[2]
        v = self.info.k[4] * points_camera[keep, 1] / z[keep] + self.info.k[5]
        image = np.zeros((height, width), dtype=np.uint8)
        ui = np.clip(u.astype(int), 0, width - 1)
        vi = np.clip(v.astype(int), 0, height - 1)
        image[vi, ui] = 1
        image = cv2.dilate(image, np.ones((7, 7), np.uint8))
        return image > 0

    def ppf_candidates(self, source):
        """Global candidates from OpenCV's PPF (point-pair-feature) matcher.

        This is the same family of algorithm RVS uses (Percipio3DMatching is a
        PPF + point-to-plane ICP matcher).  FPFH+RANSAC is weak on repetitive
        faces; PPF votes from oriented point pairs, which is what keeps it alive
        at oblique views where our deterministic seeds fail (+40 deg measured).

        Requires opencv-contrib-python: cv2.ppf_match_3d is absent from the plain
        opencv-python wheel.
        """

        import cv2
        if not hasattr(cv2, "ppf_match_3d"):
            self.get_logger().warn("cv2.ppf_match_3d unavailable (opencv-contrib missing)")
            return []
        # PPF cost grows with both clouds; on the full 600k-point scene it takes
        # minutes, so hand it a decimated pair (~8k points each).
        model_cloud = self.template.voxel_down_sample(0.012)
        scene_cloud = source.voxel_down_sample(0.012)
        model = np.asarray(model_cloud.points, dtype=np.float32)
        scene = np.asarray(scene_cloud.points, dtype=np.float32)
        if len(model) > 9000:
            model = model[np.random.default_rng(0).choice(len(model), 9000, replace=False)]
        if len(scene) > 9000:
            scene = scene[np.random.default_rng(1).choice(len(scene), 9000, replace=False)]
        self.get_logger().info(
            f"PPF input: model {len(model)} pts, scene {len(scene)} pts")
        try:
            detector = cv2.ppf_match_3d.PPF3DDetector(0.05, 0.05, 30)
            detector.trainModel(model)
            matches = detector.match(scene, 0.05, 0.05)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"PPF failed: {exc}")
            return []
        candidates = []
        for match in matches[:5]:
            matrix = np.asarray(match.pose, dtype=float)
            if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
                continue
            candidates.append(matrix)
        self.get_logger().info(f"PPF candidates: {len(candidates)}")
        return candidates

    def plane_seeds(self, source):
        """Deterministic init: align the model's dominant face with the observed face.

        The camera always sees the part's front face, so that face is the one
        constraint that is never ambiguous.  Aligning the model face normal to
        the observed face normal and matching their centroids lands inside the
        correct basin every time, which is what the random FPFH draws failed to
        do in 2 of 4 frames.
        """

        o3d = self.o3d
        observed = np.asarray(source.points)
        # segment_plane is RANSAC too: without a fixed seed it sometimes picks a
        # side face or the bracket instead of the part's front face, and the seed
        # then starts in the wrong basin.  Pin the draw and give it more tries.
        o3d.utility.random.seed(7)
        plane_model, inliers = source.segment_plane(
            distance_threshold=0.01, ransac_n=3, num_iterations=1500)
        normal_obs = np.asarray(plane_model[:3], dtype=np.float64)
        normal_obs /= np.linalg.norm(normal_obs)
        centroid_obs = observed[inliers].mean(axis=0)

        template_cloud = o3d.geometry.PointCloud()
        template_cloud.points = o3d.utility.Vector3dVector(
            np.asarray(self.template.points))
        o3d.utility.random.seed(7)
        model_plane, model_inliers = template_cloud.segment_plane(
            distance_threshold=0.01, ransac_n=3, num_iterations=1500)
        normal_model = np.asarray(model_plane[:3], dtype=np.float64)
        normal_model /= np.linalg.norm(normal_model)
        centroid_model = np.asarray(self.template.points)[model_inliers].mean(axis=0)

        seeds = []
        for sign in (1.0, -1.0):
            rotation, _ = R.align_vectors([normal_obs], [normal_model * sign])
            base_rotation = rotation.as_matrix()
            # Aligning the face normal fixes tilt only.  The part's face is
            # 441 x 465 mm, i.e. almost square, so the remaining in-plane yaw is
            # a four-fold ambiguity that ICP resolves differently each run.
            # Sweep it in 90 deg steps deterministically and let the physics
            # score pick the branch.
            for quarter in range(4):
                spin = R.from_rotvec(normal_obs * (quarter * np.pi / 2.0)).as_matrix()
                matrix = np.eye(4)
                matrix[:3, :3] = spin @ base_rotation
                matrix[:3, 3] = centroid_obs - matrix[:3, :3] @ centroid_model
                seeds.append(matrix)
        return seeds

    def score_candidate(self, transformation, source, silhouette, depth_m, obs_tree):
        """Resolve the plate-like ambiguity with physics, not just distances.

        coverage  : observed part points lying on the model (distances alone)
        iou       : projected model silhouette against the observed part silhouette
        violation : model material that sits IN FRONT of the measured surface --
                    a 180 deg flip or a slide along the face shows up here even
                    when the point distances look fine.
        """

        import cv2
        height, width = depth_m.shape
        points = np.asarray(self.template.points)
        projection_points = self.template_projection_points
        camera = projection_points @ transformation[:3, :3].T + transformation[:3, 3]
        front = camera[:, 2] > 0.05
        u = self.info.k[0] * camera[:, 0] / np.maximum(camera[:, 2], 1e-6) + self.info.k[2]
        v = self.info.k[4] * camera[:, 1] / np.maximum(camera[:, 2], 1e-6) + self.info.k[5]
        inside = front & (u >= 0) & (u < width) & (v >= 0) & (v < height)

        projected = np.zeros((height, width), dtype=np.uint8)
        ui = np.clip(u[inside].astype(int), 0, width - 1)
        vi = np.clip(v[inside].astype(int), 0, height - 1)
        projected[vi, ui] = 1
        projected = cv2.dilate(projected, np.ones((7, 7), np.uint8)) > 0
        union = np.logical_or(projected, silhouette).sum()
        iou = float(np.logical_and(projected, silhouette).sum() / max(union, 1))

        z_observed = depth_m[vi, ui]
        usable = z_observed > 0.05
        if usable.sum() > 100:
            violation = float(np.mean(camera[inside][usable][:, 2]
                                      < z_observed[usable] - 0.02))
        else:
            violation = 0.0

        inverse = np.linalg.inv(np.array(transformation, copy=True))
        observed_in_model = np.asarray(source.points) @ inverse[:3, :3].T + inverse[:3, 3]
        distances, _ = self.template_tree.query(observed_in_model, k=1)
        coverage = float((distances < 0.005).mean())

        # Anchor patch check: the model's SQUARE corner must have observed
        # support under this pose.  A wrong 90/180 deg branch puts it where the
        # real part is cut, so the patch finds no nearby observed point.
        patch = self.anchor_patch
        patch_camera = patch @ transformation[:3, :3].T + transformation[:3, 3]
        front = patch_camera[:, 2] > 0.05
        if front.sum() > 20:
            distances_anchor, _ = obs_tree.query(patch_camera[front], k=1)
            anchor_gap = float(np.median(distances_anchor))
        else:
            anchor_gap = 9.0

        score = coverage + iou - 1.5 * violation - 3.0 * max(0.0, anchor_gap - 0.01)
        return score, {"coverage": coverage, "iou": iou, "violation": violation,
                       "anchor_mm": anchor_gap * 1000.0}

    def register(self, source, init, silhouette, depth_m):
        """FPFH+RANSAC global init, then coarse-to-fine ICP; PCA init as fallback."""

        o3d = self.o3d
        point_to_plane = o3d.pipelines.registration.TransformationEstimationPointToPlane()
        criteria = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50)

        def refine_result(guess, scales):
            result = None
            for scale in scales:
                result = o3d.pipelines.registration.registration_icp(
                    self.template, source, scale, guess, point_to_plane, criteria)
                guess = np.array(result.transformation, copy=True)
            return guess

        features_source = o3d.pipelines.registration.compute_fpfh_feature(
            source, o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=100))
        features_template = o3d.pipelines.registration.compute_fpfh_feature(
            self.template, o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=100))
        ransac = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            self.template, source, features_template, features_source, True, 0.02,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(False), 3,
            [o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
             o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(0.02)],
            o3d.pipelines.registration.RANSACConvergenceCriteria(200000, 0.999))

        obs_tree = cKDTree(np.asarray(source.points))
        candidates = []
        # PPF is CPU-only in OpenCV and takes minutes on this machine; it is an
        # offline experiment switch, never part of the normal path.
        ppf = self.ppf_candidates(source) if self.use_ppf else []
        for matrix in ppf:
            candidates.append(refine_result(np.array(matrix, copy=True), (0.05, 0.02)))
        for seed in self.plane_seeds(source):
            candidates.append(refine_result(seed, (0.05, 0.02)))
        if ransac.fitness > 0.0:
            candidates.append(refine_result(np.array(ransac.transformation, copy=True), (0.05,0.02)))
        # One RANSAC draw is a lottery: in 2 of 3 frames it never produced the
        # pose that scores 1.64 (96 % coverage) while the 4-yaw PCA branch kept
        # winning at 61 %.  Draw a few times so the good candidate exists at all;
        # the physics score still decides which one is right.
        for attempt in range(1):
            o3d.utility.random.seed(attempt + 11)
            extra = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
                self.template, source, features_template, features_source, True, 0.02,
                o3d.pipelines.registration.TransformationEstimationPointToPoint(False), 3,
                [o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                 o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(0.02)],
                o3d.pipelines.registration.RANSACConvergenceCriteria(200000, 0.999))
            if extra.fitness > 0.0:
                candidates.append(refine_result(np.array(extra.transformation, copy=True), (0.05,0.02)))
        for yaw in (0.0, np.pi / 2, np.pi, 3 * np.pi / 2):
            guess = init.copy()
            guess[:3, :3] = R.from_euler("z", yaw).as_matrix() @ guess[:3, :3]
            guess[:3, 3] = self.cloud_centre - guess[:3, :3] @ self.template_centroid
            candidates.append(refine_result(guess, (0.05, 0.02)))

        # Two-stage refine: coarse ICP for every candidate, then the expensive
        # fine scales only for the three best.  Registration was 35 s of a 38 s
        # run; this plus the single template KDTree brings it to ~10 s.
        scored = []
        for candidate in candidates:
            if candidate is None:
                continue
            coarse = refine_result(np.array(candidate, copy=True), (0.05, 0.02))
            score, detail = self.score_candidate(
                coarse, source, silhouette, depth_m, obs_tree)
            scored.append((score, coarse, detail))
        scored.sort(key=lambda item: item[0], reverse=True)

        best = None
        best_score = -1.0
        for score, coarse, detail in scored[:5]:
            fine = refine_result(np.array(coarse, copy=True), (0.01, 0.005))
            score, detail = self.score_candidate(fine, source, silhouette, depth_m, obs_tree)
            self.get_logger().info(
                "candidate: coverage={coverage:.2f} iou={iou:.2f} "
                "violation={violation:.2f} anchor={anchor_mm:.0f}mm score={score:.2f}".format(
                    score=score, **detail))
            if score > best_score:
                best, best_score = fine, score
        if best is None:
            return None
        return best

    def fit_table_plane(self, depth_m, mask):
        """RANSAC on the pixels OUTSIDE the mask; the table is the dominant plane."""

        import cv2
        free = cv2.dilate(mask.astype(np.uint8), np.ones((9, 9), np.uint8)) == 0
        vs, us = np.nonzero(free & (depth_m > self.min_range / 2.0)
                            & (depth_m < self.max_range))
        if vs.size < 2000:
            return None
        fx, fy = self.info.k[0], self.info.k[4]
        cx, cy = self.info.k[2], self.info.k[5]
        zs = depth_m[vs, us]
        points = np.column_stack([(us - cx) * zs / fx, (vs - cy) * zs / fy, zs])
        # 800 RANSAC draws over ~1e6 points is minutes of numpy; subsample first.
        if len(points) > 120_000:
            pick = np.random.default_rng(0).choice(len(points), 120_000, replace=False)
            points = points[pick]
        rng = np.random.default_rng(0)
        best_count, best_normal, best_point = 0, None, None
        for _ in range(800):
            p0, p1, p2 = points[rng.choice(len(points), 3, replace=False)]
            normal = np.cross(p1 - p0, p2 - p0)
            length = np.linalg.norm(normal)
            if length < 1e-6:
                continue
            normal = normal / length
            count = int((np.abs((points - p0) @ normal) < 0.008).sum())
            if count > best_count:
                best_count, best_normal, best_point = count, normal, p0
        if best_normal is None or best_count < 0.1 * len(points):
            return None
        return best_normal, best_point

    def on_info(self, msg: CameraInfo) -> None:
        self.info = msg

    def on_tcp(self, msg) -> None:
        self.tcp = msg
        if not self.done:
            self.solve()

    def on_color(self, msg: Image) -> None:
        if not self.done:
            self.color = msg
            self.solve()

    def on_depth(self, msg: Image) -> None:
        if not self.done:
            self.depth = msg
            self.solve()

    def solve(self) -> None:
        import time as _time
        started = _time.time()
        # The TCP pose is part of the chain now, so a frame pair alone is not
        # enough; wait for all four so the first arrival does not abort.
        if (self.color is None or self.depth is None or self.info is None
                or self.tcp is None):
            return
        self.done = True
        import cv2

        color = self.bridge.imgmsg_to_cv2(self.color, desired_encoding="bgr8")
        depth = self.bridge.imgmsg_to_cv2(self.depth)
        depth_m = (depth.astype(np.float32) / 1000.0 if depth.dtype == np.uint16
                   else depth.astype(np.float32))
        height, width = depth_m.shape[:2]

        result = self.model.predict(color, imgsz=640, verbose=False,
                                    conf=self.conf, device="cpu")
        if not result or result[0].masks is None:
            self.get_logger().warn("no mask")
            return
        mask = result[0].masks.data[0].cpu().numpy()
        if mask.shape[:2] != (height, width):
            mask = cv2.resize(mask.astype(np.float32), (width, height),
                              interpolation=cv2.INTER_NEAREST)
        mask = mask > 0.5
        count, labels, stats, _ = cv2.connectedComponentsWithStats(
            mask.astype(np.uint8), connectivity=8)
        if count > 1:
            mask = labels == (1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA])))

        fx, fy = self.info.k[0], self.info.k[4]
        cx, cy = self.info.k[2], self.info.k[5]
        vs, us = np.nonzero(mask)
        zs = depth_m[vs, us]
        keep = np.isfinite(zs) & (zs > self.min_range) & (zs < self.max_range)
        if keep.sum() < 500:
            self.get_logger().warn(f"too few valid depth points: {int(keep.sum())}")
            return
        us, vs, zs = us[keep], vs[keep], zs[keep]
        cloud = np.column_stack([(us - cx) * zs / fx, (vs - cy) * zs / fy, zs])

        # The template is expressed in the robot base frame while the mask cloud
        # lives in the camera optical frame; ICP needs both in one frame.
        if self.t_tcp_cam is None:
            self.get_logger().warn(
                "config/handeye/tcp_to_camera.json is missing; run "
                "scripts/sync_real_handeye.py - refusing to guess the camera pose")
            return
        if self.tcp is None:
            self.get_logger().warn("no TCP pose yet; cannot place the camera")
            return
        position, orientation = self.tcp.pose.position, self.tcp.pose.orientation
        T_base_tcp = np.eye(4)
        T_base_tcp[:3, :3] = R.from_quat([orientation.x, orientation.y,
                                          orientation.z, orientation.w]).as_matrix()
        T_base_tcp[:3, 3] = [position.x, position.y, position.z]
        T_cam_base = np.linalg.inv(T_base_tcp @ self.t_tcp_cam)

        o3d = self.o3d
        # The trained mask swallows the table under the part (it does so even at
        # 69 % framing), and those coplanar points drag ICP towards the table.
        # Fit the table from the UNMASKED pixels and drop everything lying on it.
        plane = self.fit_table_plane(depth_m, mask)
        if plane is not None:
            normal, plane_point = plane
            off_plane = np.abs((cloud - plane_point) @ normal) > 0.015
            dropped = int(off_plane.size - off_plane.sum())
            cloud = cloud[off_plane]
            self.get_logger().info(
                f"table plane removed: {dropped} points off {off_plane.size}")
        else:
            self.get_logger().warn("no table plane found; ICP may include the table")

        # radius clip: the mask can bleed into the table, so keep only points
        # within the expected workpiece envelope around the median depth point.
        # This is the coarse->fine bridge until a dedicated fine-view stage lands.
        if self.clip_radius > 0.0:
            median = np.median(cloud, axis=0)
            dist = np.linalg.norm(cloud - median, axis=1)
            cloud = cloud[dist <= self.clip_radius]
        if len(cloud) < 500:
            self.get_logger().warn(f"clip left too few points: {len(cloud)}")
            return
        cloud = np.array(cloud, dtype=np.float64, copy=True)
        source = o3d.geometry.PointCloud()
        source.points = o3d.utility.Vector3dVector(cloud)
        source = source.voxel_down_sample(self.voxel)
        # The mask still carries background/handles-that-see-through points a few
        # hundred millimetres behind the part (measured: 15 % of points at
        # 165-822 mm residual).  They do not move the alignment (85 % of points
        # land within 4.4 mm) but they wreck an unweighted RMSE, so drop them.
        source, _ = source.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        # Table remnants and see-through pixels form their own 3D clusters; the
        # part is one big connected blob.  Keeping only the largest cluster is
        # what lifted the observed-point coverage from 64 % to 73 %.
        labels = np.array(source.cluster_dbscan(eps=0.02, min_points=10))
        if labels.max() >= 0:
            sizes = np.bincount(labels[labels >= 0])
            keep = labels == int(np.argmax(sizes))
            filtered = o3d.geometry.PointCloud()
            filtered.points = o3d.utility.Vector3dVector(
                np.asarray(source.points)[keep])
            source = filtered
            self.get_logger().info(
                f"largest cluster kept: {int(keep.sum())} of {keep.size} points, "
                f"{len(sizes)} clusters")
        source.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=30))
        if len(source.points) < 100:
            self.get_logger().warn("downsampled cloud too small")
            return

        remaining = np.asarray(source.points)
        centre = remaining.mean(axis=0)
        centred = remaining - centre
        cov = centred.T @ centred / max(len(centred) - 1, 1)
        values, vectors = np.linalg.eigh(cov)
        axes = vectors[:, np.argsort(values)[::-1]]
        if np.linalg.det(axes) < 0:
            axes[:, 2] *= -1.0
        init = np.eye(4)
        init[:3, :3] = axes
        # Place the template so its CENTROID lands on the observed cloud centroid,
        # while the output pose stays in the template's own frame.
        self.cloud_centre = centre
        init[:3, 3] = centre - init[:3, :3] @ self.template_centroid

        observed_silhouette = self.observed_silhouette(
            np.asarray(source.points), height, width)
        detect_done = _time.time()
        best = self.register(source, init, observed_silhouette, depth_m)
        if best is None:
            self.get_logger().warn("registration failed")
            return

        # Open3D's fitness is matched TEMPLATE points / all template points, and a
        # single view only ever sees ~30 % of the part, so it can never reach the
        # 0.5 acceptance bar that was written for RVS's own (multi-view) matcher.
        # The honest check for one static view is the reverse: how much of the
        # OBSERVED cloud lands on the model, and how close.
        # registration_icp(source=template, target=cloud) returns template->cloud,
        # so the inverse is what takes the observed points into the template frame.
        cloud_to_template = np.linalg.inv(np.array(best, copy=True))
        observed = np.asarray(source.points) @ cloud_to_template[:3, :3].T \
            + cloud_to_template[:3, 3]
        distances, _ = cKDTree(np.asarray(self.template.points)).query(observed, k=1)
        coverage = float((distances < 0.005).mean())          # observed -> model, 5 mm
        median = float(np.median(distances))
        outliers = float((distances > 0.02).mean())
        rmse_observed = float(np.sqrt((distances[distances < 0.02] ** 2).mean()))
        # Per-frame acceptance for a single static view.  Open3D's template
        # fitness cannot reach 0.5 (that number was written for RVS's multi-view
        # matcher; one view sees ~30-50 % of the part).  The observed->model
        # coverage is depressed by mask/table residue (26 % of points sit >20 mm
        # away even when the pose is visibly correct), so the honest per-frame
        # gate is the median residual; repeatability is checked separately by
        # scripts/check_pose_stability.py, which is what a pose must pass before
        # it is handed to the task layer.
        passed = median <= 0.005 and coverage >= 0.55

        # ICP gives T_camera_object; the task layer needs T_base_object.
        T_base_object = np.linalg.inv(T_cam_base) @ best

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "base_link"
        translation = T_base_object[:3, 3]
        quat = R.from_matrix(np.array(T_base_object[:3, :3], copy=True)).as_quat()
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = translation
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = quat
        self.pose_pub.publish(pose)

        if self.dump_projection:
            import cv2
            template_points = np.asarray(self.template.points)[::7]
            camera_points = template_points @ best[:3, :3].T + best[:3, 3]
            front = camera_points[:, 2] > 0.05
            uv = np.column_stack([
                self.info.k[0] * camera_points[:, 0] / np.maximum(camera_points[:, 2], 1e-6) + self.info.k[2],
                self.info.k[4] * camera_points[:, 1] / np.maximum(camera_points[:, 2], 1e-6) + self.info.k[5]])
            inside = front & (uv[:, 0] >= 0) & (uv[:, 0] < color.shape[1]) \
                & (uv[:, 1] >= 0) & (uv[:, 1] < color.shape[0])
            overlay = color.copy()
            for u, v in uv[inside]:
                cv2.circle(overlay, (int(u), int(v)), 1, (0, 0, 255), -1)
            cv2.imwrite(self.dump_projection, overlay)
            self.get_logger().info(f"projection written to {self.dump_projection}")

        quality = Float32MultiArray()
        quality.data = [coverage, median,
                        float(len(source.points)), float(len(cloud)),
                        coverage, rmse_observed]
        self.quality_pub.publish(quality)
        self.get_logger().info(
            "icp xyz=" + str(np.round(translation, 4).tolist())
            + " quat=" + str(np.round(quat, 4).tolist())
            + f" coverage={coverage * 100:.0f}%"
            + f" median={median * 1000:.1f}mm"
            + f" outliers={outliers * 100:.0f}%"
            + f" observed_rmse={rmse_observed * 1000:.1f}mm"
            + f" pts={len(cloud)}"
            + f" prep={detect_done - started:.1f}s total={_time.time() - started:.1f}s"
            + (" ICP_PASS" if passed else " ICP_FAIL"))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--weights", required=True)
    parser.add_argument("--template", required=True)
    parser.add_argument("--conf", type=float, default=0.4)
    parser.add_argument("--min-range", type=float, default=0.3)
    parser.add_argument("--max-range", type=float, default=2.2)
    parser.add_argument("--voxel", type=float, default=0.005)
    parser.add_argument("--clip-radius", type=float, default=0.0,
                        help="metres; drop mask pixels beyond this radius")
    parser.add_argument("--dump-projection", default="",
                        help="write an RGB image with the aligned template projected "
                             "onto it; the cheapest way to see whether the pose is right")
    parser.add_argument("--timeout", type=float, default=60.0)
    parser.add_argument("--ppf", action="store_true",
                        help="enable the slow OpenCV PPF candidate generator (offline only)")
    args = parser.parse_args()

    rclpy.init()
    node = IcpPose(args.weights, args.template, args.conf,
                   args.min_range, args.max_range, args.voxel)
    node.clip_radius = args.clip_radius
    node.use_ppf = args.ppf
    node.dump_projection = args.dump_projection
    deadline = time.time() + args.timeout
    while rclpy.ok() and not node.done and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
