"""Explicit-point physical URDF FK/Jacobian. No model assets or anatomical mapping inference."""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
MOVING = {"revolute", "continuous", "prismatic"}

def origin(el):
    t = np.eye(4)
    if el is not None:
        t[:3, :3] = Rotation.from_euler("xyz", np.fromstring(el.get("rpy", "0 0 0"), sep=" ")).as_matrix()
        t[:3, 3] = np.fromstring(el.get("xyz", "0 0 0"), sep=" ")
    return t


class PhysicalModel:
    def __init__(self, path, landmark_names, reference_q):
        root = ET.parse(path).getroot()
        elements = root.findall("joint")
        by_name = {j.get("name"): j for j in elements}
        children = {j.find("child").get("link"): j.get("name") for j in elements}
        self.all_independent = [j.get("name") for j in elements if j.get("type") in MOVING and j.find("mimic") is None]
        self.q_ref = {n: float(reference_q.get(n, 0.)) for n in self.all_independent}
        self.flattened_landmarks = [x for digit in landmark_names for x in digit]
        if len(self.flattened_landmarks) != 15:
            raise ValueError("requires exactly 15 supplied skeleton landmarks")
        ancestor_sets = []
        for name in self.flattened_landmarks:
            # A joint's own rotation does not displace its own axis origin.
            upstream = by_name[name].find("parent").get("link")
            ancestors = set()
            while upstream in children:
                jn = children[upstream]
                ancestors.add(jn)
                upstream = by_name[jn].find("parent").get("link")
            ancestor_sets.append(ancestors)

        def source(name, history=()):
            if name in history:
                raise ValueError("mimic cycle")
            j = by_name[name]
            mimic = j.find("mimic")
            if mimic is None:
                return name, 1., 0.
            base, factor, offset = source(mimic.get("joint"), history + (name,))
            multiplier, bias = float(mimic.get("multiplier", "1")), float(mimic.get("offset", "0"))
            return base, multiplier * factor, multiplier * offset + bias

        self.sources = {j.get("name"): source(j.get("name")) for j in elements if j.get("type") in MOVING}
        bounds = {n: [-np.inf, np.inf] for n in self.all_independent}
        for j in elements:
            if j.get("type") not in MOVING:
                continue
            lim = j.find("limit")
            if j.get("type") == "continuous":
                lower, upper = -np.pi, np.pi
            elif lim is None:
                raise ValueError("missing physical joint limits: " + j.get("name"))
            else:
                lower, upper = float(lim.get("lower")), float(lim.get("upper"))
            base, factor, offset = self.sources[j.get("name")]
            if factor == 0:
                if not lower <= offset <= upper:
                    raise ValueError("constant mimic violates limit")
                continue
            interval = sorted(((lower - offset) / factor, (upper - offset) / factor))
            bounds[base][0] = max(bounds[base][0], interval[0])
            bounds[base][1] = min(bounds[base][1], interval[1])
        self.bounds = bounds
        # Remove joint angles absorbed entirely by the one global rigid fit,
        # and terminal joint rotations that do not move any selected center.
        affected = {n: set() for n in self.all_independent}
        for k, ancestors in enumerate(ancestor_sets):
            for an in ancestors:
                if an in self.sources:
                    affected[self.sources[an][0]].add(k)
        self.names = [n for n in self.all_independent if 0 < len(affected[n]) < 15 and bounds[n][1] > bounds[n][0]]
        self.low = np.array([bounds[n][0] for n in self.names])
        self.high = np.array([bounds[n][1] for n in self.names])
        if not np.isfinite(self.low).all() or not np.isfinite(self.high).all():
            raise ValueError("unbounded physical joint range")
        for name in self.q_ref:
            self.q_ref[name] = float(np.clip(self.q_ref[name], *bounds[name]))
        self.reference_x = np.array([self.q_ref[n] for n in self.names])
        available_links = {l.get("name") for l in root.findall("link")} - set(children)
        self.root_links = available_links
        pending = elements.copy()
        ordered = []
        while pending:
            next_pending = []
            for j in pending:
                pa, ch = j.find("parent").get("link"), j.find("child").get("link")
                if pa not in available_links:
                    next_pending.append(j)
                    continue
                axis_el = j.find("axis")
                axis = np.fromstring(axis_el.get("xyz", "1 0 0") if axis_el is not None else "1 0 0", sep=" ")
                if j.get("type") in MOVING:
                    axis = axis / np.linalg.norm(axis)
                ordered.append((j.get("name"), pa, ch, j.get("type"), origin(j.find("origin")), axis))
                available_links.add(ch)
            if len(next_pending) == len(pending):
                raise ValueError("disconnected tree")
            pending = next_pending
        self.ordered = ordered
        self.mimic_count = sum(j.find("mimic") is not None for j in elements)
        self.path = path

    def qdict(self, x):
        q = dict(self.q_ref)
        q.update(zip(self.names, map(float, x)))
        return q

    def points(self, x):
        q = self.qdict(x)
        poses = {name: np.eye(4) for name in self.root_links}
        centers = {}
        for name, parent, child, kind, local, axis in self.ordered:
            pose = poses[parent] @ local
            centers[name] = pose[:3, 3].copy()
            if kind in MOVING:
                base, factor, offset = self.sources[name]
                value = q[base] * factor + offset
                if kind == "prismatic":
                    pose[:3, 3] += pose[:3, :3] @ (axis * value)
                else:
                    pose[:3, :3] = pose[:3, :3] @ Rotation.from_rotvec(axis * value).as_matrix()
            poses[child] = pose
        return np.array([centers[n] for n in self.flattened_landmarks])


class MotionRobot(PhysicalModel):
    def __init__(self, cfg, include_tips=False):
        self.cfg = cfg
        self.name = cfg.get("name", "unnamed physical model")
        groups = cfg["landmark_joints"]
        landmarks = list(groups) if all(isinstance(n, str) for n in groups) else [n for group in groups for n in group]
        if include_tips:
            if len(landmarks) != 15:
                raise ValueError("include_tips requires 15 internal landmarks")
            if cfg.get("tip_exclude", False):
                raise ValueError("tip semantics are explicitly unverified for " + self.name)
            tips = cfg.get("tip_joints", [])
            if len(tips) != 5:
                raise ValueError("five verified tip_joints are required for " + self.name)
            landmarks = [n for i in range(5) for n in [*landmarks[3*i:3*i+3], tips[i]]]
        if len(landmarks) not in (15, 20) or not all(isinstance(n, str) for n in landmarks):
            raise ValueError("landmark_joints must specify 15 or 20 joint names")
        path = Path(cfg["urdf"])
        if not path.is_absolute():
            path = Path.cwd() / path
        # PhysicalModel owns the parsed URDF, source-chain composition, merged
        # physical/mimic limits and reference clipping. Its neutral-only 15-point
        # variable selection is replaced below using the complete target set.
        parsing_landmarks = [landmarks[i:i+3] for i in range(0, 15, 3)]
        super().__init__(path, parsing_landmarks, cfg.get("reference_q", {}))
        self.flattened_landmarks = landmarks
        self.n_points = len(landmarks)
        joint_index = {entry[0]: i for i, entry in enumerate(self.ordered)}
        missing = set(landmarks) - joint_index.keys()
        if missing:
            raise ValueError("unknown landmark joints: " + ", ".join(sorted(missing)))
        self._point_joint_index = np.array([joint_index[n] for n in landmarks], dtype=int)
        child_joint = {entry[2]: i for i, entry in enumerate(self.ordered)}
        ancestors = []
        for n in landmarks:
            parent = self.ordered[joint_index[n]][1]
            upstream = set()
            while parent in child_joint:
                ji = child_joint[parent]
                upstream.add(ji)
                parent = self.ordered[ji][1]
            ancestors.append(upstream)
        common_ancestors = set.intersection(*ancestors)
        influence = {n: [] for n in self.all_independent}
        for ji, entry in enumerate(self.ordered):
            jn, _, _, kind, _, _ = entry
            if kind not in MOVING:
                continue
            base, factor, _ = self.sources[jn]
            indices = np.array([k for k, upstream in enumerate(ancestors) if ji in upstream], dtype=int)
            if factor != 0. and len(indices):
                influence[base].append((ji, factor, indices, kind))
        self.names = []
        self.frozen = {}
        for name in self.all_independent:
            lo, hi = self.bounds[name]
            entries = influence[name]
            if hi <= lo:
                self.frozen[name] = "zero-width merged physical/mimic interval"
            elif not entries:
                self.frozen[name] = "does not move any selected joint-axis origin"
            elif all(entry[0] in common_ancestors for entry in entries):
                self.frozen[name] = "whole-hand common-upstream rigid motion fixed at reference_q"
            else:
                self.names.append(name)
        self.low = np.array([self.bounds[n][0] for n in self.names])
        self.high = np.array([self.bounds[n][1] for n in self.names])
        self.reference_x = np.array([self.q_ref[n] for n in self.names])
        self._influences = [(col, *entry) for col, name in enumerate(self.names) for entry in influence[name]]
        self._independent_index = {n: i for i, n in enumerate(self.all_independent)}
        self._active_independent_index = np.array([self._independent_index[n] for n in self.names], dtype=int)
        self._reference_values = np.array([self.q_ref[n] for n in self.all_independent])
        links = sorted(self.root_links)
        link_index = {n: i for i, n in enumerate(links)}
        for entry in self.ordered:
            if entry[2] not in link_index:
                link_index[entry[2]] = len(link_index)
        self._n_links = len(link_index)
        self._compiled = []
        for name, parent, child, kind, local, axis in self.ordered:
            if kind in MOVING:
                base, factor, offset = self.sources[name]
                source = self._independent_index[base]
                ax, ay, az = axis
                cross = np.array([[0., -az, ay], [az, 0., -ax], [-ay, ax, 0.]])
                cross2 = cross @ cross
            else:
                source, factor, offset, cross, cross2 = -1, 0., 0., None, None
            self._compiled.append((link_index[parent], link_index[child], kind,
                                   local[:3, :3], local[:3, 3], axis,
                                   source, factor, offset, cross, cross2))
        self._cache_x = None
        self._cache_points = None
        self._cache_jacobian = None
        self._cache_centers = None
        self._cache_axes = None

    def _x(self, x):
        x = np.asarray(x, dtype=float)
        if x.shape != (len(self.names),) or not np.isfinite(x).all():
            raise ValueError("expected finite optimizer vector of shape " + str((len(self.names),)))
        return x

    def qdict(self, x):
        x = self._x(x)
        q = dict(self.q_ref)
        q.update(zip(self.names, map(float, x)))
        return q

    def allq(self, x):
        independent = self.qdict(x)
        return {name: independent[base]*factor + offset for name, (base, factor, offset) in self.sources.items()}

    def _evaluate(self, x):
        x = self._x(x)
        if self._cache_x is not None and np.array_equal(x, self._cache_x):
            return
        q = self._reference_values.copy()
        q[self._active_independent_index] = x
        rotations = np.broadcast_to(np.eye(3), (self._n_links, 3, 3)).copy()
        translations = np.zeros((self._n_links, 3))
        centers = np.empty((len(self._compiled), 3))
        axes = np.zeros_like(centers)
        eye = np.eye(3)
        for ji, (parent, child, kind, local_r, local_t, axis, source, factor, offset, cross, cross2) in enumerate(self._compiled):
            rotation = rotations[parent] @ local_r
            position = translations[parent] + rotations[parent] @ local_t
            centers[ji] = position
            if kind in MOVING:
                axes[ji] = rotation @ axis
                value = q[source]*factor + offset
                if kind == "prismatic":
                    position = position + axes[ji]*value
                else:
                    rotation = rotation @ (eye + np.sin(value)*cross + (1.-np.cos(value))*cross2)
            rotations[child] = rotation
            translations[child] = position
        self._cache_x = x.copy()
        self._cache_centers = centers
        self._cache_axes = axes
        self._cache_points = centers[self._point_joint_index]
        self._cache_jacobian = None

    def points(self, x):
        self._evaluate(x)
        return self._cache_points.copy()

    def jacobian(self, x):
        self._evaluate(x)
        if self._cache_jacobian is None:
            jac = np.zeros((self.n_points, 3, len(self.names)))
            for col, ji, factor, indices, kind in self._influences:
                axis = self._cache_axes[ji]
                if kind == "prismatic":
                    contribution = np.broadcast_to(axis, (len(indices), 3))
                else:
                    contribution = np.cross(axis, self._cache_points[indices] - self._cache_centers[ji])
                # Several physical ancestors may depend on the same source.
                # Their contributions must be summed, including mimic chains.
                jac[indices, :, col] += factor * contribution
            self._cache_jacobian = jac
        return self._cache_jacobian.copy()

    def joint_manifest(self):
        """Physical coordinate/limit inventory, suitable for an audit JSON."""
        return [{"name": name, "lower": float(self.bounds[name][0]),
                 "upper": float(self.bounds[name][1]), "reference": self.q_ref[name],
                 "optimized": name in self.names, "frozen_reason": self.frozen.get(name),
                 "dependent_moving_joints": [
                     {"joint": joint, "multiplier": factor, "offset": offset}
                     for joint, (base, factor, offset) in self.sources.items() if base == name]}
                for name in self.all_independent]

