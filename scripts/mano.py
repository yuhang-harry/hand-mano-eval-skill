"""Right-hand MANO joint FK and tip LBS from external licensed arrays.
Native45 axis-angle, flat hand mean, wrist-centred metres; thumb-first output.
"""
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
MANO_IDS = np.array([[13,14,15],[1,2,3],[4,5,6],[10,11,12],[7,8,9]])
TIP_IDS = np.array([745,317,444,556,673])

class ManoMotionReference:
    """Broadcast beta (...,10) and theta (...,45); no PCA or hand-mean offset."""
    def __init__(self, arrays_path):
        self.path = Path(arrays_path)
        with np.load(self.path) as z:
            self.z = {k: z[k] for k in z.files}
        z = self.z
        ids = z['kintree_table'][1].tolist()
        by_id = {int(v): i for i, v in enumerate(ids)}
        self.parents = np.array([-1] + [by_id[int(p)] for p in z['kintree_table'][0, 1:]])
        if self.parents.tolist() != [-1, 0, 1, 2, 0, 4, 5, 0, 7, 8, 0, 10, 11, 0, 13, 14]:
            raise ValueError('Unexpected MANO native topology')
        self.j0 = np.einsum('jv,vc->jc', z['J_regressor'], z['v_template'])
        self.jd = np.einsum('jv,vci->jci', z['J_regressor'], z['shapedirs'])

    def forward(self, beta, theta45, return_vertices=False):
        """Return joints16, internal15 (...,5,3,3), tips5; optional vertices778."""
        beta, theta = np.asarray(beta, dtype=float), np.asarray(theta45, dtype=float)
        if beta.shape[-1:] != (10,) or theta.shape[-1:] != (45,):
            raise ValueError('Expected beta[...,10], theta45[...,45]')
        if not np.all(np.isfinite(beta)) or not np.all(np.isfinite(theta)):
            raise ValueError('MANO inputs must be finite')
        shape = np.broadcast_shapes(beta.shape[:-1], theta.shape[:-1])
        b = np.broadcast_to(beta, shape + (10,)).reshape(-1, 10)
        t = np.broadcast_to(theta, shape + (45,)).reshape(-1, 15, 3).copy()
        n = len(b)
        joints = self.j0[None] + np.einsum('jci,bi->bjc', self.jd, b)
        wrist = joints[:, :1].copy()
        joints -= wrist
        rot = np.broadcast_to(np.eye(3), (n, 16, 3, 3)).copy()
        rot[:, 1:] = Rotation.from_rotvec(t.reshape(-1, 3)).as_matrix().reshape(n, 15, 3, 3)
        # Translation of child j is its rest offset, unaffected by j's own rotation.
        global_rot = np.empty_like(rot)
        posed_j = np.zeros_like(joints)
        global_rot[:, 0] = np.eye(3)
        for j in range(1, 16):
            p = self.parents[j]
            global_rot[:, j] = global_rot[:, p] @ rot[:, j]
            posed_j[:, j] = posed_j[:, p] + np.einsum('bij,bj->bi', global_rot[:, p], joints[:, j] - joints[:, p])
        # Skinning only selected vertices unless the caller requests the full mesh.
        vi = np.arange(778) if return_vertices else TIP_IDS
        v = self.z['v_template'][vi][None] + np.einsum('vci,bi->bvc', self.z['shapedirs'][vi], b) - wrist
        feature = (rot[:, 1:] - np.eye(3)).reshape(n, 135)
        v += np.einsum('vcp,bp->bvc', self.z['posedirs'][vi], feature)
        trans = posed_j - np.einsum('bjcd,bjd->bjc', global_rot, joints)
        vv = np.einsum('vj,bjcd,bvd->bvc', self.z['weights'][vi], global_rot, v) + np.einsum('vj,bjc->bvc', self.z['weights'][vi], trans)
        tips = vv[:, TIP_IDS] if return_vertices else vv
        out = dict(joints16=posed_j.reshape(shape + (16, 3)),
                   internal15=posed_j[:, MANO_IDS].reshape(shape + (5, 3, 3)),
                   tips5=tips.reshape(shape + (5, 3)))
        if return_vertices:
            out['vertices778'] = vv.reshape(shape + (778, 3))
        return out

    def points(self, beta, theta45, with_tips=False):
        """Flatten thumb-first per-finger internal centres, optionally followed by tip."""
        out = self.forward(beta, theta45)
        p = out['internal15']
        if with_tips:
            p = np.concatenate([p, out['tips5'][..., :, None, :]], axis=-2)
        return p.reshape(p.shape[:-3] + ((20 if with_tips else 15), 3))

