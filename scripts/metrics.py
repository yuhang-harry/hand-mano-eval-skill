"""Explicit static/pose metrics; no model downloads or hidden registration.

CLI inputs documented in ../references/inputs.md. Length input: metres.
"""
import argparse
import itertools
import json
from pathlib import Path
import numpy as np


def descriptor20(points):
    p=np.asarray(points,dtype=float)
    if p.shape!=(5,3,3) or not np.isfinite(p).all():
        raise ValueError('expected finite thumb-first (5,3,3) centres')
    return np.r_[np.linalg.norm(np.diff(p,axis=1),axis=2).ravel(),
                 [np.linalg.norm(p[i,0]-p[j,0])for i,j in itertools.combinations(range(5),2)]]


def match_shapes(robot,human,weights=None):
    r,h=np.asarray(robot,float),np.asarray(human,float)
    if r.ndim!=1 or h.ndim!=2 or h.shape[1]!=len(r) or len(h)==0:
        raise ValueError('expected robot(D) and humans(B,D) with B>0')
    if not np.isfinite(r).all()or not np.isfinite(h).all()or np.any(r<0)or np.any(h<0):
        raise ValueError('distance features must be finite and nonnegative')
    w=np.ones(len(r))if weights is None else np.asarray(weights,float)
    if w.shape!=r.shape or not np.isfinite(w).all()or np.any(w<0)or not np.any(w>0):
        raise ValueError('weights must be finite, nonnegative and nonzero')
    denom=np.sum(w*h*h,axis=1);rnorm=np.sqrt(np.sum(w*r*r))
    if np.any(denom<=0)or rnorm<=0:raise ValueError('nonzero descriptors required')
    alpha=np.sum(w*h*r,axis=1)/denom
    if np.any(alpha<=0):raise ValueError('positive whole-hand scale required')
    residual=alpha[:,None]*h-r
    sse=np.sum(w*residual**2,axis=1);k=int(np.argmin(sse))
    fixed_sse=np.sum(w*(h-r)**2,axis=1);kf=int(np.argmin(fixed_sse))
    contributions=w*residual[k]**2
    return dict(beta_index=k,alpha_mano_to_robot=float(alpha[k]),
                shape_NRMSE_pct=float(100*np.sqrt(sse[k])/rnorm),
                distance_RMS_robot_mm=float(1000*np.sqrt(sse[k]/w.sum())),
                residual_features_robot_mm=(1000*residual[k]).tolist(),
                residual_SSE_share=(contributions/sse[k]).tolist()if sse[k]>0 else np.zeros(len(r)).tolist(),
                feature_weights=w.tolist(),robot_squared_energy_share=(w*r*r/(rnorm*rnorm)).tolist(),
                fixed_alpha1_beta_index=kf,
                fixed_alpha1_distance_RMS_robot_mm=float(1000*np.sqrt(fixed_sse[kf]/w.sum())))


def summarize(values,thresholds=(5,10,15),weights=None):
    v=np.asarray(values,float)
    if v.ndim!=1 or not len(v)or not np.isfinite(v).all():raise ValueError('finite nonempty score vector required')
    if any(not np.isfinite(t)or t<0 for t in thresholds):raise ValueError('nonnegative finite thresholds required')
    if weights is None:
        w=np.full(len(v),1/len(v));p95=float(np.percentile(v,95));method='numpy linear'
    else:
        w=np.asarray(weights,float)
        if w.shape!=v.shape or not np.isfinite(w).all()or np.any(w<0)or w.sum()<=0:raise ValueError('invalid pose weights')
        w=w/w.sum();order=np.argsort(v);idx=min(np.searchsorted(np.cumsum(w[order]),.95,side='left'),len(v)-1)
        p95=float(v[order[idx]]);method='inverse discrete weighted CDF'
    return dict(mean_mm=float(w@v),p95_mm=p95,p95_method=method,max_mm=float(v.max()),
                threshold_attainment_pct={str(float(t)):float(100*np.sum(w[v<=t]))for t in thresholds})


def score_poses(robot_m,target_m,neutral_robot_m,neutral_target_m,alpha,thresholds=(5,10,15),families=None):
    r,y,nr,ny=map(lambda a:np.asarray(a,float),(robot_m,target_m,neutral_robot_m,neutral_target_m))
    if r.ndim!=3 or r.shape!=y.shape or r.shape[-1]!=3 or r.shape[0]==0 or r.shape[1]==0 or nr.shape!=r.shape[1:]or ny.shape!=nr.shape:
        raise ValueError('expected poses(P,N,3) and neutral(N,3)')
    if not all(np.isfinite(a).all()for a in (r,y,nr,ny))or not np.isfinite(alpha)or alpha<=0:raise ValueError('finite points and positive alpha required')
    point_rms=lambda d:1000/alpha*np.sqrt(np.mean(np.sum(d*d,axis=-1),axis=-1))
    ab=point_rms(r-y);de=point_rms((r-nr)-(y-ny))
    out=dict(unit='MANO-equivalent mm (robot metres *1000 / frozen alpha)',alpha=float(alpha),n_poses=len(r),n_points=r.shape[1],
             neutral_RMS_mm=float(point_rms(nr-ny)),absolute=summarize(ab,thresholds),increment=summarize(de,thresholds),
             per_pose_absolute_mm=ab.tolist(),per_pose_increment_mm=de.tolist())
    if families is not None:
        f=np.asarray(families).astype(str)
        if f.shape!=(len(r),):raise ValueError('one family per pose required')
        unique,counts=np.unique(f,return_counts=True);freq=dict(zip(unique,counts));w=np.array([1/freq[k]for k in f])
        out['per_family']={k:dict(count=int(freq[k]),absolute=summarize(ab[f==k],thresholds),increment=summarize(de[f==k],thresholds))for k in unique}
        out['equal_family']=dict(absolute=summarize(ab,thresholds,w),increment=summarize(de,thresholds,w))
    return out


def self_check():
    r=np.array([.03,.04,.07]);h=np.stack([r,r*np.array([1.1,.9,1.])])
    a=match_shapes(1.37*r,h);assert a['beta_index']==0 and abs(a['alpha_mano_to_robot']-1.37)<1e-12 and a['shape_NRMSE_pct']<1e-12
    # A constant geometric offset must not become movement-increment error.
    y=np.arange(18).reshape(2,3,3)*.001;ny=np.zeros((3,3));offset=np.array([.002,-.001,.003]);r=y+offset;nr=ny+offset
    s=score_poses(r,y,nr,ny,2.,families=['a','b'])
    assert max(s['per_pose_increment_mm'])<1e-12
    assert abs(s['neutral_RMS_mm']-np.linalg.norm(offset)*500)<1e-12
    # Doubling coordinates and alpha must preserve MANO-equivalent errors.
    s2=score_poses(2*r,2*y,2*nr,2*ny,4.)
    assert np.allclose(s['per_pose_absolute_mm'],s2['per_pose_absolute_mm'])
    # Duplicating one family changes raw counts but not equal-family mean.
    z=summarize([0.,10.,10.],weights=[1.,.5,.5]);assert abs(z['mean_mm']-5)<1e-12
    print(json.dumps(dict(scale_recovery=True,neutral_offset_increment=True,unit_scaling=True,equal_family_weights=True)))


def main():
    p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='command',required=True)
    for name in ['static','poses']:
        q=sub.add_parser(name);q.add_argument('--input',type=Path,required=True);q.add_argument('--output',type=Path,required=True)
        if name=='poses':q.add_argument('--thresholds',type=float,nargs='+',default=[5,10,15])
    sub.add_parser('self-check');a=p.parse_args()
    if a.command=='self-check':return self_check()
    with np.load(a.input,allow_pickle=False)as z:
        if a.command=='static':
            out=match_shapes(z['robot_descriptor'],z['human_descriptors'],z['weights']if'weights'in z else None)
            if'betas'in z:out['beta']=z['betas'][out['beta_index']].tolist()
        else:out=score_poses(z['robot_m'],z['target_m'],z['neutral_robot_m'],z['neutral_target_m'],float(z['alpha']),a.thresholds,z['families']if'families'in z else None)
    a.output.write_text(json.dumps(out,indent=2,ensure_ascii=False)+'\n')

if __name__=='__main__':main()
