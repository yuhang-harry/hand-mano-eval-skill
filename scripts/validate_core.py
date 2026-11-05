"""Synthetic physical-chain checks; no vendor or MANO assets required."""
import tempfile
from pathlib import Path
import numpy as np
from robot import MotionRobot
from metrics import self_check


def main():
    self_check()
    xml=['<robot name="synthetic"><link name="base"/>'];groups=[];tips=[]
    for f in range(5):
        parent='base';digit=[]
        for j in range(3):
            name=f'f{f}_j{j}';child=f'f{f}_l{j}';digit.append(name)
            offset=f'{f*.02} 0 0'if j==0 else '0 .03 0'
            mimic=f'<mimic joint="f{f}_j1" multiplier="0.5" offset="0.1"/>'if j==2 else ''
            xml.append(f'<link name="{child}"/><joint name="{name}" type="revolute"><parent link="{parent}"/><child link="{child}"/><origin xyz="{offset}"/><axis xyz="1 0 0"/><limit lower="-1" upper="1" effort="1" velocity="1"/>{mimic}</joint>')
            parent=child
        tip=f'f{f}_tip';tips.append(tip);groups.append(digit)
        xml.append(f'<link name="{tip}_link"/><joint name="{tip}" type="fixed"><parent link="{parent}"/><child link="{tip}_link"/><origin xyz="0 .02 0"/></joint>')
    xml.append('</robot>')
    with tempfile.TemporaryDirectory()as td:
        path=Path(td)/'synthetic.urdf';path.write_text(''.join(xml))
        cfg=dict(name='synthetic',urdf=str(path),landmark_joints=groups,tip_joints=tips)
        maxerr=0.
        for tip in (False,True):
            m=MotionRobot(cfg,include_tips=tip);x=(m.low+m.high)/2+.05;J=m.jacobian(x)
            for col in range(len(m.names)):
                h=1e-6;xp=x.copy();xm=x.copy();xp[col]+=h;xm[col]-=h
                maxerr=max(maxerr,float(np.max(abs(J[:,:,col]-(m.points(xp)-m.points(xm))/(2*h)))))
            assert all(m.bounds[n][0]<=v<=m.bounds[n][1]for n,v in m.qdict(x).items())
            assert all(abs(m.allq(x)[f'f{f}_j2']-(.5*m.qdict(x)[f'f{f}_j1']+.1))<1e-12 for f in range(5))
        assert maxerr<1e-8
    print(f'Physical synthetic chain, mimic and finite-difference Jacobian passed: {maxerr:.3g} m/rad')

if __name__=='__main__':main()
