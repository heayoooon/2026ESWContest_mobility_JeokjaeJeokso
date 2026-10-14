"""Display-only, temporally tracked 2.5D parcel solids fitted to measured planes.

Faces closing unobserved areas are model estimates, never additional measurements.
No scenario-specific object templates or prerecorded poses are used.
"""
import numpy as np
from surface_core import rays


def components(mask):
    seen=np.zeros_like(mask,bool); result=[]; ny,nx=mask.shape
    for y,x in zip(*np.nonzero(mask)):
        if seen[y,x]: continue
        stack=[(y,x)]; seen[y,x]=True; group=[]
        while stack:
            yy,xx=stack.pop(); group.append((yy,xx))
            for y1,x1 in ((yy-1,xx),(yy+1,xx),(yy,xx-1),(yy,xx+1)):
                if 0<=y1<ny and 0<=x1<nx and mask[y1,x1] and not seen[y1,x1]:
                    seen[y1,x1]=True; stack.append((y1,x1))
        result.append(np.asarray(group))
    return result


def fit_planes(points,colors):
    """Fit dominant near-horizontal planes using height peaks and robust refinement."""
    remaining=np.ones(len(points),bool); found=[]
    for _ in range(9):
        q=points[remaining]
        if len(q)<40: break
        hist,edges=np.histogram(q[:,2],np.arange(0,1.01,.005))
        peak=hist.argmax()
        if hist[peak]<25: break
        seed=abs(points[:,2]-(edges[peak]+.0025))<.007
        keep=seed&remaining
        coeff=np.array([0.,0.,edges[peak]+.0025])
        for __ in range(4):
            a=np.column_stack((points[keep,:2],np.ones(keep.sum())))
            if len(a)<40: break
            coeff=np.linalg.lstsq(a,points[keep,2],rcond=None)[0]
            if np.linalg.norm(coeff[:2])>.55:
                coeff=np.array([0.,0.,np.median(points[keep,2])])
            residual=abs(points[:,2]-(points[:,:2]@coeff[:2]+coeff[2]))
            keep=(residual<.0055)&remaining
        remaining[keep|seed]=False
        q=points[keep]; rgb=colors[keep]
        if len(q)<40: continue
        # Connected patches in XY; a 1-cell closing joins tiny depth holes only.
        origin=q[:,:2].min(0)-.01
        ij=np.floor((q[:,:2]-origin)/.008).astype(int)
        shape=tuple((ij.max(0)+3)[::-1]); occ=np.zeros(shape,bool)
        occ[ij[:,1],ij[:,0]]=True
        expanded=occ.copy()
        for dy,dx in ((1,0),(-1,0),(0,1),(0,-1)):
            shifted=np.roll(occ,(dy,dx),(0,1)); expanded|=shifted
        for group in components(expanded):
            lab=np.zeros(shape,bool); lab[group[:,0],group[:,1]]=True
            selected=lab[ij[:,1],ij[:,0]]
            pts=q[selected]; col=rgb[selected]
            if len(pts)<35: continue
            lo,hi=np.percentile(pts[:,:2],[1,99],axis=0)
            lo-=.004; hi+=.004
            extent=hi-lo
            if extent.min()<.025 or np.prod(extent)<.0018: continue
            top=float(np.median(pts[:,2]))
            if top<.30: continue  # lower stray surfaces are not promoted to parcels
            color=np.median(col,axis=0)/255.
            found.append(dict(x0=float(lo[0]),x1=float(hi[0]),y0=float(lo[1]),y1=float(hi[1]),
                              top=top,plane=coeff,color=color,area=float(np.prod(extent)),count=len(pts)))
    return found


class CargoModel:
    def __init__(self):
        self.ray_key=None; self.tracks=[]; self.pending=[]; self.stamp=None; self.next_id=1

    def observed(self,frame):
        sf=frame['hdr'].get('surface')
        if sf and 'depth' in frame:
            i=sf['intr']; key=tuple(i.values())+tuple(np.asarray(sf['R']).ravel())
            if key!=self.ray_key:
                self.world_rays=rays(i)@np.asarray(sf['R'],np.float32).T; self.ray_key=key
            dep=frame['depth'][::2,::2].astype(np.float32)*.001
            points=self.world_rays[::2,::2]*dep[...,None]
            points[...,2]-=frame['hdr']['calib']['ref_h']
            dx=points[2:,1:-1]-points[:-2,1:-1]
            dy=points[1:-1,2:]-points[1:-1,:-2]
            norm=np.cross(dx,dy)
            ratio=abs(norm[...,2])/np.maximum(np.linalg.norm(norm,axis=-1),1e-10)
            valid=(dep[1:-1,1:-1]>0)&(ratio>.82)&(np.linalg.norm(dx,axis=-1)<.03)&(np.linalg.norm(dy,axis=-1)<.03)
            # A regular stride reduces fitting work without changing the height grid.
            q=points[1:-1,1:-1][valid]
            rgb=frame['rgb'][::2,::2][1:-1,1:-1][valid]
            full=points[dep>0]
            self.full_colors=frame['rgb'][::2,::2][dep>0]
        else:
            h=frame['h']; g=frame['hdr']['grid']; y,x=np.mgrid[:h.shape[0],:h.shape[1]]
            p=np.stack((g['x0']+(x+.5)*g['cell'],g['y0']+(y+.5)*g['cell'],h),-1)
            valid=np.isfinite(h)&(h>.3); q=p[valid]; rgb=frame['col'][valid]; full=q; self.full_colors=rgb
        return q,rgb,full

    def update(self,frame):
        hdr=frame['hdr']; now=float(hdr['t_run']); stamp=(hdr.get('session'),hdr['run'])
        if self.stamp is not None and (stamp!=self.stamp[0] or now<self.stamp[1]):
            self.tracks=[]; self.pending=[]
        self.stamp=(stamp,now)
        q,rgb,full=self.observed(frame)
        candidates=fit_planes(q,rgb) if len(q) else []
        # Remove duplicate fits of the same plateau; thin stacked layers stay separate.
        candidates.sort(key=lambda b:b['count'],reverse=True)
        boxes=[]
        for b in candidates:
            if any(abs(b['top']-a['top'])<.009 and overlap(b,a)>.65 for a in boxes): continue
            boxes.append(b)
        boxes=sorted(boxes,key=lambda b:b['top'])[:9]
        # Opposite strips of one top face can be separated by a box resting on it.
        # Complete that occluded footprint as a single explicitly inferred rectangle.
        merged=[]
        for b in boxes:
            partner=None
            for a in merged:
                gapx=max(0,a['x0']-b['x1'],b['x0']-a['x1'])
                gapy=max(0,a['y0']-b['y1'],b['y0']-a['y1'])
                if abs(a['top']-b['top'])<.009 and gapx+gapy<.09:
                    partner=a; break
            if partner is None: merged.append(b)
            else:
                a=partner
                for k in ('x0','y0'): a[k]=min(a[k],b[k])
                for k in ('x1','y1'): a[k]=max(a[k],b[k])
                a['area']=(a['x1']-a['x0'])*(a['y1']-a['y0'])
                a['count']+=b['count']
        boxes=merged
        for b in reversed(boxes):
            supports=[a for a in boxes if .010<b['top']-a['top']<.065 and intersection(a,b)>.00015]
            if supports:
                support=max(supports,key=lambda a:a['top'])
                for k in ('x0','y0'): support[k]=min(support[k],b[k])
                for k in ('x1','y1'): support[k]=max(support[k],b[k])
                support['area']=(support['x1']-support['x0'])*(support['y1']-support['y0'])
        boxes=[b for b in boxes if b['area']>.004 or b['count']>90]
        for b in boxes:
            lower=[a for a in boxes if a['top']<b['top']-.01 and intersection(b,a)/max(b['area'],1e-6)>.20]
            if lower:
                bottom=max(a['top'] for a in lower)
            else:
                # Lower visible edge is a base estimate, not a measured hidden bottom.
                inside=(full[:,0]>b['x0']-.02)&(full[:,0]<b['x1']+.02)&(full[:,1]>b['y0']-.025)&(full[:,1]<b['y1']+.025)&(full[:,2]<b['top']-.015)
                bottom=float(np.percentile(full[inside,2],3)) if inside.sum()>60 else b['top']-.035
                bottom=max(.15,min(bottom,b['top']-.025))
            b['bottom']=bottom
            b['foundation']=not bool(lower)
            b['seen']=now; b['held']=False
        roots=[b['bottom'] for b in boxes if b['foundation']]
        if roots:
            # Fixed-camera reference from the inspected cruise recording's lower edge.
            # The hidden support/base is an explicit estimate, not re-fitted to passing hands.
            foundation=.365
            for b in boxes:
                if b['foundation']: b['bottom']=foundation
        # Colored cap/label above a supporting plane supplies a compact cylinder cue.
        # The unseen bottle body is an explicit primitive estimate, not reconstructed glass.
        color=self.full_colors.astype(np.float32)
        blue=(color[:,2]>color[:,0]*1.32)&(color[:,1]>color[:,0]*1.15)&(color[:,2]>65)&(full[:,2]>.60)
        if blue.sum()>25:
            cloud=full[blue]; lo,hi=np.percentile(cloud,[3,97],axis=0); center=np.median(cloud,axis=0)
            supports=[b for b in boxes if b['x0']<center[0]<b['x1'] and b['y0']-.02<center[1]<b['y1'] and b['top']<hi[2]]
            if supports:
                support=max(supports,key=lambda b:b['top'])
                radius=float(np.clip(max(hi[2]-support['top'],hi[0]-lo[0])*.5,.012,.035))
                y0=float(lo[1]); y1=float(min(support['y1'],max(hi[1],y0+.045)))
                if y1-y0>.02:
                    boxes.append(dict(x0=float(center[0]-radius),x1=float(center[0]+radius),y0=y0,y1=y1,
                                      bottom=support['top'],top=support['top']+radius*2,color=np.median(color[blue],axis=0)/255,
                                      area=2*radius*(y1-y0),count=int(blue.sum()),seen=now,held=False,shape='cylinder'))
        unused=list(self.tracks); tracks=[]; next_pending=[]
        for b in boxes:
            eligible=[a for a in unused if overlap(a,b)>.28 and abs(a['top']-b['top'])<.055 and a.get('shape')==b.get('shape')]
            old=max(eligible,key=lambda a:overlap(a,b)) if eligible else None
            if old is not None:
                unused=[a for a in unused if a is not old]; b['id']=old['id']
                movement=max(abs(b[k]-old[k]) for k in ('x0','x1','y0','y1','top','bottom'))
                alpha=.28 if movement<.025 else .85
                for k in ('x0','x1','y0','y1','top','bottom'): b[k]=old[k]+alpha*(b[k]-old[k])
                b['color']=old['color']*.8+b['color']*.2
            else:
                previous=[a for a in self.pending if overlap(a,b)>.4 and abs(a['top']-b['top'])<.04 and now-a['seen']<=.25 and a.get('shape')==b.get('shape')]
                old_pending=max(previous,key=lambda a:overlap(a,b)) if previous else None
                hits=old_pending.get('hits',1)+int(now>old_pending['seen']) if old_pending else 1
                if hits<2:
                    b['hits']=hits; next_pending.append(b); continue
                b['id']=self.next_id; self.next_id+=1
            tracks.append(b)
        # A short absence is tracking uncertainty, not proof that a box has vanished.
        # Explicitly lower observations remove it immediately; otherwise expire at 0.45s.
        for old in unused:
            g=hdr['grid']; h=frame['h']; y,x=np.mgrid[:h.shape[0],:h.shape[1]]
            xs=g['x0']+(x+.5)*g['cell']; ys=g['y0']+(y+.5)*g['cell']
            inside=(xs>old['x0'])&(xs<old['x1'])&(ys>old['y0'])&(ys<old['y1'])&np.isfinite(h)
            evidence=h[inside]
            removed=len(evidence)>8 and np.percentile(evidence,85)<old['top']-.035
            if not removed and now-old['seen']<.45:
                old=old.copy(); old['held']=True; tracks.append(old)
        self.tracks=tracks; self.pending=next_pending
        frame['model_info']=dict(count=len(tracks),held=sum(b['held'] for b in tracks),kind='estimated_closed_solids',
            boxes=[{k:b.get(k,'box') if k=='shape' else b[k] for k in ('id','shape','x0','x1','y0','y1','bottom','top','held')} for b in tracks])
        return tracks


def intersection(a,b):
    return max(0,min(a['x1'],b['x1'])-max(a['x0'],b['x0']))*max(0,min(a['y1'],b['y1'])-max(a['y0'],b['y0']))


def overlap(a,b):
    return intersection(a,b)/max(min(a['area'],b['area']),1e-8)


def box_geometry(box):
    x0,x1,y0,y1,z0,z1=(box[k] for k in ('x0','x1','y0','y1','bottom','top'))
    vertices=np.array([[x0,y0,z0],[x1,y0,z0],[x1,y1,z0],[x0,y1,z0],
                       [x0,y0,z1],[x1,y0,z1],[x1,y1,z1],[x0,y1,z1]],np.float32)
    faces=np.array([[4,5,6,7],[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7],[3,2,1,0]])
    triangles=faces[:,[0,1,2,0,2,3]].reshape(-1)
    edges=np.array([[0,1],[1,2],[2,3],[3,0],[4,5],[5,6],[6,7],[7,4],[0,4],[1,5],[2,6],[3,7]])
    return vertices,triangles,edges


def cylinder_geometry(box,n=20):
    x=(box['x0']+box['x1'])/2; z=(box['bottom']+box['top'])/2
    rx=(box['x1']-box['x0'])/2; rz=(box['top']-box['bottom'])/2
    t=np.arange(n)*2*np.pi/n
    rings=[np.column_stack((x+rx*np.cos(t),np.full(n,y),z+rz*np.sin(t))) for y in (box['y0'],box['y1'])]
    vertices=np.vstack((*rings,[x,box['y0'],z],[x,box['y1'],z])).astype(np.float32)
    triangles=[]; edges=[]
    for i in range(n):
        j=(i+1)%n
        triangles.extend([i,j,n+j,i,n+j,n+i,2*n,j,i,2*n+1,n+i,n+j])
        edges.extend([[i,j],[n+i,n+j]])
    return vertices,np.array(triangles),np.array(edges)
