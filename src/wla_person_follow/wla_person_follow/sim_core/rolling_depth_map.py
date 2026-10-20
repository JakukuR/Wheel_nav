"""ROG-inspired 2-D ring-buffer map projected from a depth camera cloud.

Not the upstream ROG-Map library. Occupancy is persistent, finite, timestamped;
unobserved/expired cells are unknown. Grid axes remain in continuous odom.
"""
import base64
import numpy as np
from scipy.ndimage import distance_transform_edt


class RollingDepthMap:
    def __init__(self,resolution=.1,size=120,ttl=12.):
        self.res=resolution;self.n=size;self.ttl=ttl
        self.log=np.zeros((size,size));self.stamp=np.full((size,size),-np.inf)
        self.tags=np.full((size,size,2),-10**9,dtype=np.int64)
        self.lower=np.zeros(2,dtype=int);self.replacements=0

    def slide(self,position):
        self.lower=np.floor(np.asarray(position)/self.res).astype(int)-self.n//2
        x,y=np.meshgrid(np.arange(self.n)+self.lower[0],np.arange(self.n)+self.lower[1],indexing='ij')
        self.cells=np.stack((x,y),axis=-1);self.ix=x%self.n;self.iy=y%self.n
        changed=np.any(self.tags[self.ix,self.iy]!=self.cells,axis=-1)
        a,b=self.ix[changed],self.iy[changed]
        self.log[a,b]=0.;self.stamp[a,b]=-np.inf;self.tags[a,b]=self.cells[changed]
        self.replacements+=np.sum(changed)

    def _slots(self,xy):
        cell=np.floor(np.asarray(xy)/self.res).astype(int)
        valid=np.all((cell>=self.lower)&(cell<self.lower+self.n),axis=-1)
        cell=np.unique(cell[valid],axis=0)
        return cell[:,0]%self.n,cell[:,1]%self.n

    def initial_clear_patch(self,position,t,radius=1.):
        """Explicit simulation initial condition: inspected clear start patch.

        Required by a forward depth camera's near/rear blind zone. Not inferred
        from no returns; not populated with scene obstacle positions.
        """
        centers=(self.cells+.5)*self.res
        mask=np.linalg.norm(centers-position,axis=-1)<radius
        a,b=self.ix[mask],self.iy[mask];self.log[a,b]=-2.;self.stamp[a,b]=t

    def update(self,cloud,pose,t):
        self.slide(pose[:2])
        # Keep per-head ray origins; process every miss before this frame's hits.
        clouds=cloud if isinstance(cloud,(list,tuple)) else [cloud]
        occupied_points=[]
        for cloud in clouds:
            p=cloud['points'];dynamic=cloud['dynamic']
            # Perception has already applied the timestamped full 3-D odom transform.
            origin=cloud['origin_odom'][:2]
            world=cloud['points_odom'][:,:2]
            # High rays can pass OVER low obstacles: never use them to erase cells.
            lower=p[:,2]<=cloud['origin'][2]+.02
            endpoint=world[lower]
            if len(endpoint):
                # One ray per endpoint cell; retain the nearest endpoint.
                _,idx=np.unique(np.floor(endpoint/self.res).astype(int),axis=0,return_index=True)
                endpoint=endpoint[idx]
                delta=endpoint-origin;length=np.linalg.norm(delta,axis=1)
                distance=np.arange(0,7.,self.res*.45)
                points=origin+delta[:,None,:]/np.maximum(length[:,None,None],1e-9)*distance[None,:,None]
                mask=distance[None,:]<length[:,None]-.12
                a,b=self._slots(points[mask]);self.log[a,b]=np.maximum(-2.,self.log[a,b]-.8);self.stamp[a,b]=t
            # Floor endpoints certify traversable ground only in this flat-floor demo.
            a,b=self._slots(world[p[:,2]<.04]);self.log[a,b]=np.maximum(-2.,self.log[a,b]-.8);self.stamp[a,b]=t
            hits=(p[:,2]>=.08)&(p[:,2]<=1.6)&~dynamic
            occupied_points.append(world[hits])
        hits=np.concatenate(occupied_points) if occupied_points else np.empty((0,2))
        a,b=self._slots(hits);self.log[a,b]=np.minimum(3.,self.log[a,b]+2.);self.stamp[a,b]=t
        # The robot itself occupies this footprint; keep its current cell region
        # known when stationary, without declaring its surroundings free.
        centers=(self.cells+.5)*self.res
        footprint=np.linalg.norm(centers-pose[:2],axis=-1)<.24
        a,b=self.ix[footprint],self.iy[footprint]
        self.log[a,b]=np.minimum(self.log[a,b],-1.);self.stamp[a,b]=t
        self.rebuild(t)

    def rebuild(self,t):
        log=self.log[self.ix,self.iy];age=t-self.stamp[self.ix,self.iy]
        self.known=age<=self.ttl
        self.occupied=self.known&(log>=0)
        # Pad map boundaries as blocked. EDT lower bounds compensate for cell
        # area and query-point position; interpolation must not overstate safety.
        self.dist=distance_transform_edt(np.pad(~self.occupied,1,constant_values=False))[1:-1,1:-1]*self.res
        self.free_dist=distance_transform_edt(np.pad(self.known&~self.occupied,1,constant_values=False))[1:-1,1:-1]*self.res

    def query(self,xy,unknown=False,field=None):
        xy=np.asarray(xy);index=np.floor(xy/self.res).astype(int)-self.lower
        valid=np.all((index>=0)&(index<self.n),axis=-1)
        bounded=np.clip(index,0,self.n-1)
        if field is None:field=self.free_dist if unknown else self.dist
        center=(index+self.lower+.5)*self.res
        value=field[bounded[...,0],bounded[...,1]]-self.res/np.sqrt(2)-np.linalg.norm(xy-center,axis=-1)
        return np.where(valid,value,0.)

    def snapshot(self,t,pose):
        # Occupied + free display at 20 cm resolution. Unknown is transparent.
        known=self.known.reshape(self.n//2,2,self.n//2,2).all(axis=(1,3))
        occupied=self.occupied.reshape(self.n//2,2,self.n//2,2).any(axis=(1,3))
        pack=lambda a:base64.b64encode(np.packbits(a.reshape(-1)).tobytes()).decode()
        return dict(t=round(t,2),lower=(self.lower*self.res).round(3).tolist(),
                    free=pack(known&~occupied),occupied=pack(occupied),pose=np.round(pose,4).tolist())
