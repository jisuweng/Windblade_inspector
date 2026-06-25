#!/usr/bin/env python3
"""
Tower Axis Detector — line fitting, control points, visualisation.
No cylinder filtering / point deletion.
"""
import rospy, numpy as np, time, math
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64


class TowerDetector:
    def __init__(self):
        self.dist_th   = rospy.get_param("~distance_threshold", 0.8)
        self.iters     = rospy.get_param("~num_iterations", 50)
        self.ang_max   = rospy.get_param("~vertical_angle_max", 15.0)
        self.ratio_min = rospy.get_param("~min_inlier_ratio", 0.05)
        self.xy_tol    = rospy.get_param("~xy_tolerance", 1.5)
        self.rate_hz   = rospy.get_param("~rate", 2.0)
        self.Z = np.array([0., 0., 1.])

        self.ref_line   = None
        self.diameter   = None   # estimated once
        self.radius     = None
        self.circle_cx  = None   # fitted circle centre XY
        self.circle_cy  = None
        self.frame_n    = 0
        self.cp_id      = 0
        self.last_cp    = None

        self.diam_pub   = rospy.Publisher("/tower_filter/diameter", Float64, queue_size=1, latch=True)
        self.radius_pub = rospy.Publisher("/tower_filter/radius",   Float64, queue_size=1, latch=True)

        self.line_pub    = rospy.Publisher("/tower_axis_line",      Marker, queue_size=1)
        self.cyl_pub    = rospy.Publisher("/tower_axis_cylinder",  Marker, queue_size=1)
        self.ctrl_pub   = rospy.Publisher("/tower_control_point",  PoseStamped, queue_size=1)
        self.path_pub   = rospy.Publisher("/tower_path",           Path, queue_size=1)
        self.marker_pub = rospy.Publisher("/tower_markers",        Marker, queue_size=10)

        self.tower_path = Path(); self.tower_path.header.frame_id = "map"

        self.timer = rospy.Timer(rospy.Duration(1./self.rate_hz), self._tick)

        print("\n" + "="*60)
        print("  TowerDetector  %.1f Hz  frame=map" % self.rate_hz)
        print("="*60 + "\n")

    def _tick(self, event):
        self.frame_n += 1

        try:    cloud_msg = rospy.wait_for_message("/cloud_registered", PointCloud2, timeout=0.5)
        except: return
        try:    odom_msg = rospy.wait_for_message("/Odometry", Odometry, timeout=0.3)
        except: return

        drone = np.array([odom_msg.pose.pose.position.x,
                          odom_msg.pose.pose.position.y,
                          odom_msg.pose.pose.position.z])

        t0 = time.time()
        gen = pc2.read_points(cloud_msg, field_names=("x","y","z"), skip_nans=True)
        pts = np.array([[p[0],p[1],p[2]] for p in gen])
        if len(pts) > 3000:
            pts = pts[np.random.choice(len(pts), 3000, replace=False)]
        if len(pts) < 30: return

        best = self._ransac(pts)
        if best is None: return
        dir_raw, n_in, mask = best
        inl = pts[mask]

        ang = math.degrees(math.acos(abs(np.dot(dir_raw, self.Z))))
        if ang > self.ang_max: return

        new_x  = float(np.mean(inl[:,0]))
        new_y  = float(np.mean(inl[:,1]))
        new_zn = float(np.min(inl[:,2]))
        new_zx = float(np.max(inl[:,2]))
        dt     = (time.time()-t0)*1000

        # ---- estimate diameter once (first valid frame) ----
        if self.diameter is None:
            self._est_diameter(inl)

        if self.ref_line is None:
            self.ref_line = (new_x, new_y, new_zn, new_zx)
            print("[%d] INIT (%.2f,%.2f) Z[%.1f,%.1f] in=%d/%d %dms" %
                  (self.frame_n, new_x, new_y, new_zn, new_zx, n_in, len(pts), dt))
        else:
            rx, ry, rzn, rzx = self.ref_line
            dxy = math.hypot(new_x-rx, new_y-ry)
            if dxy < self.xy_tol:
                self.ref_line = (rx, ry, min(rzn, new_zn), max(rzx, new_zx))
                print("[%d] EXT Z[%.1f,%.1f] xy=%.2f in=%d/%d %dms" %
                      (self.frame_n, self.ref_line[2], self.ref_line[3], dxy, n_in, len(pts), dt))
            else:
                self.ref_line = (new_x, new_y, new_zn, new_zx)
                print("[%d] NEW Z[%.1f,%.1f] xy=%.2f>%.2f" %
                      (self.frame_n, new_zn, new_zx, dxy, self.xy_tol))

        rx, ry, rz_min, rz_max = self.ref_line

        # publish global params for other nodes
        rospy.set_param("/tower/ref_x", float(rx))
        rospy.set_param("/tower/ref_y", float(ry))
        rospy.set_param("/tower/z_min", float(rz_min))
        rospy.set_param("/tower/z_max", float(rz_max))
        rospy.set_param("/tower/direction", [0.0, 0.0, 1.0])

        # control point (project drone onto line)
        cp = np.array([rx, ry, (rz_min+rz_max)/2.])
        cp[2] += float(np.dot(drone-cp, self.Z))

        rospy.set_param("/tower/control_x", float(cp[0]))
        rospy.set_param("/tower/control_y", float(cp[1]))
        rospy.set_param("/tower/control_z", float(cp[2]))

        # visualisation
        self._line(rx, ry, rz_min, rz_max, 0.08, 0, 1, 0)      # green line
        self._cyl(rx, ry, rz_min, rz_max, 0.6)                   # cylinder
        self._ctrl(cp)
        self._path(cp)

        # control-point spheres (spacing >= 1.5m)
        d = float(np.linalg.norm(cp-self.last_cp)) if self.last_cp is not None else 999
        if self.last_cp is None or d > 1.5:
            self._sph(cp[0],cp[1],cp[2], 0.2, 0,1,0, self.cp_id)
            self.cp_id += 1; self.last_cp = cp.copy()
            print("    CP#%d (%.2f,%.2f,%.2f) d=%.2f" % (self.cp_id-1, cp[0],cp[1],cp[2], d))

        # endpoint spheres
        self._sph(rx, ry, rz_min, 0.25, 0,0.7,1, 9998)
        self._sph(rx, ry, rz_max, 0.25, 1,0.7,0, 9999)

        # width marker: sphere at circle centre + ring (once estimated)
        if self.circle_cx is not None:
            z_mid = (rz_min+rz_max)/2.
            self._sph(self.circle_cx, self.circle_cy, z_mid, 0.3, 1.0,0.5,0, 9997)
            # horizontal ring at circle centre
            self._ring(self.circle_cx, self.circle_cy, z_mid, self.radius, 1.0, 0.5, 0)

    # ==================================================================
    def _est_diameter(self, inl):
        """Circle fit in XY plane → diameter.  Returns True on success."""
        xy = inl[:,:2]; n = len(xy)
        A = np.column_stack([xy[:,0], xy[:,1], np.ones(n)])
        b = xy[:,0]**2 + xy[:,1]**2
        try:
            s, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
        except np.linalg.LinAlgError:
            print("  !! circle-fit error"); return False
        a, bc, c = s; cx = a/2; cy = bc/2; r2 = c + cx**2 + cy**2
        if r2 <= 0: print("  !! R^2=%.3f" % r2); return False
        r = math.sqrt(r2)
        if r < 0.1 or r > 10.0: print("  !! radius %.2f out of range" % r); return False
        self.diameter = 2.0*r
        self.radius   = r
        self.circle_cx = cx
        self.circle_cy = cy
        rospy.set_param("/tower/diameter",  float(self.diameter))
        rospy.set_param("/tower/radius",    float(self.radius))
        rospy.set_param("/tower/circle_cx", float(cx))
        rospy.set_param("/tower/circle_cy", float(cy))
        self.diam_pub.publish(Float64(self.diameter))
        self.radius_pub.publish(Float64(self.radius))
        print("  >>> 塔筒直径: %.2f m  半径: %.2f m  圆心XY: (%.2f, %.2f)" %
              (self.diameter, self.radius, cx, cy))
        return True

    def _ransac(self, pts):
        n=len(pts); bn=0; be=float("inf"); bm=None; bd=None
        for _ in range(self.iters):
            i,j=np.random.choice(n,2,replace=False)
            d=pts[j]-pts[i]; dl=float(np.linalg.norm(d))
            if dl<1e-4: continue; d/=dl
            ds=np.linalg.norm(np.cross(pts-pts[i],d),axis=1)
            m=ds<self.dist_th; ni=int(np.sum(m))
            e=float(np.mean(ds[m])) if ni>0 else float("inf")
            if ni>bn or (ni==bn and e<be):
                bn=ni; be=e; bm=m.copy()
                if ni>=2: il=pts[m]; c=np.mean(il,0); _,v=np.linalg.eigh(np.cov((il-c).T)); bd=v[:,-1]
        if bn<n*self.ratio_min or bm is None: return None
        return (bd if bd is not None else self.Z), bn, bm

    # ==================================================================
    def _ring(self, cx, cy, z, radius, r, g, b):
        """Draw a horizontal circle (LINE_STRIP) at (cx,cy,z) with given radius."""
        m=Marker(); m.header.frame_id="map"; m.header.stamp=rospy.Time.now()
        m.ns="width"; m.id=0; m.type=Marker.LINE_STRIP; m.action=Marker.ADD
        m.scale.x=0.06; m.color.r=r; m.color.g=g; m.color.b=b; m.color.a=1.0
        m.lifetime=rospy.Duration(0)
        for i in range(37):
            a = 2*math.pi*i/36
            m.points.append(Point(cx+radius*math.cos(a), cy+radius*math.sin(a), z))
        self.marker_pub.publish(m)

    def _sph(self,x,y,z,sc,r,g,b,id):
        m=Marker(); m.header.frame_id="map"; m.header.stamp=rospy.Time.now()
        m.ns="cp"; m.id=id; m.type=Marker.SPHERE; m.action=Marker.ADD
        m.pose.position.x=x; m.pose.position.y=y; m.pose.position.z=z; m.pose.orientation.w=1
        m.scale.x=m.scale.y=m.scale.z=sc; m.color.r=r; m.color.g=g; m.color.b=b; m.color.a=1
        m.lifetime=rospy.Duration(0); self.marker_pub.publish(m)

    def _line(self,x,y,zmin,zmax,w,r,g,b):
        m=Marker(); m.header.frame_id="map"; m.header.stamp=rospy.Time.now()
        m.ns="tower"; m.id=0; m.type=Marker.LINE_STRIP; m.action=Marker.ADD
        m.scale.x=w; m.color.r=r; m.color.g=g; m.color.b=b; m.color.a=1
        m.lifetime=rospy.Duration(0); m.points=[Point(x,y,zmin),Point(x,y,zmax)]
        self.line_pub.publish(m)

    def _cyl(self,x,y,zmin,zmax,s):
        h=max(zmax-zmin,0.5); m=Marker()
        m.header.frame_id="map"; m.header.stamp=rospy.Time.now()
        m.ns="tower"; m.id=1; m.type=Marker.CYLINDER; m.action=Marker.ADD
        m.pose.position.x=x; m.pose.position.y=y; m.pose.position.z=(zmin+zmax)/2
        m.pose.orientation.w=1; m.scale.x=m.scale.y=s; m.scale.z=h
        m.color.g=1; m.color.a=0.15; m.lifetime=rospy.Duration(0)
        self.cyl_pub.publish(m)

    def _ctrl(self,pt):
        m=PoseStamped(); m.header.frame_id="map"; m.header.stamp=rospy.Time.now()
        m.pose.position.x=pt[0]; m.pose.position.y=pt[1]; m.pose.position.z=pt[2]
        m.pose.orientation.w=1; self.ctrl_pub.publish(m)

    def _path(self,pt):
        ps=PoseStamped(); ps.header.frame_id="map"; ps.header.stamp=rospy.Time.now()
        ps.pose.position.x=pt[0]; ps.pose.position.y=pt[1]; ps.pose.position.z=pt[2]
        ps.pose.orientation.w=1
        self.tower_path.header.stamp=rospy.Time.now(); self.tower_path.poses.append(ps)
        if len(self.tower_path.poses)>5000: self.tower_path.poses=self.tower_path.poses[-5000:]
        self.path_pub.publish(self.tower_path)


if __name__=="__main__":
    rospy.init_node("tower_detector")
    try: TowerDetector(); rospy.spin()
    except rospy.ROSInterruptException: pass
