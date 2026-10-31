from __future__ import annotations
import math
from typing import Callable
from core.waypoint_loader import Waypoint

EARTH_M_PER_DEG = 111_319.49

def _xy(lat: float, lon: float, origin: tuple[float, float]) -> tuple[float, float]:
    return ((lon-origin[1])*EARTH_M_PER_DEG*math.cos(math.radians(origin[0])), (lat-origin[0])*EARTH_M_PER_DEG)

def point_in_polygon(point: tuple[float, float], polygon: list[tuple[float, float]]) -> bool:
    x, y = point; inside = False
    for i, (xi, yi) in enumerate(polygon):
        xj, yj = polygon[i-1]
        if (yi > y) != (yj > y) and x < (xj-xi)*(y-yi)/(yj-yi + 1e-15)+xi: inside = not inside
    return inside

def _point_segment_distance(point: tuple[float, float], start: tuple[float, float],
                            end: tuple[float, float]) -> float:
    px,py=point; ax,ay=start; bx,by=end; dx,dy=bx-ax,by-ay; denom=dx*dx+dy*dy
    t=0.0 if denom==0 else max(0.0,min(1.0,((px-ax)*dx+(py-ay)*dy)/denom))
    return math.hypot(px-(ax+t*dx),py-(ay+t*dy))

def nearest_interior_vector(lat: float, lon: float, polygon: list[tuple[float, float]]) -> tuple[float, float, float]:
    """Approximate a restoring vector toward polygon centroid, plus edge distance."""
    origin = (lat, lon); p = (0., 0.); vertices = [_xy(a,b,origin) for a,b in polygon]
    nearest, centroid = 1e9, (sum(v[0] for v in vertices)/len(vertices), sum(v[1] for v in vertices)/len(vertices))
    for i, a in enumerate(vertices):
        b = vertices[(i+1) % len(vertices)]; dx,dy=b[0]-a[0],b[1]-a[1]; denom=dx*dx+dy*dy
        t = 0 if denom == 0 else max(0,min(1, - (a[0]*dx+a[1]*dy)/denom)); q=(a[0]+t*dx,a[1]+t*dy)
        nearest=min(nearest, math.hypot(q[0],q[1]))
    norm=math.hypot(*centroid) or 1; return centroid[1]/norm, centroid[0]/norm, nearest # north/east, metres

class GeofenceEnforcer:
    def __init__(self, vehicle, polygons: list[list[tuple[float,float]]], margin_m: float = 2.0):
        self.vehicle, self.outer, self.exclusions, self.margin = vehicle, polygons[0], polygons[1:], margin_m
    def upload(self) -> None:
        # Pixhawk 6X Mini/ArduPilot polygon storage uses the MAVLink2 mission
        # protocol. The upload method requires ACK and item-by-item read-back.
        self.vehicle.upload_fence_polygons(self.outer, self.exclusions)
        if not self.vehicle.command_long_confirmed(207, 1):  # MAV_CMD_DO_FENCE_ENABLE
            raise RuntimeError("Pixhawk rejected MAV_CMD_DO_FENCE_ENABLE")
        self.vehicle.log(
            f"Companion fence active: {len(self.outer)} inclusion vertices, "
            f"{sum(len(polygon) for polygon in self.exclusions)} exclusion vertices"
        )
    def safe_velocity(self, vx: float, vy: float, vz: float = 0.0) -> tuple[float,float,float]:
        s = self.vehicle.status; lat,lon=s.lat,s.lon
        if not lat or not lon: return vx,vy,vz
        heading=math.radians(s.heading_deg); north=vx*math.cos(heading)-vy*math.sin(heading); east=vx*math.sin(heading)+vy*math.cos(heading)
        future=(lat + north*2/EARTH_M_PER_DEG, lon + east*2/(EARTH_M_PER_DEG*math.cos(math.radians(lat))))
        n,e,d=nearest_interior_vector(lat,lon,self.outer)
        correction: tuple[float,float] | None = None
        if not point_in_polygon(future,self.outer) or d < self.margin:
            correction=(n,e)  # north/east toward the inclusion centroid
        else:
            for polygon in self.exclusions:
                vertices=[_xy(a,b,(lat,lon)) for a,b in polygon]
                edge_distance=min(
                    _point_segment_distance((0.0,0.0),vertices[index-1],vertex)
                    for index,vertex in enumerate(vertices)
                )
                inside_now=point_in_polygon((lat,lon),polygon)
                if not (inside_now or point_in_polygon(future,polygon) or edge_distance<self.margin):
                    continue
                centroid_e=sum(vertex[0] for vertex in vertices)/len(vertices)
                centroid_n=sum(vertex[1] for vertex in vertices)/len(vertices)
                norm=math.hypot(centroid_n,centroid_e)
                if norm<1e-6:
                    # At the polygon centroid, head toward the nearest vertex;
                    # any direction away from the centroid is an escape.
                    escape_e,escape_n=min(vertices,key=lambda vertex: math.hypot(*vertex))
                    norm=math.hypot(escape_n,escape_e) or 1.0
                    correction=(escape_n/norm,escape_e/norm)
                else:
                    correction=(-centroid_n/norm,-centroid_e/norm)
                break
        if correction is not None:
            self.vehicle.log("FENCE: velocity clamped", 4, throttle_s=2.0, key="fence-clamp")
            correction_n,correction_e=correction
            # Convert the earth-frame correction back to body forward/right.
            body_x=correction_n*math.cos(heading)+correction_e*math.sin(heading)
            body_y=-correction_n*math.sin(heading)+correction_e*math.cos(heading)
            return (.8*body_x,.8*body_y,vz)
        return vx,vy,vz
    def send_body_velocity(self, vx: float, vy: float, vz: float = 0.0) -> None:
        self.vehicle.send_body_velocity(*self.safe_velocity(vx,vy,vz))
    def assert_safe_leg(self, start: tuple[float,float], end: tuple[float,float],
                        clearance_m: float | None = None) -> None:
        """Reject a straight Guided leg that approaches an exclusion polygon."""
        clearance=max(self.margin,0.0) if clearance_m is None else max(float(clearance_m),self.margin)
        north=(end[0]-start[0])*EARTH_M_PER_DEG
        east=(end[1]-start[1])*EARTH_M_PER_DEG*math.cos(math.radians(start[0]))
        samples=max(2,int(math.ceil(math.hypot(north,east)/.5))+1)
        for step in range(samples):
            ratio=step/(samples-1)
            point=(start[0]+(end[0]-start[0])*ratio,start[1]+(end[1]-start[1])*ratio)
            if not point_in_polygon(point,self.outer):
                raise ValueError("refusing GPS leg outside whole-flight containment")
            for polygon in self.exclusions:
                if point_in_polygon(point,polygon):
                    raise ValueError("refusing GPS leg through red/exclusion polygon")
                local_point=(0.0,0.0); vertices=[_xy(lat,lon,point) for lat,lon in polygon]
                distance=min(
                    _point_segment_distance(local_point,vertices[index-1],vertex)
                    for index,vertex in enumerate(vertices)
                )
                if distance<clearance:
                    raise ValueError(
                        f"refusing GPS leg within {clearance:.1f}m of red/exclusion polygon"
                    )
    def goto_gps(self, waypoint: Waypoint, altitude_m: float | None = None) -> None:
        if not point_in_polygon((waypoint.lat,waypoint.lon),self.outer) or any(point_in_polygon((waypoint.lat,waypoint.lon),p) for p in self.exclusions):
            raise ValueError("refusing GPS target outside containment or in exclusion polygon")
        status=self.vehicle.status
        if status.lat and status.lon:
            self.assert_safe_leg((status.lat,status.lon),(waypoint.lat,waypoint.lon))
        self.vehicle.goto_gps(waypoint.lat,waypoint.lon, waypoint.alt if altitude_m is None else altitude_m)
