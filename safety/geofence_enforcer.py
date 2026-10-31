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
        # ArduPilot's FENCE_POINT protocol requires FENCE_TOTAL before the indexed vertices.
        self.vehicle.set_parameter("FENCE_TOTAL", len(self.outer))
        for index, (lat, lon) in enumerate(self.outer): self.vehicle.send_fence_point(index, len(self.outer), lat, lon)
        self.vehicle.command_long(207, 1) # MAV_CMD_DO_FENCE_ENABLE
        self.vehicle.log(f"Companion fence active: {len(self.outer)} vertices")
    def safe_velocity(self, vx: float, vy: float, vz: float = 0.0) -> tuple[float,float,float]:
        s = self.vehicle.status; lat,lon=s.lat,s.lon
        if not lat or not lon: return vx,vy,vz
        heading=math.radians(s.heading_deg); north=vx*math.cos(heading)-vy*math.sin(heading); east=vx*math.sin(heading)+vy*math.cos(heading)
        future=(lat + north*2/EARTH_M_PER_DEG, lon + east*2/(EARTH_M_PER_DEG*math.cos(math.radians(lat))))
        valid = point_in_polygon(future,self.outer) and not any(point_in_polygon(future,p) for p in self.exclusions)
        n,e,d=nearest_interior_vector(lat,lon,self.outer)
        if not valid or d < self.margin:
            self.vehicle.log("FENCE: velocity clamped", 4)
            return (max(-.2,min(.2,vx))+n, max(-.2,min(.2,vy))+e, vz)
        return vx,vy,vz
    def send_body_velocity(self, vx: float, vy: float, vz: float = 0.0) -> None:
        self.vehicle.send_body_velocity(*self.safe_velocity(vx,vy,vz))
    def goto_gps(self, waypoint: Waypoint, altitude_m: float | None = None) -> None:
        if not point_in_polygon((waypoint.lat,waypoint.lon),self.outer) or any(point_in_polygon((waypoint.lat,waypoint.lon),p) for p in self.exclusions):
            raise ValueError("refusing GPS target outside containment or in exclusion polygon")
        self.vehicle.goto_gps(waypoint.lat,waypoint.lon, waypoint.alt if altitude_m is None else altitude_m)
