"""Generate a self-contained interactive Canvas preview of the converted HD map."""

import json
import math
import subprocess
import webbrowser
from pathlib import Path

from .geometry import convex_hull, simplify_rdp
from .lanelet2_export import boundary_tags, surface_marking_maneuver


def _xy(transformer, points, tolerance):
    simplified = simplify_rdp(points, tolerance)
    return [[round(value[0], 3), round(value[1], 3)]
            for value in (transformer.mgeo_to_sim(point) for point in simplified)]


def load_global_route(path):
    """Load the provided MORAI SIM-local XYZ route without reprojecting it."""
    if path is None:
        return {"id": "", "p": [], "point_count": 0, "source": "",
                "coordinate_frame": "MORAI SIM local ENU (metres)", "closed": False}
    path = Path(path)
    if not path.is_file():
        return {"id": path.name, "p": [], "point_count": 0,
                "source": path.name,
                "coordinate_frame": "MORAI SIM local ENU (metres)",
                "closed": False}
    points = []
    with path.open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            values = line.split()
            if not values:
                continue
            if len(values) < 2:
                raise ValueError(
                    "global route line {} has fewer than two coordinates".format(
                        line_number))
            try:
                x, y = float(values[0]), float(values[1])
            except ValueError as error:
                raise ValueError(
                    "global route line {} has invalid coordinates".format(
                        line_number)) from error
            if not math.isfinite(x) or not math.isfinite(y):
                raise ValueError(
                    "global route line {} has non-finite coordinates".format(
                        line_number))
            # Keep all source samples and their ordering; 4,430 points are small
            # enough for Canvas and are also the alignment validator's input.
            points.append([round(x, 3), round(y, 3)])
    return {
        "id": path.name,
        "p": points,
        "point_count": len(points),
        "source": path.name,
        "coordinate_frame": "MORAI SIM local ENU (metres)",
        "closed": bool(len(points) > 1 and points[0] == points[-1]),
    }


def _bounds(points, margin=0.0):
    """Return an axis-aligned SIM-local extent around a point sequence."""
    return {
        "min_x": min(point[0] for point in points) - margin,
        "min_y": min(point[1] for point in points) - margin,
        "max_x": max(point[0] for point in points) + margin,
        "max_y": max(point[1] for point in points) + margin,
    }


def _intersects_bounds(points, bounds):
    """Keep preview geometry whose own extent overlaps the route extent."""
    if not points:
        return False
    return not (
        max(point[0] for point in points) < bounds["min_x"] or
        min(point[0] for point in points) > bounds["max_x"] or
        max(point[1] for point in points) < bounds["min_y"] or
        min(point[1] for point in points) > bounds["max_y"]
    )


def build_viewer_data(dataset, transformer, config, exporter=None,
                      reference_path=None):
    tolerance = float(config.get("conversion", {}).get("viewer_simplification_m", 0.2))
    mapping = config.get("lane_boundary", {})
    signal_links = dataset.traffic_light_link_ids()
    boundaries = []
    for boundary_id, boundary in sorted(dataset.lane_boundaries.items()):
        points = _xy(transformer, boundary.get("points") or [], tolerance)
        if len(points) < 2:
            continue
        tags = boundary_tags(boundary, mapping)
        boundaries.append({
            "id": boundary_id,
            "p": points,
            "category": tags["mgeo:boundary_category"],
            "style": tags["subtype"],
            "color": tags["color"],
            "lanelet_type": tags["type"],
            "lane_change": tags.get("lane_change", ""),
        })
    centerlines = []
    marking_maneuvers = {}
    for marking in dataset.surface_markings.values():
        maneuver = surface_marking_maneuver(marking)
        if maneuver:
            for link_id in marking.get("link_id_list") or []:
                if link_id:
                    marking_maneuvers.setdefault(str(link_id), set()).add(maneuver)
    for link_id, link in sorted(dataset.links.items()):
        points = _xy(transformer, link.get("points") or [], tolerance)
        if len(points) < 2:
            continue
        centerlines.append({
            "id": link_id,
            "p": points,
            "speed": link.get("max_speed"),
            "direction": link.get("related_signal") or ",".join(
                sorted(marking_maneuvers.get(link_id, set()))),
            "predecessors": dataset.predecessors.get(link_id, []),
            "successors": dataset.successors.get(link_id, []),
            "left_change": link.get("left_lane_change_dst_link_idx"),
            "right_change": link.get("right_lane_change_dst_link_idx"),
        })
    crosswalks = []
    for crossing_id, crossing in sorted(dataset.single_crosswalks.items()):
        if str(crossing.get("sign_type", "")) not in ("5321", "533", "534"):
            continue
        points = _xy(transformer, crossing.get("points") or [], tolerance)
        if len(points) >= 3:
            crosswalks.append({"id": crossing_id, "p": points,
                               "links": [value for value in crossing.get("link_id_list") or [] if value]})
    surface_markings = []
    for marking_id, marking in sorted(dataset.surface_markings.items()):
        points = _xy(transformer, marking.get("points") or [], tolerance)
        if len(points) >= 3:
            surface_markings.append({
                "id": marking_id,
                "p": points,
                "subtype": str(marking.get("sub_type", "")),
                "maneuver": surface_marking_maneuver(marking),
                "links": [value for value in marking.get("link_id_list") or [] if value],
            })
    signals = []
    for signal_id, signal in sorted(dataset.traffic_lights.items()):
        point = transformer.mgeo_to_sim(signal.get("point") or [0.0, 0.0, 0.0])
        signals.append({
            "id": signal_id,
            "p": [round(point[0], 3), round(point[1], 3)],
            "links": signal_links.get(signal_id, []),
            "kind": signal.get("type", ""),
            "heading": signal.get("heading", 0.0),
        })
    intersections = []
    if exporter is not None:
        geometries = exporter.intersection_geometries
    else:
        geometries = {}
        for junction_id, junction in dataset.junctions.items():
            points = [point
                      for road_id in junction.get("road_id_list") or []
                      for link_id in dataset.links_by_road.get(str(road_id), [])
                      for point in dataset.links[link_id].get("points") or []]
            geometries[junction_id] = convex_hull(points)
    for junction_id, points in sorted(geometries.items()):
        transformed = _xy(transformer, points, tolerance)
        if len(transformed) >= 3:
            intersections.append({"id": junction_id, "p": transformed})
    global_route = load_global_route(reference_path)
    crop_applied = bool(global_route["p"])
    crop_anchor_boundary_ids = []
    if crop_applied:
        margin = float(config.get("conversion", {}).get(
            "viewer_route_crop_margin_m", 0.0))
        bounds = _bounds(global_route["p"], margin=max(0.0, margin))
        requested_anchors = {
            str(value) for value in config.get("conversion", {}).get(
                "viewer_crop_anchor_boundary_ids", [])
        }
        anchor_boundaries = [item for item in boundaries
                             if item["id"] in requested_anchors]
        crop_anchor_boundary_ids = [item["id"] for item in anchor_boundaries]
        anchor_points = [point for item in anchor_boundaries for point in item["p"]]
        if anchor_points:
            anchor_bounds = _bounds(anchor_points)
            bounds = {
                "min_x": min(bounds["min_x"], anchor_bounds["min_x"]),
                "min_y": min(bounds["min_y"], anchor_bounds["min_y"]),
                "max_x": max(bounds["max_x"], anchor_bounds["max_x"]),
                "max_y": max(bounds["max_y"], anchor_bounds["max_y"]),
            }
        centerlines = [item for item in centerlines
                       if _intersects_bounds(item["p"], bounds)]
        boundaries = [item for item in boundaries
                      if _intersects_bounds(item["p"], bounds)]
        crosswalks = [item for item in crosswalks
                      if _intersects_bounds(item["p"], bounds)]
        surface_markings = [item for item in surface_markings
                            if _intersects_bounds(item["p"], bounds)]
        signals = [item for item in signals
                   if _intersects_bounds([item["p"]], bounds)]
        intersections = [item for item in intersections
                         if _intersects_bounds(item["p"], bounds)]
    else:
        all_points = [point for item in centerlines for point in item["p"]]
        if not all_points:
            all_points = [[0.0, 0.0], [1.0, 1.0]]
        bounds = _bounds(all_points)
    return {
        "metadata": {
            "title": "KATRI MGeo 3.0 → Lanelet2",
            "source_status": config.get("source", {}).get("status"),
            "source_commit": config.get("source", {}).get("commit"),
            "scene": config.get("coordinates", {}).get("simulator_scene"),
            "coordinate_frame": "MORAI SIM local ENU (metres)",
            "bounds": bounds,
            "route_crop_applied": crop_applied,
            "crop_anchor_boundary_ids": crop_anchor_boundary_ids,
            "counts": {
                "centerlines": len(centerlines),
                "boundaries": len(boundaries),
                "stop_lines": sum(item["category"] == "stop_line" for item in boundaries),
                "crosswalks": len(crosswalks),
                "road_markings": len(surface_markings),
                "traffic_lights": len(signals),
                "intersections": len(intersections),
                "global_route_points": global_route["point_count"],
            },
        },
        "centerlines": centerlines,
        "boundaries": boundaries,
        "crosswalks": crosswalks,
        "surfaceMarkings": surface_markings,
        "signals": signals,
        "intersections": intersections,
        "globalRoute": global_route,
    }


HTML_TEMPLATE = r'''<!doctype html>
<html lang="ko">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>KATRI HD Map Preview</title>
<style>
:root { color-scheme: dark; font-family: Inter, Pretendard, system-ui, sans-serif; }
* { box-sizing: border-box; }
body { margin: 0; overflow: hidden; background: #071018; color: #dbe7ef; }
#map { position: fixed; inset: 0 340px 0 0; width: calc(100vw - 340px); height: 100vh; cursor: grab; }
#map.dragging { cursor: grabbing; }
aside { position: fixed; top: 0; right: 0; width: 340px; height: 100vh; overflow-y: auto;
  background: rgba(12,24,34,.97); border-left: 1px solid #26404f; padding: 18px; }
h1 { margin: 0 0 4px; font-size: 20px; letter-spacing: -.02em; }
.subtitle { color: #83a1b2; font-size: 12px; line-height: 1.5; overflow-wrap: anywhere; }
.badge { display: inline-block; margin-top: 10px; padding: 4px 8px; border: 1px solid #c69034;
  color: #ffd47b; border-radius: 999px; font-size: 11px; text-transform: uppercase; }
h2 { font-size: 12px; color: #83a1b2; text-transform: uppercase; letter-spacing: .12em; margin: 22px 0 10px; }
label { display: flex; align-items: center; gap: 9px; padding: 5px 0; font-size: 13px; }
input { accent-color: #32d3a2; }
.swatch { width: 22px; border-top: 3px solid #fff; }
.swatch.dashed { border-top-style: dashed; }.swatch.yellow { border-color:#ffd84e; }
.swatch.blue { border-color:#39a9ff; }.swatch.cyan { border-color:#3ce5e7; }
.swatch.red { border-color:#ff5c72; }.swatch.purple { border-color:#b88cff; }
.swatch.green { border-color:#39ff88; box-shadow:0 0 5px rgba(57,255,136,.65); }
.stats { display:grid; grid-template-columns:1fr 1fr; gap:7px; font-size:12px; }
.stat { background:#102432; padding:8px; border-radius:6px; }.stat b { display:block; font-size:17px; color:#f5fbff; }
#inspect { min-height:100px; padding:10px; border:1px solid #294655; background:#091721; border-radius:7px;
  white-space:pre-wrap; overflow-wrap:anywhere; font:11px/1.45 ui-monospace, monospace; color:#bcd1dd; }
.help { color:#7893a2; font-size:11px; line-height:1.5; }
</style>
</head>
<body>
<canvas id="map"></canvas>
<aside>
  <h1 id="title"></h1>
  <div class="subtitle" id="subtitle"></div>
  <span class="badge">immutable candidate</span>
  <h2>Layers</h2>
  <label><input data-layer="globalRoute" type="checkbox" checked><span class="swatch green"></span>전역경로 TXT</label>
  <label><input data-layer="intersections" type="checkbox"><span class="swatch purple"></span>교차로 영역(파생)</label>
  <label><input data-layer="centerlines" type="checkbox" checked><span class="swatch cyan"></span>차선 중심선</label>
  <label><input data-layer="solid" type="checkbox" checked><span class="swatch"></span>실선 경계</label>
  <label><input data-layer="dashed" type="checkbox" checked><span class="swatch dashed"></span>점선 경계</label>
  <label><input data-layer="roadBorders" type="checkbox" checked><span class="swatch yellow"></span>도로/중앙분리대 경계</label>
  <label><input data-layer="stopLines" type="checkbox" checked><span class="swatch red"></span>정지선</label>
  <label><input data-layer="crosswalks" type="checkbox" checked><span class="swatch blue"></span>횡단보도</label>
  <label><input data-layer="surfaceMarkings" type="checkbox"><span class="swatch cyan"></span>방향 화살표/노면표시</label>
  <label><input data-layer="signals" type="checkbox" checked>🚦 신호등 + ID</label>
  <label><input data-layer="topology" type="checkbox">→ 선행/후행 연결</label>
  <label><input data-layer="labels" type="checkbox">50 속도/방향 라벨</label>
  <h2>Counts</h2><div class="stats" id="stats"></div>
  <h2>Inspector</h2><div id="inspect">지형지물을 클릭하면 MGeo ID와 속성이 표시됩니다.</div>
  <h2>Navigation</h2><div class="help">휠: 확대/축소 · 드래그: 이동 · 더블클릭: 전체 보기<br>좌표는 실행 중인 K-City scene의 local ENU(m) 기준입니다.</div>
</aside>
<script>
const MAP = __MAP_DATA__;
const canvas = document.getElementById('map'), ctx = canvas.getContext('2d');
const enabled = {}; document.querySelectorAll('[data-layer]').forEach(el => {
  enabled[el.dataset.layer] = el.checked; el.addEventListener('change',()=>{enabled[el.dataset.layer]=el.checked;draw();});
});
document.getElementById('title').textContent=MAP.metadata.title;
document.getElementById('subtitle').textContent=`${MAP.metadata.scene} · ${MAP.metadata.coordinate_frame}\ncommit ${MAP.metadata.source_commit}`;
document.getElementById('stats').innerHTML=Object.entries(MAP.metadata.counts).map(([k,v])=>`<div class="stat"><b>${v.toLocaleString()}</b>${k}</div>`).join('');
let dpr=window.devicePixelRatio||1, scale=1, ox=0, oy=0, dragging=false, last=null;
function resize(){const r=canvas.getBoundingClientRect();canvas.width=r.width*dpr;canvas.height=r.height*dpr;ctx.setTransform(dpr,0,0,dpr,0,0);fit();}
function fit(){const b=MAP.metadata.bounds,w=canvas.clientWidth,h=canvas.clientHeight,p=35;scale=Math.min((w-2*p)/(b.max_x-b.min_x),(h-2*p)/(b.max_y-b.min_y));ox=w/2-scale*(b.min_x+b.max_x)/2;oy=h/2+scale*(b.min_y+b.max_y)/2;draw();}
function s(p){return [p[0]*scale+ox,-p[1]*scale+oy];}
function path(points,close=false){if(!points.length)return;let q=s(points[0]);ctx.beginPath();ctx.moveTo(q[0],q[1]);for(let i=1;i<points.length;i++){q=s(points[i]);ctx.lineTo(q[0],q[1]);}if(close)ctx.closePath();}
function stroke(item,color,width,dash=[]){path(item.p);ctx.strokeStyle=color;ctx.lineWidth=width;ctx.setLineDash(dash);ctx.stroke();ctx.setLineDash([]);}
function boundaryColor(item){if(item.color==='yellow')return '#ffd84e';if(item.color==='blue')return '#39a9ff';return '#edf5f7';}
function arrow(a,b){const p=s(a),q=s(b),ang=Math.atan2(q[1]-p[1],q[0]-p[0]);ctx.beginPath();ctx.moveTo(p[0],p[1]);ctx.lineTo(q[0],q[1]);ctx.lineTo(q[0]-5*Math.cos(ang-.6),q[1]-5*Math.sin(ang-.6));ctx.moveTo(q[0],q[1]);ctx.lineTo(q[0]-5*Math.cos(ang+.6),q[1]-5*Math.sin(ang+.6));ctx.stroke();}
const centers=Object.fromEntries(MAP.centerlines.map(v=>[v.id,v]));
function draw(){const w=canvas.clientWidth,h=canvas.clientHeight;ctx.clearRect(0,0,w,h);ctx.fillStyle='#071018';ctx.fillRect(0,0,w,h);ctx.lineJoin='round';ctx.lineCap='round';
 if(enabled.intersections){ctx.fillStyle='rgba(184,140,255,.12)';ctx.strokeStyle='rgba(184,140,255,.5)';ctx.lineWidth=1;MAP.intersections.forEach(x=>{path(x.p,true);ctx.fill();ctx.stroke();});}
 if(enabled.centerlines)MAP.centerlines.forEach(x=>stroke(x,'rgba(60,229,231,.55)',1));
 MAP.boundaries.forEach(x=>{if(x.category==='stop_line'){if(enabled.stopLines)stroke(x,'#ff5c72',2.3);return;}if(x.category==='road_border'||x.category==='centerline'){if(enabled.roadBorders)stroke(x,boundaryColor(x),2);return;}if(x.style.includes('dashed')){if(enabled.dashed)stroke(x,boundaryColor(x),1.2,[6,5]);}else if(enabled.solid)stroke(x,boundaryColor(x),1.2);});
 if(enabled.crosswalks){ctx.fillStyle='rgba(57,169,255,.42)';ctx.strokeStyle='#52b8ff';ctx.lineWidth=1;MAP.crosswalks.forEach(x=>{path(x.p,true);ctx.fill();ctx.stroke();});}
 if(enabled.surfaceMarkings){ctx.fillStyle='rgba(60,229,231,.30)';ctx.strokeStyle='#3ce5e7';ctx.lineWidth=.8;MAP.surfaceMarkings.forEach(x=>{path(x.p,true);ctx.fill();ctx.stroke();});}
 if(enabled.topology){ctx.strokeStyle='rgba(255,143,77,.42)';ctx.lineWidth=.8;MAP.centerlines.forEach(x=>x.successors.forEach(id=>{const y=centers[id];if(y)arrow(x.p[x.p.length-1],y.p[0]);}));}
 if(enabled.globalRoute&&MAP.globalRoute.p.length){stroke(MAP.globalRoute,'rgba(2,9,12,.92)',5.4);stroke(MAP.globalRoute,'#39ff88',2.8);}
 if(enabled.signals){ctx.font='10px ui-monospace';MAP.signals.forEach(x=>{const p=s(x.p);ctx.fillStyle=x.kind==='pedestrian'?'#55b7ff':'#ff5f65';ctx.beginPath();ctx.arc(p[0],p[1],3.3,0,Math.PI*2);ctx.fill();if(scale>.55){ctx.fillStyle='#f6d9dc';ctx.fillText(x.id,p[0]+5,p[1]-5);}});}
 if(enabled.labels&&scale>.12){ctx.font='9px ui-monospace';ctx.fillStyle='#b8f3d0';MAP.centerlines.forEach(x=>{const p=s(x.p[Math.floor(x.p.length/2)]);ctx.fillText(`${x.speed||'?'} ${x.direction||''}`,p[0]+3,p[1]-3);});}
}
canvas.addEventListener('wheel',e=>{e.preventDefault();const r=canvas.getBoundingClientRect(),mx=e.clientX-r.left,my=e.clientY-r.top,wx=(mx-ox)/scale,wy=-(my-oy)/scale,f=Math.exp(-e.deltaY*.001);scale*=f;ox=mx-wx*scale;oy=my+wy*scale;draw();},{passive:false});
canvas.addEventListener('mousedown',e=>{dragging=true;last=[e.clientX,e.clientY];canvas.classList.add('dragging');});
window.addEventListener('mouseup',()=>{dragging=false;canvas.classList.remove('dragging');});
window.addEventListener('mousemove',e=>{if(!dragging)return;ox+=e.clientX-last[0];oy+=e.clientY-last[1];last=[e.clientX,e.clientY];draw();});
canvas.addEventListener('dblclick',fit);
function segDist(p,a,b){const vx=b[0]-a[0],vy=b[1]-a[1],wx=p[0]-a[0],wy=p[1]-a[1],d=vx*vx+vy*vy,t=d?Math.max(0,Math.min(1,(wx*vx+wy*vy)/d)):0;return Math.hypot(p[0]-a[0]-t*vx,p[1]-a[1]-t*vy);}
canvas.addEventListener('click',e=>{if(last&&Math.hypot(e.clientX-last[0],e.clientY-last[1])>3)return;const r=canvas.getBoundingClientRect(),p=[e.clientX-r.left,e.clientY-r.top];let best=null,dist=12;
 const consider=(kind,item,points)=>{for(let i=1;i<points.length;i++){const d=segDist(p,s(points[i-1]),s(points[i]));if(d<dist){dist=d;best={kind,...item};}}};
 if(enabled.globalRoute&&MAP.globalRoute.p.length)consider('global_route',MAP.globalRoute,MAP.globalRoute.p);MAP.boundaries.forEach(x=>consider('boundary',x,x.p));MAP.centerlines.forEach(x=>consider('lane/link',x,x.p));MAP.crosswalks.forEach(x=>consider('crosswalk',x,x.p));MAP.surfaceMarkings.forEach(x=>consider('surface_marking',x,x.p));MAP.signals.forEach(x=>{const d=Math.hypot(p[0]-s(x.p)[0],p[1]-s(x.p)[1]);if(d<dist){dist=d;best={kind:'traffic_light',...x};}});
 document.getElementById('inspect').textContent=best?JSON.stringify(best,(k,v)=>k==='p'?undefined:v,2):'선택된 객체가 없습니다.';});
window.addEventListener('resize',resize);resize();
</script>
</body></html>'''


def write_viewer(data, output_path):
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    payload = json.dumps(data, ensure_ascii=False, separators=(",", ":")).replace("</", "<\\/")
    output_path.write_text(HTML_TEMPLATE.replace("__MAP_DATA__", payload), encoding="utf-8")
    return output_path


def open_viewer(path):
    """Open the preview without coupling this package to a ROS visualization topic."""
    uri = Path(path).resolve().as_uri()
    try:
        return webbrowser.open(uri, new=2)
    except webbrowser.Error:
        subprocess.Popen(["xdg-open", uri], stdout=subprocess.DEVNULL,
                         stderr=subprocess.DEVNULL, start_new_session=True)
        return True
