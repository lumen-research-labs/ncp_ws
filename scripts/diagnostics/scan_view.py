#!/usr/bin/env python3
"""
scan_view.py — a live, unmasked view of the raw /scan, served from the robot.

What it does:
    Subscribes to a LaserScan topic and serves one web page that draws every
    ray of every scan: a dot for each return, coloured by range, and a faint
    tick at the plot edge for each ray with NO return, so a blind sector reads
    as a blank wedge of ticks rather than as nothing. A table gives, per 30
    degree sector, how many rays returned and the nearest range, plus the
    scan rate and the single nearest return with its bearing. No mask, no
    resampling, no zones: this is the driver's output and nothing else.

Why it exists:
    The dashboard shows the scan after the stack's own reductions, decimated
    for the page. When a lidar seems to see nothing where a hand is (an A1's
    15 cm dead zone, a mount that blocks half the window, a mirrored or
    rotated frame) the question is what the DRIVER publishes, bearing by
    bearing, live. 2026-11-18, bernard-3: the RPLIDAR A1's whole front half
    returned nothing in five consecutive scans.

Run (on the robot, with the stack or just the lidar driver up):
    python3 scripts/diagnostics/scan_view.py              # http://<robot>:8090/
    python3 scripts/diagnostics/scan_view.py --topic /scan_sanitized --port 8091

Publishes:   nothing. Subscribes to the scan topic only. Cannot move the robot.
Depends on:  rclpy, sensor_msgs, the standard library. No pip.
"""

from __future__ import annotations

import argparse
import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
import rclpy.node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

_LATEST = {"scan": None, "stamps": []}
_LOCK = threading.Lock()


class _ScanTap(rclpy.node.Node):
    def __init__(self, topic: str) -> None:
        super().__init__("scan_view")
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(LaserScan, topic, self._on_scan, qos)
        self.get_logger().info(f"scan_view: watching {topic}")

    def _on_scan(self, msg: LaserScan) -> None:
        now = time.monotonic()
        ranges = [None if (r is None or not math.isfinite(r) or r <= 0.0) else round(float(r), 3)
                  for r in msg.ranges]
        with _LOCK:
            _LATEST["stamps"] = [t for t in _LATEST["stamps"] if now - t < 5.0] + [now]
            _LATEST["scan"] = {
                "frame_id": msg.header.frame_id,
                "angle_min": float(msg.angle_min), "angle_increment": float(msg.angle_increment),
                "range_min": float(msg.range_min), "range_max": float(msg.range_max),
                "ranges": ranges, "received_at": now,
            }


_PAGE = """<!doctype html><html><head><meta charset="utf-8"><title>scan view</title>
<style>
body{font-family:system-ui,sans-serif;background:#111;color:#ddd;margin:0;display:flex;flex-wrap:wrap}
#left{padding:12px}#right{padding:12px;font-size:14px;min-width:330px}
canvas{background:#000;border:1px solid #333}
table{border-collapse:collapse}td,th{padding:2px 8px;text-align:right;border-bottom:1px solid #222}
th{color:#999;font-weight:normal}.blind{color:#f55}.ok{color:#5d5}
select,button{background:#222;color:#ddd;border:1px solid #444;padding:3px 6px}
.legend span{display:inline-block;margin-right:12px}.legend i{display:inline-block;width:10px;height:10px;margin-right:4px;border-radius:50%}
</style></head><body>
<div id="left"><canvas id="c" width="720" height="720"></canvas>
<div style="margin-top:8px">plot radius <select id="scale"><option>1</option><option>2</option><option selected>4</option><option>8</option><option>12</option></select> m
&nbsp; <span class="legend"><span><i style="background:#f33"></i>&lt; 0.3 m</span><span><i style="background:#fa3"></i>&lt; 0.6 m</span><span><i style="background:#5d5"></i>return</span><span><i style="background:#444;border-radius:0"></i>no return (tick at edge)</span></span></div></div>
<div id="right"><h3 style="margin:4px 0">raw <span id="topic"></span></h3>
<div id="status">waiting for a scan…</div>
<table id="t"><thead><tr><th>sector (0 ahead, + left)</th><th>returns / rays</th><th>nearest</th></tr></thead><tbody></tbody></table>
<p style="color:#888;max-width:330px">Driver output only: no mask, no resampling. A sector reading 0 returns while something is there is the lidar not seeing it (dead zone under its range_min, or its window blocked). The dead-zone ring is dashed red.</p></div>
<script>
const c=document.getElementById('c'),ctx=c.getContext('2d');const W=c.width,H=c.height,cx=W/2,cy=H/2;
function draw(s){const R=parseFloat(document.getElementById('scale').value);const px=m=>m/R*(W/2-10);
ctx.clearRect(0,0,W,H);ctx.strokeStyle='#333';ctx.lineWidth=1;
for(const m of [0.5,1,2,3,4,6,8,10,12]){if(m>R)break;ctx.beginPath();ctx.arc(cx,cy,px(m),0,2*Math.PI);ctx.stroke();ctx.fillStyle='#555';ctx.fillText(m+' m',cx+px(m)+2,cy-2);}
ctx.setLineDash([4,4]);ctx.strokeStyle='#a33';ctx.beginPath();ctx.arc(cx,cy,px(s.range_min),0,2*Math.PI);ctx.stroke();ctx.setLineDash([]);
ctx.strokeStyle='#2a6';ctx.beginPath();ctx.moveTo(cx,cy);ctx.lineTo(cx,cy-px(Math.min(R,0.5)));ctx.stroke();ctx.fillStyle='#2a6';ctx.fillText('ahead',cx+4,cy-px(Math.min(R,0.5)));
ctx.fillStyle='#666';ctx.fillText('left',14,cy);ctx.fillText('right',W-40,cy);ctx.fillText('behind',cx-18,H-6);
let nearest=null;const sect={};
for(let i=0;i<s.ranges.length;i++){const a=s.angle_min+i*s.angle_increment;const r=s.ranges[i];
 const deg=((a*180/Math.PI+180)%360+360)%360-180;const k=Math.floor(deg/30)*30;if(!sect[k])sect[k]={n:0,hit:0,near:null};sect[k].n++;
 const ux=-Math.sin(a),uy=-Math.cos(a);
 if(r===null){ctx.strokeStyle='#333';ctx.beginPath();ctx.moveTo(cx+ux*(W/2-10),cy+uy*(W/2-10));ctx.lineTo(cx+ux*(W/2-2),cy+uy*(W/2-2));ctx.stroke();continue;}
 sect[k].hit++;if(sect[k].near===null||r<sect[k].near)sect[k].near=r;if(nearest===null||r<nearest.r)nearest={r:r,deg:deg};
 const d=Math.min(r,R);ctx.fillStyle=r<0.3?'#f33':r<0.6?'#fa3':'#5d5';ctx.beginPath();ctx.arc(cx+ux*px(d),cy+uy*px(d),r<0.6?3.5:2.5,0,2*Math.PI);ctx.fill();}
const tb=document.querySelector('#t tbody');tb.innerHTML='';
for(const k of Object.keys(sect).map(Number).sort((a,b)=>a-b)){const v=sect[k];const tr=document.createElement('tr');
 tr.innerHTML=`<td>${k>=0?'+':''}${k} … ${k+30>=0?'+':''}${k+30}°</td><td class="${v.hit?'ok':'blind'}">${v.hit} / ${v.n}</td><td>${v.near===null?'—':v.near.toFixed(2)+' m'}</td>`;tb.appendChild(tr);}
const hits=s.ranges.filter(x=>x!==null).length;
document.getElementById('status').innerHTML=`<b>${s.hz.toFixed(1)} Hz</b> · ${s.ranges.length} rays · <b>${hits}</b> with a return · range_min ${s.range_min} m · frame ${s.frame_id}<br>nearest: <b>${nearest?nearest.r.toFixed(2)+' m at '+nearest.deg.toFixed(0)+'°':'none'}</b> · age ${s.age_s.toFixed(1)} s`;}
async function tick(){try{const r=await fetch('/scan.json',{cache:'no-store'});const s=await r.json();document.getElementById('topic').textContent=s.topic;if(s.ranges)draw(s);}catch(e){}setTimeout(tick,150);}
tick();
</script></body></html>"""


def _make_handler(topic: str):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):  # quiet
            pass

        def do_GET(self):
            if self.path.startswith("/scan.json"):
                with _LOCK:
                    scan = dict(_LATEST["scan"]) if _LATEST["scan"] else {}
                    stamps = list(_LATEST["stamps"])
                now = time.monotonic()
                hz = (len(stamps) - 1) / (stamps[-1] - stamps[0]) if len(stamps) > 1 and stamps[-1] > stamps[0] else 0.0
                scan["hz"] = hz
                scan["age_s"] = now - scan["received_at"] if scan else math.inf
                scan.pop("received_at", None)
                scan["topic"] = topic
                body = json.dumps(scan).encode()
                self.send_response(200); self.send_header("Content-Type", "application/json")
            else:
                body = _PAGE.encode()
                self.send_response(200); self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body))); self.end_headers()
            self.wfile.write(body)
    return Handler


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--topic", default="/scan")
    ap.add_argument("--port", type=int, default=8090)
    args = ap.parse_args()
    rclpy.init()
    node = _ScanTap(args.topic)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    server = ThreadingHTTPServer(("0.0.0.0", args.port), _make_handler(args.topic))
    print(f"scan_view: http://0.0.0.0:{args.port}/  ({args.topic})", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
