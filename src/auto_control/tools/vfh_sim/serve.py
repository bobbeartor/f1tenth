"""Serve the existing course editor with selectable local planners on localhost."""
import argparse
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import subprocess
import tempfile
from threading import Lock
from vehicle_profile import load_profile, fingerprint, write_header

HERE = Path(__file__).resolve().parent
SRC = HERE.parents[2]


def build_planner(binary, profile=None):
    profile=profile or load_profile()
    profile.setdefault("fingerprint",fingerprint(profile))
    write_header(Path(binary).parent / "simulation_profile.hpp",profile)
    subprocess.run([
        "c++", "-std=c++17", "-O3", "-DNDEBUG",
        "-I" + str(Path(binary).parent),
        "-I" + str(SRC / "roi_vfh_test/include"),
        "-I" + str(SRC / "auto_control/include"),
        str(HERE / "native_planner.cpp"),
        str(SRC / "auto_control/src/control_core.cpp"),
        "-o", str(binary),
    ], check=True)


def page(planner, scene=None, profile=None):
    profile=profile or load_profile()
    profile.setdefault("fingerprint",fingerprint(profile))
    title = "프레넷" if planner == "frenet" else "VFH+"
    fragment = (HERE / "view.html").read_text()
    scripts = {
        "CORE": (HERE / "core.js").read_text() + "\n" + (HERE / "track.js").read_text() + "\n" + (HERE / "native.js").read_text(),
        "UI": (HERE / "ui.js").read_text(),
    }
    for key, script in scripts.items():
        fragment = fragment.replace(f"/*__SIM_{key}__*/", script.replace("</script", "<\\/script"))
    native_profiles=json.loads((HERE / "native-presets.json").read_text())
    encoded_profiles=json.dumps(native_profiles,ensure_ascii=True).replace("</", "<\\/")
    encoded_scene=json.dumps(scene,ensure_ascii=True).replace("</", "<\\/")
    fragment=fragment.replace("/*__SIM_SCENE__*/",
        f"window.VFHSimNativeProfiles={encoded_profiles};window.VFHSimInitialScene={encoded_scene};")
    return ('<!doctype html><html lang="ko"><meta charset="utf-8">'
            '<meta name="viewport" content="width=device-width,initial-scale=1">'
            f'<title>{title} 로컬 시뮬레이션</title>'
            '<style>:root{--foreground:#202632;--background:#fff;--muted:#aab3bd;'
            '--muted-foreground:#526070;--border:#cad2dd;--viz-series-1:#146ed0;'
            '--viz-series-2:#d04435;--viz-series-3:#dd8a24;--destructive:#af1624}'
            'body{font:15px system-ui;margin:24px}button,input,select{font:inherit}'
            'button{padding:5px 9px}td,th{padding:4px 10px;text-align:left}</style>'
            f'<h1>{title} 로컬 시뮬레이션</h1><script>window.simPlanner={planner!r};'
            f'window.VFHSimVehicleProfile={json.dumps(profile,ensure_ascii=True)};</script>' + fragment + '</html>').encode()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--planner", choices=("frenet", "vfh"), default="vfh")
    parser.add_argument("--scene", type=Path, help="Load a saved obstacle/mark scene and its configuration")
    parser.add_argument("--params-file",type=Path,help="ROI VFH preview YAML (defaults to the real vehicle profile)")
    parser.add_argument("--reference-dir",type=Path,help="ROI VFH reference YAML directory")
    args = parser.parse_args()
    profile=load_profile(args.params_file,args.reference_dir)
    profile["fingerprint"]=fingerprint(profile)
    scene=json.loads(args.scene.read_text(encoding="utf-8")) if args.scene else None
    if args.scene and args.scene.resolve()==(HERE/"scenes/default.json").resolve():
        # The bundled layout carries no separate tuning authority.
        scene["config"].update(profile["defaults"])
    document = page(args.planner,scene,profile)
    with tempfile.TemporaryDirectory(prefix="vfh-sim-") as temp:
        binary = Path(temp) / "planner"
        build_planner(binary,profile)
        with subprocess.Popen([str(binary), args.planner], stdin=subprocess.PIPE, stdout=subprocess.PIPE) as worker:
            worker_lock = Lock()

            class Handler(BaseHTTPRequestHandler):
                def reply(self, status, kind, data):
                    self.send_response(status)
                    self.send_header("Content-Type", kind)
                    self.send_header("Content-Length", str(len(data)))
                    self.send_header("Cache-Control","no-store")
                    self.end_headers()
                    self.wfile.write(data)

                def do_GET(self):
                    if self.path == "/":
                        self.reply(200, "text/html; charset=utf-8", document)
                    elif self.path == "/profile":
                        try:
                            current=load_profile(args.params_file,args.reference_dir)
                            restart=fingerprint(current)!=profile["fingerprint"]
                        except (OSError,ValueError,KeyError):
                            restart=True
                        self.reply(200,"application/json",json.dumps({
                            "fingerprint":profile["fingerprint"],"restartRequired":restart,
                            "defaults":profile["defaults"]}).encode())
                    else:
                        self.send_error(404)

                def do_POST(self):
                    if self.path != "/plan":
                        self.send_error(404)
                        return
                    size = self.headers.get("Content-Length", "")
                    if not size.isdecimal() or not 0 < int(size) <= 65536:
                        self.send_error(400)
                        return
                    payload = self.rfile.read(int(size))
                    if not payload.startswith(("V2 "+profile["fingerprint"]+" ").encode()):
                        self.reply(409,"application/json",json.dumps({"error":"서버/페이지 정책 버전이 다릅니다. 배치를 JSON으로 보관한 뒤 페이지를 새로고침하세요."},ensure_ascii=False).encode())
                        return
                    if b"\n" in payload or b"\r" in payload:
                        self.send_error(400)
                        return
                    try:
                        with worker_lock:
                            worker.stdin.write(payload + b"\n")
                            worker.stdin.flush()
                            result = worker.stdout.readline()
                        if not result:
                            raise BrokenPipeError
                    except OSError:
                        self.send_error(503, "Native planner unavailable")
                        return
                    self.reply(200, "application/json", result)

                def log_message(self, format, *args):
                    if self.path not in ("/plan","/profile"):
                        super().log_message(format, *args)

            with ThreadingHTTPServer(("127.0.0.1", args.port), Handler) as server:
                print(f"http://127.0.0.1:{args.port}/", flush=True)
                try:
                    server.serve_forever()
                except KeyboardInterrupt:
                    pass
                finally:
                    worker.stdin.close()


if __name__ == "__main__":
    main()
