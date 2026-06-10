# Standalone MediaMTX Relay (`compose.mediamtx.yaml`)

A single-container MediaMTX server that pulls video from WildBridge drones and exposes
each one as an **RTSP feed addressed by the drone's IP**. No dashboard, no extra services.

```
rtsp://<this-host>:8554/<drone_ip>     ← what you came for
http://<this-host>:8889/<drone_ip>/whep   (browser / WebRTC playback)
```

---

## How it works

WildBridge phones **publish** video over WHIP; a phone only starts publishing once a
client connects to its telemetry port (`8081`), and it publishes under its configured
**drone name**. For every IP you pass in, this stack:

1. **Triggers** the phone — holds a telemetry connection open to `<ip>:8081`, so the phone
   pushes its camera to *this* host (under its drone name).
2. **Aliases by IP** — discovers the drone name (`GET <ip>:8080/config`) and republishes
   that `<drone_name>` path to a `<drone_ip>` path with `ffmpeg -c copy` (no transcode).

The original `<drone_name>` path still exists (it's the WHIP ingest, used by the
`compose.video-test.yaml` dashboard); the `<drone_ip>` path is an **additional** alias.

---

## Prerequisites

- Docker + Docker Compose on the ground-station machine.
- The drones (phones running WildBridge) are powered on, on the **same network**, and
  reachable from this host on ports `8081` (telemetry) and `8080` (HTTP/config).
- Run from the repo root (it reads `./GroundStation/video_test/mediamtx.yml`).

---

## Quick start — 3 drones

Say you have three drones:

| Drone IP        | Drone name (in-app) |
| --------------- | ------------------- |
| `192.168.1.50`  | `drone_1`           |
| `192.168.1.51`  | `drone_2`           |
| `192.168.1.52`  | `drone_3`           |

### Option A — inline

```bash
DRONE_IPS="192.168.1.50 192.168.1.51 192.168.1.52" \
  docker compose -f compose.mediamtx.yaml up
```

Add `-d` to run detached. IPs may be separated by spaces, commas, or semicolons.

### Option B — `.env` file (recommended)

Create a file named `.env` next to `compose.mediamtx.yaml`:

```dotenv
# .env
DRONE_IPS=192.168.1.50,192.168.1.51,192.168.1.52
```

Then simply:

```bash
docker compose -f compose.mediamtx.yaml up -d
```

On startup you'll see one line per drone:

```
[wildbridge] drone 192.168.1.50  ->  rtsp://<this-host>:8554/192.168.1.50
[wildbridge] drone 192.168.1.51  ->  rtsp://<this-host>:8554/192.168.1.51
[wildbridge] drone 192.168.1.52  ->  rtsp://<this-host>:8554/192.168.1.52
```

---

## Your feeds

Replace `<host>` with this machine's IP (use `localhost` if you're on it directly):

| Drone          | RTSP                              | WHEP (browser)                          |
| -------------- | --------------------------------- | --------------------------------------- |
| `192.168.1.50` | `rtsp://<host>:8554/192.168.1.50` | `http://<host>:8889/192.168.1.50/whep`  |
| `192.168.1.51` | `rtsp://<host>:8554/192.168.1.51` | `http://<host>:8889/192.168.1.51/whep`  |
| `192.168.1.52` | `rtsp://<host>:8554/192.168.1.52` | `http://<host>:8889/192.168.1.52/whep`  |

Play one:

```bash
ffplay rtsp://localhost:8554/192.168.1.50
# or
vlc    rtsp://localhost:8554/192.168.1.50
```

---

## Verify it's working

```bash
# Which streams are live right now (expect the drone_name paths and the IP aliases):
curl -s http://localhost:9997/v3/paths/list | python3 -m json.tool

# Container logs:
docker compose -f compose.mediamtx.yaml logs -f
```

A drone appears only **while it's publishing**. If a path is missing, the phone isn't
publishing yet — check the next section.

---

## Parameters

| Variable                      | Default  | Purpose                                                        |
| ----------------------------- | -------- | -------------------------------------------------------------- |
| `DRONE_IPS`                   | *(empty)*| Space/comma/semicolon list of drone IPs. Empty = passive relay.|
| `TELEMETRY_PORT`              | `8081`   | Phone telemetry port used to trigger publishing.               |
| `HTTP_PORT`                   | `8080`   | Phone HTTP port used to discover the drone name.               |
| `MEDIAMTX_WEBRTC_UDP_ADDRESS` | `:8189`  | WebRTC ICE/UDP address.                                        |

## Ports (host networking)

| Port       | Protocol                          |
| ---------- | --------------------------------- |
| `8554`     | RTSP                              |
| `8889`     | WebRTC — WHIP publish / WHEP play |
| `9997`     | MediaMTX HTTP API                 |
| `8189/udp` | WebRTC ICE/UDP                    |

---

## Stop / clean up

```bash
docker compose -f compose.mediamtx.yaml down
```

> On a host whose Docker daemon is AppArmor/cgroup-confined, `docker stop/kill/rm` can
> fail with `could not kill container: permission denied`. If that happens, disable the
> restart policy and kill the container's host process directly:
> ```bash
> docker update --restart=no wildbridge_mediamtx
> sudo kill -9 "$(docker inspect -f '{{.State.Pid}}' wildbridge_mediamtx)"
> docker rm wildbridge_mediamtx
> ```

---

## Troubleshooting

- **No `<drone_ip>` path appears.** The phone isn't publishing. Confirm the host can reach
  `telnet <drone_ip> 8081` and `curl http://<drone_ip>:8080/config` (should return the
  drone name). Both phone and host must be on the same reachable network.
- **`<drone_ip>` path missing but `<drone_name>` exists.** The IP-alias `ffmpeg` couldn't
  read the name stream yet; it retries every 3 s. Check `docker compose ... logs`.
- **Empty `DRONE_IPS`.** The relay runs but triggers nothing — point each phone here via
  the in-app **Set WHIP Server** (`<host>:8889`) and have a telemetry consumer connect, or
  supply `DRONE_IPS`.
- **Remote WebRTC/WHEP won't connect.** Ensure UDP `8189` is open between viewer and host.
- **Image note.** This uses the `bluenviron/mediamtx:latest-ffmpeg` tag on purpose — it's
  Alpine-based and includes `sh`/`nc`/`ffmpeg`, which the trigger and IP-alias need. The
  default (scratch) image has no shell and won't work here.

---

## Relationship to `compose.video-test.yaml`

`compose.video-test.yaml` is the full diagnostics stack (MediaMTX **+** browser dashboard
on `:8090`). `compose.mediamtx.yaml` is just the MediaMTX relay with IP-addressed RTSP.
**Run one or the other** — both bind `8554/8889/9997`, so they conflict.
