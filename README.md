# rosbridge-web-demo (ROS 2 Jazzy)

A minimal ROS 2 Jazzy + rosbridge + roslibjs demo with a Start/Stop custom message.

Minimal demo to:
- build a custom ROS 2 message `my_msgs/msg/Command`
- run **rosbridge** (WebSocket gateway) in Docker
- publish **Start/Stop** from a web page (roslibjs)
- run a Python listener node (`my_controller/command_listener`) that logs received commands

## Repo layout

```
rosbridge-web-demo/
├─ docker-compose.yml
├─ .dev/
│  ├─ Dockerfile
│  └─ entrypoint.sh      # loads ROS + overlay in all containers      
├─ index.html            # webpage using roslibjs
└─ src/
   ├─ my_controller/
   └─ my_msgs/
```

## 1) Start the stack

From the repo root:

```bash
docker compose up -d --build
docker compose ps
```

Services:
- `ros` – dev shell with your workspace mounted at `/ws`
- `rosbridge` – WebSocket server on **localhost:9090**
- `listener` – runs `my_controller/command_listener` (after the workspace is built)

> If `listener` logs “No executable found” or “Package not found”, see **Troubleshooting**.

## 2) Build the workspace (first time)

Open a shell in the dev container, build, and load the overlay:

```bash
docker compose exec ros bash
# inside container
source /opt/ros/jazzy/setup.bash
cd /ws
rosdep install --rosdistro jazzy --from-paths src -i -y
colcon build --symlink-install --merge-install
source /ws/install/setup.bash
ros2 interface show my_msgs/msg/Command
# expect:
# uint8 START=1
# uint8 STOP=2
# uint8 command
exit
```

Restart services that need to see the new interfaces:

```bash
docker compose restart rosbridge listener
```

## 3) Publish from the browser (roslibjs)

Open **`index.html`** in your browser (double-click is fine).

- It connects to `ws://localhost:9090`.
- Click **Start** / **Stop** to publish `Command` messages (`command=1`/`2`) to `/my_robot/command`.

To watch messages in a terminal (optional):

```bash
docker compose exec ros bash -lc "source /ws/install/setup.bash && ros2 topic echo /my_robot/command"
```

## 4) Run the listener node

```bash
docker compose logs -f listener
```

The Compose service `listener` runs the Python node that subscribes to `/my_robot/command` and logs:

```
[INFO] [command_listener]: Received: START
[INFO] [command_listener]: Received: STOP
```


### Run listener manually instead (optional)

```bash
docker compose exec ros bash
# inside
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
ros2 pkg executables | grep my_controller
ros2 run my_controller command_listener.py   # or 'command_listener' if you installed without .py
```

## Useful commands

```bash
# Open a dev shell
docker compose exec ros bash

# Tail rosbridge logs
docker compose logs -f rosbridge

# Confirm rosbridge port
docker compose port rosbridge 9090   # -> 0.0.0.0:9090

# Stop stack
docker compose down
```

## Troubleshooting

**“Package ‘my_controller’ not found” (listener)**  
Cause: service commands don’t read `~/.bashrc`. We fix this via `entrypoint.sh` which sources ROS and `/ws/install` for every container command; still, the workspace must be built at least once.

- Build once:
  ```bash
  docker compose exec ros bash -lc "source /opt/ros/jazzy/setup.bash && cd /ws && colcon build --symlink-install --merge-install"
  docker compose restart listener
  ```
- Sanity check:
  ```bash
  docker compose exec ros bash -lc "source /ws/install/setup.bash && ros2 pkg list | grep my_controller"
  ```

**“No executable found” (listener)**  
Your installed executable is likely `command_listener.py`. Either:
- Run with suffix in compose:
  ```yaml
  command: ros2 run my_controller command_listener.py
  ```
- Or install without suffix. In `src/my_controller/CMakeLists.txt`:
  ```cmake
  install(PROGRAMS
    scripts/command_listener.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME command_listener
  )
  ```
  Rebuild, then use:
  ```yaml
  command: ros2 run my_controller command_listener
  ```

**Browser can’t connect to rosbridge**  
Ensure only `rosbridge` maps port `9090:9090` in `docker-compose.yml`. If 9090 is in use:
```bash
lsof -iTCP:9090 -sTCP:LISTEN
docker compose down && docker compose up -d
```

**Unknown message `my_msgs/msg/Command`**  
Rebuild and re-source:
```bash
docker compose exec ros bash -lc "source /opt/ros/jazzy/setup.bash && cd /ws && colcon build --symlink-install --merge-install && source /ws/install/setup.bash && ros2 interface show my_msgs/msg/Command"
docker compose restart rosbridge listener
```

## Notes

- The **entrypoint** ensures any service (`listener`, `rosbridge`, etc.) runs with the correct environment:
  ```bash
  #!/usr/bin/env bash
  set -e
  source /opt/ros/jazzy/setup.bash
  if [ -f /ws/install/setup.bash ]; then
    source /ws/install/setup.bash
    case ":$AMENT_PREFIX_PATH:" in *:/ws/install:*) ;; *) export AMENT_PREFIX_PATH="/ws/install:${AMENT_PREFIX_PATH:-}";; esac
    case ":$CMAKE_PREFIX_PATH:" in *:/ws/install:*) ;; *) export CMAKE_PREFIX_PATH="/ws/install:${CMAKE_PREFIX_PATH:-}";; esac
  fi
  exec "$@"
  ```
- `index.html` uses `ws://localhost:9090`. If you ever serve it via **https**, browsers may block `ws://`; use `wss://` with TLS or serve the page over `http://` during development.

Happy bridging!
