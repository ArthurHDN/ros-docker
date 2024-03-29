# ROS-Docker Environment
The power of ROS with Docker encapsulation

1. Download Docker: https://docs.docker.com/get-docker/
You need to download Docker to manage the containerization.

2. Run compose

```bash
docker compose up --build
```

3. Enter the dev container
```bash
docker exec -it ros-dev tmux
```

4. Access dev GUI

```bash
http://localhost:6080
```


Refs

Wiki ROS Docker http://wiki.ros.org/docker/Tutorials

Docker Hub ROS https://hub.docker.com/_/ros

Install docker windows https://docs.docker.com/docker-for-windows/install/

Docker compose install https://docs.docker.com/compose/install/

ROS Docker compose http://wiki.ros.org/docker/Tutorials/Compose

https://tuw-cpsg.github.io/tutorials/docker-ros/

ROS GUI in docker
https://answers.ros.org/question/313786/running-ros-and-its-gui-tools-through-a-docker-image/

Docker noVNC
https://wiki.xnat.org/display/CS/Container+Desktop+Access+via+noVNC

Change docker images location Windows
https://stackoverflow.com/questions/40465979/change-docker-native-images-location-on-windows-10-pro

Windows enable docker WSL2
https://docs.docker.com/desktop/windows/wsl/
