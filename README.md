# ros-docker-arthur
Repository to store custom ROS docker images and configurations

1. Download Docker: https://docs.docker.com/get-docker/

2. Pull ros image
```
arthurhdn/ros:melodic-full-novnc
```

3. cd to the repo and run the compose
```
docker compose -f src/compose/compose-simple.yml  up
```

4. Enter the container
```
docker exec -it ros-master bash
```

5. Open your web browser and access
```
http://localhost:6081
```
