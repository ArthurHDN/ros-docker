# ros-docker-arthur
Repository to store custom ROS docker images and configurations

1. Download Docker: https://docs.docker.com/get-docker/
You need to download Docker to manage the containerization.

2. Pull custom ros image
This image contains the full desktop of melodic ROS and noVNC feature to GUI access
```
docker pull arthurhdn/ros:melodic-full-novnc
```

3. cd to this repo and run the compose
This repo provides a simple compose file with the basics configuration for a ros-master container:
- Exposing port to access the noVNC (see step 5)
- sharing /shared/ROS/workspaces with the container, so you can store and edit your workspace content in both host machine (your computer) or container (ros-master)
- Initialize roscore and the script for noVNC
```
cd ros-docker-arthur
docker compose -f src/compose/compose-simple.yml  up
```

4. Enter the container
This step will enter the running container and open a bash on it. It will behave exactly like a terminal for ros-master container and you can enter this command in multiple terminals of the host machine (your computer)
```
docker exec -it ros-master bash
```

5. Open your web browser and access http://localhost:6081
The host port 6081 is defined in the compose file of step 3. Here you will have access to the GUI by connecting to noVNC in your Web Browser.  
