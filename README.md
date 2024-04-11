# DIARC Unity Space Station Simulation

This project contains the DIARC space station simulation comprised of two executables and a script for launching the docker containers and services required to simulate a PR2 running DIARC.



### Unity Server

The server build connects to the virtual PR2 via RosBridge and DIARC. 
It allows one or more client to connect to it and interact with the robot.

* [Space Station Server v3.5.91 - Latest]()

### Unity Client

The client build connects to the Unity server and provides the user interaction. 
When using a[Vive Pro Eye](https://www.vive.com/sea/product/vive-pro-eye/overview/) and with the [SRanipal](https://docs.vrcft.io/docs/hardware/VIVE/sranipal) eye tracking software installed, user eye gaze and pupil size will be tracked by the application.

* [Space Station Client v5.5.50 - Latest]()

### Dependencies

* [docker](https://docs.docker.com/engine/install/ubuntu/)
* [docker compose](https://docs.docker.com/compose/install/linux/)


### Usage

```
Usage: usd [command] <options>
Commands:
  build   - Build this container with optional -f/--force arguments for forcing re-build of entire image
  run     - Run container: usd run <UNITY SERVER IP> [GUI ON]
  dev     - Run container in development mode: usd dev <LOCAL REPO PATH> [GUI ON]
  stop    - Gracefully bring down container. Not needed for docker compose commands run and dev.
  kill    - Ungracefully bring down container. Not needed for docker compose commands run and dev.
```

### Configuration

A `.env` file defines all of the 

```
BIN="docker compose"
UNITY_IP="127.0.0.1"
DIARC_CONFIG="edu.tufts.hrilab.config.unity.UnityDIARCSpaceStation"
WAIT=40
```

### Script Commands

#### run

Run DIARC and ROS on localhost.
This will start a single container with the diarc configuration.
Robot will connect to `127.0.0.1:1755` on the Unity server and publish a Rosbridge websocket server on port `9090`.

```bash
usd run
```
This will start and launch associated ROS and DIARC.

```bash
usd run 127.0.0.1 true
```

#### dev

Run DIARC in debug mode and use a development repository which can be edited locally.
This will rebuild the `unity` component in `../local_ade_repo` and launch DIARC with debug port `5005` expecting a connection from IntelliJ to start DIARC.

```bash
usd dev ../diarc
```
