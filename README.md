# DIARC Unity Space Station Simulation

This repository contains the DIARC space station simulation comprised of two executables and a script for launching the docker containers and services required to simulate a PR2 running DIARC.

## Quickstart

1. Clone this repo
1. Download the [Unity space station server](https://github.com/hrilabtufts/DIARC_Space_Station_Simulation/releases/download/repo_release/Space_Station_SMM_Server_3.5.91.zip)
1. Download the [Unity space station client](https://github.com/hrilabtufts/DIARC_Space_Station_Simulation/releases/download/repo_release/Space_Station_SMM_Client_5.5.50.zip)
1. Create your [.env configuration](#script-config)
1. Create your [Unity configuration files](#unity-config)
1. Build the docker image with `bash usd.sh build`
1. Start the docker container with `bash usd.sh run`
1. When ROS has started, launch the Unity server
1. When the Unity server and DIARC have completed launching, start the client

### Unity Server

The server build connects to the virtual PR2 via RosBridge and DIARC. 
It allows one or more client to connect to it and interact with the robot.

This requires a `server_settings.json` file in the `StreamingAssets` directory of the build.
See [Configuring Unity Space Station Server](#server-config) for more information.

* [Space Station Server v3.5.91 - Latest](https://github.com/hrilabtufts/DIARC_Space_Station_Simulation/releases/download/repo_release/Space_Station_SMM_Server_3.5.91.zip)

### Unity Client

The client build connects to the Unity server and provides the user interaction. 
When using a[Vive Pro Eye](https://www.vive.com/sea/product/vive-pro-eye/overview/) and with the [SRanipal](https://docs.vrcft.io/docs/hardware/VIVE/sranipal) eye tracking software installed, user eye gaze and pupil size will be tracked by the application.

This requires a `client_settings.json` file in the `StreamingAssets` directory of the build.
See [Configuring Unity Space Station Client](#client-config) for more information.

* [Space Station Client v5.5.50 - Latest](https://github.com/hrilabtufts/DIARC_Space_Station_Simulation/releases/download/repo_release/Space_Station_SMM_Client_5.5.50.zip)

### Script Dependencies

* [docker](https://docs.docker.com/engine/install/ubuntu/)
* [docker compose](https://docs.docker.com/compose/install/linux/)

<a name="script-config"></a>
### Script Usage

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

A `.env` file defines all of the environment variables needed to launch the space station simulation on your machine or network.
It must be created in this project directory and available to all scripts.

Example, also found in default.env:

```
BIN="docker compose"
UNITY_IP="127.0.0.1"
DIARC_CONFIG="edu.tufts.hrilab.config.unity.UnityDIARCSpaceStation"
WAIT=40
```

`BIN` is the command that will be used to launch the docker-compose.yaml file that is generated by this script.
If your system requires root to run this command it should be changed to `sudo docker compose`.
If you have installed the `docker-compose` script instead of the plugin, it should be changed to `docker-compose` or `sudo docker-compose` if your system requires root to launch containers.

`UNITY_IP` is the IP address of the machine the Unity server build is running on.
If this is a separate machine from the one running these docker containers it should have the port 1755 is open for network connections.

`DIARC_CONFIG` is the configuration file to launch on startup.
If you are developing your own DIARC configuration, change this value to point to your new class.

`WAIT` is the time, in seconds, between starting up ROS and starting up DIARC during which the Unity server needs to be started.

### Script Commands

#### build

The docker image must be built before the container with DIARC can be launched.

```bash
bash usd.sh build
```

In the case that your docker installation requires root to run:

```bash
sudo bash usd.sh build
```

#### run

Run DIARC and ROS on localhost.
This will start a single container with the diarc configuration.
Robot will connect to `127.0.0.1:8000` on the Unity server and publish a Rosbridge websocket server on port `9090`.

If your docker installation requires root to run, do not run this command with `sudo`, instead update your `.env` file's `BIN` variable to be `sudo docker compose` or `sudo docker-compose` as needed.

```bash
bash usd.sh run
```
This will start and launch associated ROS and DIARC.

```bash
bash usd.sh run
```

#### dev

Run DIARC in debug mode and use a development repository which can be edited locally.
This will rebuild the `unity` component in `../local_ade_repo` and launch DIARC with debug port `5005` expecting a connection from IntelliJ to start DIARC.

```bash
bash usd.sh dev ../diarc
```

<a name="unity-config"></a>
### DIARC Space Station Simulation Settings Generator

To generate settings files for the server and client, you can use the following web app or read below for more information.

[https://hrilabtufts.github.io/DIARC_Space_Station_Simulation_Settings_Generator/](https://hrilabtufts.github.io/DIARC_Space_Station_Simulation_Settings_Generator/)

<a name="server-config"></a>
### Configuring Unity Space Station Server

The following is the default `server_settings.json` file with all required values to configure the Unity server.

```json
{
    "created" : "2024-02-28T20:43:24.460Z",
    "name" : "Space Station SMM Study",
    "type" : "server",
    "room" : "Test Room 1",
    "maxPlayers" : 1,
    "tubeOnDecayRate" : 1.0,
    "tubeOffDecayRate" : 0.5,
    "tubeRepairRate" : 5.0,
    "stationNotifications" : true,
    "truncateRepairStatements" : true,
    "networkConnectionUrl" : "127.0.0.1",
    "networkConnectionPort" : 8868,
    "useLSL" : false,
    "DIARC" : [
        {
            "port" : 1755,
            "ROS" : [
                {
                    "model" : "PR2",
                    "IP" : "127.0.0.1",
                    "port" : 9090,
                    "voice" : "default"
                }
            ]
        }
    ],
    "trials" : [
        {
            "seconds" : 300,
            "robots" : 1,
            "survey" : true,
            "tubes" : [
                {
                    "time" : 10
                }
            ]
        },
        {
            "seconds" : 300,
            "robots" : 1,
            "survey" : true,
            "tubes" : [
                {
                    "time" : 10
                }
            ],
            "rovers" : [
                {
                    "time" : 40
                }
            ]
        },
        {
            "seconds" : 300,
            "robots" : 1,
            "survey" : true,
            "tubes" : [
                {
                    "time" : 10
                }
            ],
            "rovers" : [
                {
                    "time" : 45
                }
            ]
        },
        {
            "seconds" : 300,
            "robots" : 1,
            "survey" : true,
            "tubes" : [
                {
                    "time" : 10
                }
            ],
            "rovers" : [
                {
                    "time" : 40
                }
            ]
        }
    ]
}
```

* **created** - String of date showing when the configuration was made 
* **name** - Name of the study, to be included in all data files
* **type** - Type of configuration. Must be `server` or program will exit. Used to prevent using server configuration on the server.
* **room** - Name of room server is located. Used to track origin of data in multi subject studies.
* **maxPlayers** - Integer defining the maxiumum number of players allowed to connect to the server at once. Default is `1`.
* **tubeOnDecayRate** - Float defining how fast tubes will decay when damaged. Health points per second. Defaults to `1.0`.
* **tubeOffDecayRate** - Float defining how fast tubes will decay when damaged and turned off. Health points per second. Defaults to `0.5`.
* **tubeRepairRate** - Float defining how fast tube repair occurs. Health points per second. Defaults to `5.0`.
* **stationNotifications** - Boolean determining whether to allow space station notifications to be audible. Defaults to `true`.
* **truncateRepairStatements** - Boolean determining whether to shorten utterances containing the word "repair". Defaults to `true`.
* **networkConnectionUrl** - IP address of the machine the server is running on for it to bind to. Defaults to `127.0.0.1` but should be a remotely-accessible address if using multiple machines.
* **networkConnectionPort** - Integer defining the port to run the server on. Defaults to `8868`.
* **useLSL** - Boolean defining whether to public clock data to LSL. Defaults to `false`.

<a name="client-config"></a>
### Configuring Unity Space Station Client

The following is the default `client_settings.json` file with all required values to configure the Unity client.

```json
{
  "created" : "2024-02-28T20:43:24.460Z",
  "name" : "Space Station SMM Study",
  "type" : "client",
  "room" : "Test Room 1",
  "movementType" : "jump",
  "movementHopDistance" : 3,
  "kaldiASRUrl" : "ws://127.0.0.1",
  "kaldiASRPort" : 2700,
  "openTTSUrl" : "http://127.0.0.1",
  "openTTSPort" : 5500,
  "networkConnectionUrl" : "127.0.0.1",
  "networkConnectionPort" : 8868,
  "calibrateEyeTracker" :  false,
  "useLSL" : false,
  "allowCrosstalk" : false
}
```

* **created** - String of date showing when the configuration was made 
* **name** - Name of the study, to be included in all data files
* **type** - Type of configuration. Must be `client` or program will exit. Used to prevent using server configuration on the client.
* **room** - Name of room client is located. Used to track origin of data in multi subject studies.
* **movementType** - Type of movement controls on the Vive Pro Eye. Either `jump` or `linear`.
* **movementHopDistance** - Integer defining the distance in a jump. Defaults to `3`.
* **kaldiASRUrl** - IP or URL of the Kaldi ASR server used for speech transcription. Must start with `ws://` or `wss://` protocol. When using this script it will be the IP of the machine that `bash usd.sh run` is executed on.
* **kaldiASRPort** - Integer for the port of the Kaldi ASR server. If using this script, will be `2700`.
* **openTTSUrl** - IP or URL of the OpenTTS server used for text to speech. When using this script it will be the IP of the machine that `bash usd.sh run` is executed on
* **openTTSPort** - Integer for the port of the OpenTTS server. If using this script, will be `5500`.
* **networkConnectionUrl** - IP address of the machine the server is run on. No preceeding protocol.
* **networkConnectionPort** - Integer for the port of the server. Default is `8868`.
* **calibrateEyeTracker** - Boolean defining whether to run the eye tracker calibration script on startup. Defaults to `false`.
* **useLSL** - Boolean defining whether to public clock data to LSL. Defaults to `false`.
* **allowCrosstalk** - Boolean defining whether to allow robot and space station agents to talk over one another with generated speech. Defaults to `false`.