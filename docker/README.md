# Alternative installation methods

## Local development environment

1) Install `git`, `make` and `docker`

2) If you have an NVIDIA graphics card, install drivers. Tutorial is [`here`][https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html]

3) Run docker daemon and add yourself to docker group
    ```sh
    sudo systemctl enable docker.service
    sudo systemctl start docker.service
    sudo usermod -aG docker $USER
    ```
4) Clone this repository
    ```sh
    # Make sure to have your SSH keys added to GitHub
    git clone git@github.com:memristor/mep3.git
    ```
5) Run provisioning script to build and run the container
   ```sh
   cd ./mep3/docker
   make build run
   ```

6) Wait for the provisioning script to finish

7) _Optional:_ run container setup script
    ```sh
    make setup
    ```
8) Acces the environment from any terminal window
    ```sh
    docker exec -it mep3-devel bash
    ```
    Graphical applications started inside this terminal will use your existing Xorg session to display.

## Remote development environment (VNC)



1) Follow steps in [Local development environment](#local-development-environment), but add `vnc` after
   `make` in steps 4 and 6 (eg. `make vnc setup`)
2) Enable VNC preferences in step 6 and wait for the container to restart
3) Web-based VNC client will be accessible at `http://localhost:6810/` if you keep default noVNC webserver port
4) In step 7 replace `mep3-devel` with `mep3-vnc`

**Note:** If you are setting up through SSH, make sure to have a running Xorg server on host machine,
and set `DISPLAY` environment variable on step 4 to its value (eg `:0`).
```sh
export DISPLAY=:0
```
