# Alternative installation methods

## Local development environment

1. Install `git`, `make`, `curl`, and `docker`
   ```sh
   sudo apt install git make curl
   curl -sSL https://get.docker.com | sh && sudo usermod -aG docker $USER
   ```
   and reboot the PC.
2. Clone this repository

   ```sh
   # Make sure to have your SSH keys added to GitHub
   git clone git@github.com:memristor/mep3.git
   ```

3. Run provisioning script to pull the image and run the container:

   ```sh
   cd ./mep3/docker
   make all
   ```
   If you want to build image localy you can use command:
   ```sh
     IMAGE=mep3 make build destroy run exec
   ``` 

   > Time to time you can run `make destroy run` to get the newest packages.

4. Wait for the provisioning script to finish

5. Run the `cb` command in the opened terminal. (`cb` is a shortcut for `colcon build` with the necessary arguments.)

6. Run the command `source ./install/setup.bash` or exit and re-enter the container. This step is necessary every time you start the build process.

5. _Optional:_ run interactive setup if you prefer to manually configure the container

   ```sh
   make setup-interactive
   ```

6. Acces the environment from any terminal window
   ```sh
   docker exec -it mep3-devel bash
   ```
   Graphical applications started inside this terminal will use your existing Xorg session to display.

## Remote development environment (VNC)

1. Follow steps in [Local development environment](#local-development-environment), but add `vnc` after
   `make` in steps 4 and 6 (eg. `make vnc setup`)
2. Enable VNC preferences in step 6 and wait for the container to restart
3. Web-based VNC client will be accessible at `http://localhost:6810/` if you keep default noVNC webserver port
4. In step 6 replace `mep3-devel` with `mep3-vnc`

**Note:** If you are setting up through SSH, make sure to have a running Xorg server on host machine,
and set `DISPLAY` environment variable on step 4 to its value (eg `:0`).

```sh
export DISPLAY=:0
```

## NVIDIA GPU

If you happen to have NVIDIA GPUs that you wish to use within these development environments, make sure
to have NVIDIA Container Toolkit installed on your system. More info, specific to your distribution [here](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).
