# Alternative installation methods

## Local development environment

1) Install `git`, `make` and `docker`
1) Run docker daemon and add yourself to docker group
    ```sh
    sudo systemctl enable docker.service
    sudo systemctl start docker.service
    sudo usermod -aG docker $USER
    ```
2) Clone this repository
    ```sh
    # Make sure to have your SSH keys added to GitHub
    git clone git@github.com:memristor/mep3.git
    ```
3) Run provisioning script to build and run the container
   ```sh
   cd ./mep3/docker
   make build run
   ```

4) Wait for the provisioning script to finish

5) Acces the environment from any terminal window
    ```sh
    docker exec -it mep3-devel bash
    ```
    Graphical applications started inside this terminal will use your existing Xorg session to display.

## Remote development environment (VNC)

1) Follow [Local development environment](#local-development-environment) steps 1 to 4  
   Note: If you are setting up through SSH, make sure to have a running Xorg server on host machine,
    and set `DISPLAY` environment variable on step 3 to its value (eg `:0`).
    ```sh
    DISPLAY=:0 make build run
    ```
2) Run container dynamic setup script
    ```sh
    make setup
    ```
3) Select your preferences and wait for the container to restart
4) Web-based VNC client will be accessible at `http://localhost:6810/`.
