# Alternative installation methods

## Local development environment

1) Install `git`, `make`, `curl`, and `docker`
    ```sh
    sudo apt install git make curl
    curl -sSL https://get.docker.com | sh && sudo usermod -aG docker $USER
    ```
    and reboot the PC.
2) Clone this repository
    ```sh
    # Make sure to have your SSH keys added to GitHub
    git clone git@github.com:memristor/mep3.git
    ```

3) Run provisioning script to pull the image and run the container:
   ```sh
   cd ./mep3/docker
   make run
   ```
   > Time to time you can run `make destroy run` to get the newest packages.

4) Wait for the provisioning script to finish

5) _Optional:_ run container setup script
    ```sh
    make setup-default

    # If you prefer to manually configure setup, you can just do make setup and go through the prompts
    make setup-interactive
    ```
6) Acces the environment from any terminal window
    ```sh
    docker exec -it mep3-devel bash
    ```
    Graphical applications started inside this terminal will use your existing Xorg session to display.


## Code Server
If you prefer to use browser based VS Code, you can start it in the container and then access it locally through your browser at `localhost:31415`
    
```sh
# This will start the VS code server with your mep3 repo
make start-code-server

# To stop the VS code server
make stop-code-server
```

## Remote development environment (VNC)



1) Follow steps in [Local development environment](#local-development-environment), but add `vnc` after
   `make` in steps 4 and 6 (eg. `make vnc setup`)
2) Enable VNC preferences in step 6 and wait for the container to restart
3) Web-based VNC client will be accessible at `http://localhost:6810/` if you keep default noVNC webserver port
4) In step 6 replace `mep3-devel` with `mep3-vnc`

**Note:** If you are setting up through SSH, make sure to have a running Xorg server on host machine,
and set `DISPLAY` environment variable on step 4 to its value (eg `:0`).
```sh
export DISPLAY=:0
```
