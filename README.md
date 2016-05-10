# Automated Install ROS on Ubilinux Intel Edison Board
The scripts in this repository will connect from a host machine over serial to an Intel Edison board running Ubilinux and execute 

Built and tested on OS X.

## Important Notes
- These scripts are not production-hardened, but should contain all of the instructions required to install ROS and have it operational.
- If commands run and fail, the installation will be aborted.
- You can resolve issues that cause a command to fail and then safely re-run the installation at any point.
- This installer should be run on an otherwise clean install of Ubilinux on Intel Edison.

## Instructions
1. Install Ubilinux on Intel Edison
  - See this guide to install Ubilinux on Intel Edison: <TODO>
- Connect the Edison to the host machine via both USB cables
- Clone this repository somewhere on the host machine
  - `git clone https://github.com/c-h-/edion_install_ros.git`
  - `cd edison_install_ros`
- Two Options:
  - Run the installer from the host machine
    - `./run_on_host.sh`
  - Run the Edison based installer from Edison
    - copy `run_on_ubilinux_edison.sh` and `run_on_ubilinux_edison_optional_part2.sh` onto the Edison board
    - Run `run_on_ubilinux_edison.sh` first, then run `run_on_ubilinux_edison_optional_part2.sh` if desired
      - `run_on_ubilinux_edison_optional_part2.sh` installs the tutorials and Android sensor topic subscriber.
