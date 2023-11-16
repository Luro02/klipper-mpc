klipper-mpc
===

Implementation of Model Predictive Temperature Control for klipper.

# ⚠️ Warning ⚠️

This repository implements a different algorithm for heating and holding the
3d printer at a target temperature. The code itself has not been tested thoroughly,
therefore it is not recommended to leave your 3d printer alone with the new algorithm
enabled.

A 3d printer can become very hot, in the worst-case it could catch fire.
There are safety measures implemented in klipper to prevent this, but there is no gurantee
that they will work.


# Installation

First open a shell on the computer which has klipper installed, and then execute the following commands.

Navigate into your home directory
```console
user@pi:~$ cd ~/
```

Clone the repository
```console
user@pi:~$ git clone git@github.com:Luro02/klipper-mpc.git
```

Then open the `~/printer_data/config/moonraker.conf` with an editor like `nano` and add the following code to the end of it:
```console
user@pi:~$ nano ~/printer_data/config/moonraker.conf
```

```ini
[update_manager klipper-mpc]
type: git_repo
channel: dev
path: ~/klipper-mpc
origin: https://github.com/Luro02/klipper-mpc.git
managed_services: klipper
primary_branch: master
install_script: install.sh
is_system_service: False
```

Please ensure that the following sections are present (or included) in your `printer.cfg`.
```ini
[respond]
#default_type: echo
#   Sets the default prefix of the "M118" and "RESPOND" output to one
#   of the following:
#       echo: "echo: " (This is the default)
#       command: "// "
#       error: "!! "
#default_prefix: echo:
#   Directly sets the default prefix. If present, this value will
#   override the "default_type".
```

Depending on your setup, some of those sections might be set in other included configs like `mainsail.cfg`.

Then execute the `install.sh` script:
```console
user@pi:~$ cd ~/klipper-mpc
user@pi:~/klipper-mpc$ ./install.sh
```

# Enabling MPC

For each heater that you want to enable MPC on, a section has to be added in the `~/printer_data/config/printer.cfg`.
At the moment only the `extruder` works.

```ini
[mpc extruder]
include_fan: True
heater_power: 40.0
```

Before activating MPC, one should calibrate it first, which can be done by executing the command:
```
MPC_CALIBRATE HEATER=extruder
```

and then it can be enabled with
```
MPC_ENABLE HEATER=extruder
```

# License

This implementation is based on the [Marlin](https://github.com/MarlinFirmware/Marlin) code for
Model Predictive Temperature Control. It was at first a direct port of the source code.

The following resources have been used for this port:
- https://marlinfw.org/docs/features/model_predictive_control.html
- https://github.com/Klipper3d/klipper/blob/master/klippy/extras/heaters.py (current PID implementation)
- https://github.com/MarlinFirmware/Marlin/pull/23751 (PR that introduced MPC to Marlin)
- https://github.com/MarlinFirmware/Marlin/blob/bugfix-2.1.x/Marlin/src/module/temperature.cpp
- https://github.com/mriscoc/Special_Configurations/blob/main/_features/MPC.json

A substantial amount of code is based on the klipper and marlin source code. Therefore the
`src/` folder is licensed under GPL3.
