#!/usr/bin/env python

import getpass
import sys
import pexpect

from typing import Optional
from pathlib import Path

class Shell:
    sudo_password: Optional[str]
    path: Path

    def __init__(self) -> None:
        self.sudo_password = None
        self.path = Path.cwd()

    def klipper_path(self) -> Path:
        return Path.home().joinpath("klipper/")

    def source_path(self) -> Path:
        return self.path.joinpath("src")

    def klipper_extension_path(self) -> Path:
        return self.klipper_path().joinpath("klippy/extras/")

    def _get_sudo_password(self) -> str:
        if self.sudo_password is None:
            self.sudo_password = getpass.getpass(prompt=f"[sudo] password for {getpass.getuser()}: ")

        return self.sudo_password

    def execute(self, command: list[str]):
        args = []
        if len(command) > 1:
            args = command[1:]
        child = pexpect.spawn(command[0], args)

        if command[0] == 'sudo':
            # it will prompt something like: "[sudo] password for < generic_user >:"
            # you "expect" to receive a string containing keyword "password"
            index = child.expect(['password', pexpect.EOF, pexpect.TIMEOUT])
            # if it's found, send the password
            if index == 0:
                child.sendline(self._get_sudo_password())

        return child.read()

def ensure_klipper_is_installed(shell: Shell):
    if b"klipper.service" in shell.execute(["sudo", "systemctl", "list-units", "--full", "-all", "-t", "service", "--no-legend", "--no-pager"]):
        print("Klipper service found!")
    else:
        print("Klipper service not found, please install Klipper first")
        sys.exit(-1)

def list_missing_extensions(shell: Shell, extensions: list[Path]) -> list[Path]:
    extras_path: Path = shell.klipper_extension_path()
    result: list[Path] = []

    for extension in extensions:
        if not extras_path.joinpath(extension.name).exists():
            result.append(extension)

    return result

def install_extension(shell: Shell, extension: Path) -> None:
    # create a symbolic link from src/extension.py to klipper/klippy/extras/extension.py
    # so that updates in the repo automatically apply to klipper
    shell.klipper_extension_path().joinpath(extension.name).symlink_to(extension)

def remove_deleted_extensions(shell: Shell) -> None:
    # go through all extensions
    for file in [e for e in shell.klipper_extension_path().iterdir() if e.is_file()]:
        # check if the symbolic link is broken (exists() will return false):
        if file.is_symlink() and not file.exists():
            # remove that symbolic link
            file.unlink(missing_ok=True)

def restart_klipper(shell: Shell):
    print("Restarting Klipper...")
    shell.execute(["sudo", "systemctl", "restart", "klipper"])

if __name__ == "__main__":
    shell = Shell()
    ensure_klipper_is_installed(shell)

    extensions = [e for e in shell.source_path().iterdir() if e.is_file()]

    # install missing extensions
    for extension in list_missing_extensions(shell, extensions):
        install_extension(shell, extension)

    # remove old extensions that have been deleted
    remove_deleted_extensions(shell)

    # always restart klipper, to load the updated code
    restart_klipper(shell)

    sys.exit(0)
