from __future__ import annotations

import shutil
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

import rclpy
from rclpy.node import Node


VALID_ACTIONS = {"connect", "disconnect", "status"}


class CommandError(RuntimeError):
    def __init__(self, command: Sequence[str], result: subprocess.CompletedProcess[str]):
        self.command = command
        self.result = result
        stderr = result.stderr.strip()
        stdout = result.stdout.strip()
        detail = stderr or stdout or f"exit code {result.returncode}"
        super().__init__(f"{' '.join(command)} failed: {detail}")


@dataclass(frozen=True)
class StaConfig:
    action: str
    ssid: str
    interface: str
    connection_name: str
    ipv4_method: str
    ip_cidr: str
    gateway: str
    rescan: bool


def run_command(command: Sequence[str], *, check: bool = True) -> subprocess.CompletedProcess[str]:
    result = subprocess.run(
        list(command),
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    if check and result.returncode != 0:
        raise CommandError(command, result)
    return result


def clean_lines(output: str) -> list[str]:
    return [line.strip() for line in output.splitlines() if line.strip()]


class StaManagerNode(Node):
    """连接车端 wifi_ap_manager 开出的开放热点(STA/客户端模式)。

    与车端 wifi_ap_manager 对称:车开 AP(192.168.50.1/24 shared),
    飞车在此连入,默认拿静态 IP 192.168.50.2,便于 pose UDP 上送与 NoMachine 远程观察。
    """

    def __init__(self) -> None:
        super().__init__("wifi_sta_manager")
        self.declare_parameter("action", "connect")
        self.declare_parameter("ssid", "OPi_ROS2_TEST")
        self.declare_parameter("interface", "wlan0")
        self.declare_parameter("connection_name", "OPi_ROS2_JOIN_AP")
        # manual: 用 ip_cidr/gateway 配静态地址; auto: 由车端 shared 模式的 DHCP 自动分配
        self.declare_parameter("ipv4_method", "manual")
        self.declare_parameter("ip_cidr", "192.168.50.2/24")
        self.declare_parameter("gateway", "192.168.50.1")
        self.declare_parameter("rescan", True)

    def execute(self) -> None:
        config = self.read_config()
        if config.action == "connect":
            self.connect(config)
        elif config.action == "disconnect":
            self.disconnect(config)
        elif config.action == "status":
            self.show_status(config)
        else:
            actions = ", ".join(sorted(VALID_ACTIONS))
            raise ValueError(f"Unsupported action '{config.action}'. Use one of: {actions}")

    def read_config(self) -> StaConfig:
        config = StaConfig(
            action=str(self.get_parameter("action").value).strip().lower(),
            ssid=str(self.get_parameter("ssid").value).strip(),
            interface=str(self.get_parameter("interface").value).strip(),
            connection_name=str(self.get_parameter("connection_name").value).strip(),
            ipv4_method=str(self.get_parameter("ipv4_method").value).strip().lower(),
            ip_cidr=str(self.get_parameter("ip_cidr").value).strip(),
            gateway=str(self.get_parameter("gateway").value).strip(),
            rescan=bool(self.get_parameter("rescan").value),
        )
        self.validate_config(config)
        return config

    def validate_config(self, config: StaConfig) -> None:
        if not config.ssid:
            raise ValueError("Parameter 'ssid' must not be empty")
        if not config.interface:
            raise ValueError("Parameter 'interface' must not be empty")
        if not config.connection_name:
            raise ValueError("Parameter 'connection_name' must not be empty")
        if config.ipv4_method not in {"manual", "auto"}:
            raise ValueError("Parameter 'ipv4_method' must be 'manual' (static) or 'auto' (DHCP)")
        if config.ipv4_method == "manual":
            if "/" not in config.ip_cidr:
                raise ValueError("Parameter 'ip_cidr' must include a prefix, for example 192.168.50.2/24")
            if not config.gateway:
                raise ValueError("Parameter 'gateway' must not be empty when ipv4_method is 'manual'")

    def connect(self, config: StaConfig) -> None:
        self.ensure_prerequisites(config)

        if config.rescan:
            self.get_logger().info(f"Rescanning Wi-Fi for SSID '{config.ssid}' on {config.interface}")
            run_command(["nmcli", "device", "wifi", "rescan", "ifname", config.interface], check=False)
            time.sleep(2.0)
        if not self.ssid_visible(config.ssid):
            self.get_logger().warn(
                f"SSID '{config.ssid}' not seen in scan yet; will still try to connect. "
                "Make sure the car ran wifi_ap_manager (action:=start) first."
            )

        if not self.connection_exists(config.connection_name):
            self.get_logger().info(f"Creating NetworkManager connection '{config.connection_name}'")
            run_command(
                [
                    "nmcli",
                    "connection",
                    "add",
                    "type",
                    "wifi",
                    "ifname",
                    config.interface,
                    "con-name",
                    config.connection_name,
                    "autoconnect",
                    "no",
                    "ssid",
                    config.ssid,
                ]
            )

        modify = [
            "nmcli",
            "connection",
            "modify",
            config.connection_name,
            "connection.interface-name",
            config.interface,
            "connection.autoconnect",
            "no",
            "802-11-wireless.mode",
            "infrastructure",
            "802-11-wireless.ssid",
            config.ssid,
            "ipv6.method",
            "disabled",
        ]
        if config.ipv4_method == "manual":
            self.get_logger().info(
                f"Joining open AP '{config.ssid}' with static IP {config.ip_cidr}, gateway {config.gateway}"
            )
            modify += [
                "ipv4.method",
                "manual",
                "ipv4.addresses",
                config.ip_cidr,
                "ipv4.gateway",
                config.gateway,
            ]
        else:
            self.get_logger().info(f"Joining open AP '{config.ssid}' with DHCP (ipv4.method=auto)")
            modify += [
                "ipv4.method",
                "auto",
                # 清掉可能残留的静态地址,避免 auto 模式下冲突
                "ipv4.addresses",
                "",
                "ipv4.gateway",
                "",
            ]
        run_command(modify)
        self.remove_wifi_security(config.connection_name)

        self.get_logger().warn(
            "Bringing up this connection will disconnect the interface from any other Wi-Fi network."
        )
        run_command(["nmcli", "connection", "up", config.connection_name])
        self.log_ip(config.interface)
        self.get_logger().info(
            f"Joined hotspot: SSID='{config.ssid}', interface={config.interface}, "
            f"method={config.ipv4_method}"
        )

    def disconnect(self, config: StaConfig) -> None:
        self.ensure_nmcli()
        if not self.connection_exists(config.connection_name):
            self.get_logger().info(f"Connection '{config.connection_name}' does not exist; nothing to disconnect")
            return
        if not self.connection_active(config.connection_name):
            self.get_logger().info(f"Connection '{config.connection_name}' is already inactive")
            return
        run_command(["nmcli", "connection", "down", config.connection_name])
        self.get_logger().info(f"Disconnected from hotspot connection '{config.connection_name}'")
        self.get_logger().info("NetworkManager may reconnect a saved Wi-Fi network automatically.")

    def show_status(self, config: StaConfig) -> None:
        self.ensure_nmcli()
        self.get_logger().info(f"nmcli: {shutil.which('nmcli')}")
        self.log_command_output(["nmcli", "-t", "-f", "DEVICE,TYPE,STATE,CONNECTION", "device", "status"])
        self.log_command_output(["nmcli", "-t", "-f", "NAME,TYPE,DEVICE", "connection", "show", "--active"])
        self.log_command_output(["ip", "-br", "addr", "show", config.interface], check=False)

        exists = self.connection_exists(config.connection_name)
        active = self.connection_active(config.connection_name) if exists else False
        visible = self.ssid_visible(config.ssid)
        self.get_logger().info(
            f"join_connection={config.connection_name} exists={exists} active={active} "
            f"ssid='{config.ssid}' visible={visible}"
        )

    def ensure_prerequisites(self, config: StaConfig) -> None:
        self.ensure_nmcli()
        if not Path("/sys/class/net", config.interface).exists():
            raise RuntimeError(f"Interface '{config.interface}' was not found")
        wifi_state = run_command(["nmcli", "radio", "wifi"]).stdout.strip().lower()
        if wifi_state != "enabled":
            raise RuntimeError("Wi-Fi radio is disabled. Enable it first with: nmcli radio wifi on")

    def ensure_nmcli(self) -> None:
        if shutil.which("nmcli") is None:
            raise RuntimeError("nmcli was not found. Install or enable NetworkManager first.")

    def ssid_visible(self, ssid: str) -> bool:
        result = run_command(["nmcli", "-t", "-f", "SSID", "device", "wifi", "list"], check=False)
        return ssid in clean_lines(result.stdout)

    def connection_exists(self, connection_name: str) -> bool:
        result = run_command(["nmcli", "-g", "NAME", "connection", "show"])
        return connection_name in clean_lines(result.stdout)

    def connection_active(self, connection_name: str) -> bool:
        result = run_command(["nmcli", "-g", "NAME", "connection", "show", "--active"])
        return connection_name in clean_lines(result.stdout)

    def remove_wifi_security(self, connection_name: str) -> None:
        command = [
            "nmcli",
            "connection",
            "modify",
            connection_name,
            "remove",
            "802-11-wireless-security",
        ]
        result = run_command(command, check=False)
        if result.returncode == 0:
            return
        message = f"{result.stdout}\n{result.stderr}".lower()
        if "not present" in message or "no such setting" in message:
            return
        raise CommandError(command, result)

    def log_ip(self, interface: str) -> None:
        result = run_command(["ip", "-br", "addr", "show", interface], check=False)
        output = result.stdout.strip() or result.stderr.strip() or "(no output)"
        self.get_logger().info(f"$ ip -br addr show {interface}\n{output}")

    def log_command_output(self, command: Iterable[str], *, check: bool = True) -> None:
        command_list = list(command)
        result = run_command(command_list, check=check)
        output = result.stdout.strip() or result.stderr.strip() or "(no output)"
        self.get_logger().info(f"$ {' '.join(command_list)}\n{output}")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = StaManagerNode()
    exit_code = 0
    try:
        node.execute()
    except Exception as exc:  # noqa: BLE001 - this is a command-line ROS helper.
        node.get_logger().error(str(exc))
        exit_code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if exit_code:
        raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
