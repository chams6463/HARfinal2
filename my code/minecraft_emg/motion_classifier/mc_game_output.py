#!/usr/bin/env python3
"""50 ms H10 판정 dict를 Minecraft RCON·DirectInput 출력으로 변환한다."""
from __future__ import annotations

import atexit
import json
import math
from pathlib import Path
import re
import signal
import sys
import threading
import time
from typing import Any, Callable, Mapping


DEFAULT_TUNING: dict[str, Any] = {
    "walk_speed_levels": [0.06, 0.10, 0.14, 0.19],
    "attack_ms": 60.0,
    "release_ms": 180.0,
    "jump_mode": "attribute",
    "jump_power_levels": [0.42, 0.52, 0.62],
    "jump_boost_levels": [0, 1, 2],
    "jump_refractory_ms": 400.0,
    "jump_key_hold_ms": 80.0,
    "turn_mode": "mouse",
    "turn_deadzone_dps": 8.0,
    "turn_gain": 3.0,
    "turn_max_dps": 180.0,
    "turn_invert": False,
    "turn_sync_enable": False,
    "turn_sync_th_deg": 12.0,
    "turn_sync_gain": 0.25,
    "arm_click_th": 0.15,
    "arm_hold_th": 0.40,
    "arm_hold_release_th": 0.35,
    "arm_click_refractory_ms": 300.0,
    "arm_click_max_burst_ms": 800.0,
    "min_confidence": 0.5,
    "base_speed": 0.1,
    "base_jump": 0.42,
    "tx_deadband": 0.005,
    "tx_min_interval_s": 0.05,
}

WATCHDOG_TIMEOUT_S = 0.200
RCON_RETRY_S = 2.0
TUNING_RELOAD_S = 0.5
TURN_SYNC_PERIOD_S = 2.0
_PLAYER_RE = re.compile(r"^[A-Za-z0-9_]{1,16}$")
_YAW_RE = re.compile(r"\[\s*(-?(?:\d+(?:\.\d*)?|\.\d+))(?:[fFdD])?")


class GameOutputError(RuntimeError):
    """Minecraft 출력 모듈 설정 또는 의존성 오류."""


class DirectInputBackend:
    """pydirectinput을 시험 가능한 작은 인터페이스로 감싼다."""

    def __init__(self) -> None:
        try:
            import pydirectinput
        except ModuleNotFoundError as exc:
            raise GameOutputError(
                "pydirectinput이 없습니다: python -m pip install pydirectinput"
            ) from exc
        pydirectinput.PAUSE = 0.0
        pydirectinput.FAILSAFE = False
        self.api = pydirectinput

    def key_down(self, key: str) -> None:
        self.api.keyDown(key)

    def key_up(self, key: str) -> None:
        self.api.keyUp(key)

    def press(self, key: str) -> None:
        self.api.press(key)

    def move_rel(self, x: int, y: int) -> None:
        self.api.moveRel(x, y, relative=True)

    def mouse_down(self, button: str) -> None:
        self.api.mouseDown(button=button)

    def mouse_up(self, button: str) -> None:
        self.api.mouseUp(button=button)

    def click(self, button: str) -> None:
        self.api.click(button=button)


def _default_rcon_factory(host: str, password: str, port: int):
    try:
        from mcrcon import MCRcon
    except ModuleNotFoundError as exc:
        raise GameOutputError("mcrcon이 없습니다: python -m pip install mcrcon") from exc
    return MCRcon(host, password, port=port)


def _as_number(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name}은 숫자여야 합니다.")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name}은 유한한 숫자여야 합니다.")
    return result


def _number_list(value: Any, name: str, length: int) -> list[float]:
    if not isinstance(value, list) or len(value) != length:
        raise ValueError(f"{name}은 길이 {length}의 배열이어야 합니다.")
    return [_as_number(item, f"{name}[{index}]") for index, item in enumerate(value)]


def validate_tuning(candidate: Mapping[str, Any]) -> dict[str, Any]:
    """부분 JSON을 기본값과 합치고 모든 안전 관련 계수를 검증한다."""
    merged = dict(DEFAULT_TUNING)
    merged.update({key: value for key, value in candidate.items() if key in merged})

    merged["walk_speed_levels"] = _number_list(
        merged["walk_speed_levels"], "walk_speed_levels", 4
    )
    merged["jump_power_levels"] = _number_list(
        merged["jump_power_levels"], "jump_power_levels", 3
    )
    boost = _number_list(merged["jump_boost_levels"], "jump_boost_levels", 3)
    if any(not value.is_integer() or value < 0 for value in boost):
        raise ValueError("jump_boost_levels는 0 이상의 정수여야 합니다.")
    merged["jump_boost_levels"] = [int(value) for value in boost]

    numeric_keys = (
        "attack_ms",
        "release_ms",
        "jump_refractory_ms",
        "jump_key_hold_ms",
        "turn_deadzone_dps",
        "turn_gain",
        "turn_max_dps",
        "turn_sync_th_deg",
        "turn_sync_gain",
        "arm_click_th",
        "arm_hold_th",
        "arm_hold_release_th",
        "arm_click_refractory_ms",
        "arm_click_max_burst_ms",
        "min_confidence",
        "base_speed",
        "base_jump",
        "tx_deadband",
        "tx_min_interval_s",
    )
    for key in numeric_keys:
        merged[key] = _as_number(merged[key], key)

    if merged["jump_mode"] not in ("attribute", "effect"):
        raise ValueError("jump_mode는 attribute 또는 effect여야 합니다.")
    if merged["turn_mode"] not in ("mouse", "command"):
        raise ValueError("turn_mode는 mouse 또는 command여야 합니다.")
    for key in ("turn_invert", "turn_sync_enable"):
        if not isinstance(merged[key], bool):
            raise ValueError(f"{key}는 true/false여야 합니다.")
    if merged["attack_ms"] <= 0 or merged["release_ms"] <= 0:
        raise ValueError("attack_ms와 release_ms는 양수여야 합니다.")
    for key in (
        "jump_refractory_ms",
        "jump_key_hold_ms",
        "turn_deadzone_dps",
        "turn_gain",
        "turn_max_dps",
        "turn_sync_th_deg",
        "arm_click_th",
        "arm_click_refractory_ms",
        "arm_click_max_burst_ms",
        "tx_deadband",
        "tx_min_interval_s",
    ):
        if merged[key] < 0:
            raise ValueError(f"{key}는 음수일 수 없습니다.")
    if not 0.0 <= merged["turn_sync_gain"] <= 1.0:
        raise ValueError("turn_sync_gain은 0~1이어야 합니다.")
    if not 0.0 <= merged["min_confidence"] <= 1.0:
        raise ValueError("min_confidence는 0~1이어야 합니다.")
    if merged["base_speed"] <= 0 or merged["base_jump"] <= 0:
        raise ValueError("base_speed와 base_jump는 양수여야 합니다.")
    if not (
        merged["arm_click_th"] <= merged["arm_hold_release_th"] <= merged["arm_hold_th"]
    ):
        raise ValueError("arm_click_th <= arm_hold_release_th <= arm_hold_th여야 합니다.")
    if any(value <= 0 for value in merged["walk_speed_levels"]):
        raise ValueError("walk_speed_levels는 모두 양수여야 합니다.")
    if any(value <= 0 for value in merged["jump_power_levels"]):
        raise ValueError("jump_power_levels는 모두 양수여야 합니다.")
    return merged


class TuningStore:
    def __init__(
        self,
        path: Path,
        warn_once: Callable[[str, str], None],
    ) -> None:
        self.path = path
        self.warn_once = warn_once
        self.config = validate_tuning({})
        self.mtime_ns: int | None = None
        self.last_check = -math.inf
        self.maybe_reload(0.0, force=True)

    def maybe_reload(self, now: float, *, force: bool = False) -> bool:
        if not force and now - self.last_check < TUNING_RELOAD_S:
            return False
        self.last_check = now
        try:
            stat = self.path.stat()
        except OSError as exc:
            self.warn_once("tuning_missing", f"튜닝 파일을 읽을 수 없어 기본값 유지: {exc}")
            return False
        if not force and self.mtime_ns == stat.st_mtime_ns:
            return False
        try:
            payload = json.loads(self.path.read_text(encoding="utf-8"))
            if not isinstance(payload, dict):
                raise ValueError("최상위 JSON은 object여야 합니다.")
            config = validate_tuning(payload)
        except (OSError, UnicodeError, json.JSONDecodeError, ValueError) as exc:
            self.warn_once(
                f"tuning_invalid:{stat.st_mtime_ns}",
                f"튜닝 파일 오류로 이전 값 유지: {exc}",
            )
            return False
        self.config = config
        self.mtime_ns = stat.st_mtime_ns
        return True


def _clamp(value: float, low: float, high: float) -> float:
    return min(max(value, low), high)


def _piecewise(value: float, anchors: list[float]) -> float:
    position = _clamp(value, 0.0, 1.0) * (len(anchors) - 1)
    left = min(int(math.floor(position)), len(anchors) - 2)
    fraction = position - left
    return anchors[left] + fraction * (anchors[left + 1] - anchors[left])


def _wrap_degrees(value: float) -> float:
    return (value + 180.0) % 360.0 - 180.0


class GameOutput:
    """판정 dict 하나를 받을 때마다 Minecraft 입력 상태를 갱신한다."""

    def __init__(
        self,
        *,
        player: str,
        rcon_host: str,
        rcon_port: int,
        rcon_password: str,
        tuning_path: str | Path,
        direct_input: Any | None = None,
        rcon_factory: Callable[[str, str, int], Any] | None = None,
        clock: Callable[[], float] = time.monotonic,
        start_watchdog: bool = True,
        install_handlers: bool = True,
        console: bool = True,
    ) -> None:
        if not _PLAYER_RE.fullmatch(player):
            raise GameOutputError("player는 1~16자의 Minecraft 사용자명이어야 합니다.")
        if not isinstance(rcon_port, int) or not 1 <= rcon_port <= 65535:
            raise GameOutputError("rcon_port 범위가 잘못됐습니다.")
        if not rcon_host or not isinstance(rcon_host, str):
            raise GameOutputError("rcon_host가 비어 있습니다.")
        if not isinstance(rcon_password, str):
            raise GameOutputError("rcon_password는 문자열이어야 합니다.")

        self.player = player
        self.rcon_host = rcon_host
        self.rcon_port = rcon_port
        self.rcon_password = rcon_password
        self.direct = direct_input or DirectInputBackend()
        self.rcon_factory = rcon_factory or _default_rcon_factory
        self.clock = clock
        self.console = console
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.closed = False
        self.warned: set[str] = set()
        self.tuning = TuningStore(Path(tuning_path), self._warn_once)

        now = self.clock()
        self.last_update_at = now
        self.last_decision_at: float | None = None
        self.stale = False
        self.rcon = None
        self.last_rcon_attempt = -math.inf
        self.tx_values: dict[str, float] = {}
        self.tx_times: dict[str, float] = {}

        self.state = "stand"
        self.jump_forward = False
        self.last_walk_intensity = 0.0
        self.walk_speed = float(self.tuning.config["base_speed"])
        self.jump_refractory_until = -math.inf
        self.held_keys: set[str] = set()
        self.held_buttons: set[str] = set()

        self.mouse_accum = 0.0
        self.turn_command_accum = 0.0
        self.last_turn_command = -math.inf
        self.last_turn_sync = -math.inf
        self.last_mouse_step = 0

        self.arm_burst_active = False
        self.arm_burst_started = 0.0
        self.arm_burst_peak = 0.0
        self.arm_hold_triggered = False
        self.arm_refractory_until = -math.inf

        self.watchdog_thread: threading.Thread | None = None
        if start_watchdog:
            self.watchdog_thread = threading.Thread(
                target=self._watchdog_loop,
                name="minecraft-output-watchdog",
                daemon=True,
            )
            self.watchdog_thread.start()
        if install_handlers:
            self._install_cleanup_handlers()

    def _warn_once(self, code: str, message: str) -> None:
        if code in self.warned:
            return
        self.warned.add(code)
        print(f"\n[WARN] {message}", file=sys.stderr, flush=True)

    def _install_cleanup_handlers(self) -> None:
        atexit.register(self.close)

        def _term(_signum, _frame):
            raise SystemExit(0)

        for name in ("SIGTERM", "SIGBREAK", "SIGHUP"):
            sig = getattr(signal, name, None)
            if sig is None:
                continue
            try:
                signal.signal(sig, _term)
            except (ValueError, OSError):
                pass

    def _value(self, decision: Mapping[str, Any], key: str, default: float) -> float:
        if key not in decision or decision[key] is None:
            self._warn_once(f"missing:{key}", f"{key} 누락: 기본값 {default} 사용")
            return default
        value = decision[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            self._warn_once(f"type:{key}", f"{key} 타입 오류: 기본값 {default} 사용")
            return default
        result = float(value)
        if not math.isfinite(result):
            self._warn_once(f"finite:{key}", f"{key} 비유한 값: 기본값 {default} 사용")
            return default
        return result

    def _state_value(self, decision: Mapping[str, Any]) -> str:
        value = decision.get("state")
        if not isinstance(value, str):
            self._warn_once("state_type", "state 누락 또는 타입 오류: stand 사용")
            return "stand"
        normalized = "".join(value.lower().split()).replace("-", "_")
        aliases = {
            "walk": "walk",
            "walking": "walk",
            "jump": "jump",
            "crouch": "crouch",
            "sit": "crouch",
            "stand": "stand",
            "stand_up": "stand",
            "standup": "stand",
        }
        if normalized not in aliases:
            self._warn_once(
                f"state_unknown:{normalized}", f"알 수 없는 state={value!r}: stand 사용"
            )
            return "stand"
        return aliases[normalized]

    def _turn_rate(
        self, decision: Mapping[str, Any], config: Mapping[str, Any]
    ) -> float:
        raw_rate = self._value(decision, "turn_rate_dps", 0.0)
        if "turn_direction" not in decision or decision["turn_direction"] is None:
            self._warn_once("missing:turn_direction", "turn_direction 누락: rate 부호 사용")
            direction_value = "none"
        else:
            direction_value = decision["turn_direction"]
        if not isinstance(direction_value, str):
            self._warn_once("turn_direction_type", "turn_direction 타입 오류: none 사용")
            direction = "none"
        else:
            direction = direction_value.strip().lower().replace(" ", "")
        clockwise = {"cw", "right", "clockwise"}
        counterclockwise = {"ccw", "left", "counterclockwise"}
        if direction in clockwise:
            rate = abs(raw_rate)
        elif direction in counterclockwise:
            rate = -abs(raw_rate)
        elif direction in ("", "none", "stop", "0"):
            rate = raw_rate
        else:
            self._warn_once(
                f"turn_direction_unknown:{direction}",
                f"알 수 없는 turn_direction={direction_value!r}: rate 부호 사용",
            )
            rate = raw_rate
        if abs(rate) < float(config["turn_deadzone_dps"]):
            rate = 0.0
        rate = _clamp(
            rate,
            -float(config["turn_max_dps"]),
            float(config["turn_max_dps"]),
        )
        return -rate if config["turn_invert"] else rate

    def _set_key(self, key: str, pressed: bool) -> None:
        if pressed and key not in self.held_keys:
            self.direct.key_down(key)
            self.held_keys.add(key)
        elif not pressed and key in self.held_keys:
            self.direct.key_up(key)
            self.held_keys.remove(key)

    def _set_button(self, button: str, pressed: bool) -> None:
        if pressed and button not in self.held_buttons:
            self.direct.mouse_down(button)
            self.held_buttons.add(button)
        elif not pressed and button in self.held_buttons:
            self.direct.mouse_up(button)
            self.held_buttons.remove(button)

    def _release_local_inputs(self, *, force_all: bool) -> None:
        keys = ("w", "shift", "space") if force_all else tuple(self.held_keys)
        buttons = ("left", "right") if force_all else tuple(self.held_buttons)
        for key in keys:
            try:
                self.direct.key_up(key)
            except Exception as exc:
                self._warn_once(f"key_up:{key}", f"{key} keyUp 실패: {exc}")
        for button in buttons:
            try:
                self.direct.mouse_up(button)
            except Exception as exc:
                self._warn_once(f"mouse_up:{button}", f"{button} mouseUp 실패: {exc}")
        self.held_keys.clear()
        self.held_buttons.clear()

    def _mark_rcon_down(self, exc: Exception) -> None:
        self._warn_once(f"rcon_down:{type(exc).__name__}", f"RCON 끊김: {exc}")
        client = self.rcon
        self.rcon = None
        self.tx_values.clear()
        self.tx_times.clear()
        if client is not None:
            try:
                client.disconnect()
            except Exception:
                pass
        self._release_local_inputs(force_all=True)

    def _rcon_command(self, command: str, purpose: str) -> str | None:
        if self.rcon is None:
            return None
        try:
            response = self.rcon.command(command)
        except Exception as exc:
            self._mark_rcon_down(exc)
            return None
        text = "" if response is None else str(response)
        lowered = text.lower()
        if any(
            marker in lowered
            for marker in (
                "unknown",
                "incorrect argument",
                "no entity was found",
                "error",
            )
        ):
            self._warn_once(f"rcon_response:{purpose}:{text}", f"RCON 응답 확인 필요: {text}")
        return text

    def _ensure_rcon(self, now: float) -> bool:
        if self.rcon is not None:
            return True
        if now - self.last_rcon_attempt < RCON_RETRY_S:
            return False
        self.last_rcon_attempt = now
        try:
            client = self.rcon_factory(
                self.rcon_host, self.rcon_password, self.rcon_port
            )
            client.connect()
            self.rcon = client
            self.tx_values.clear()
            self.tx_times.clear()
            config = self.tuning.config
            self._rcon_command(
                f"attribute {self.player} minecraft:movement_speed base set "
                f"{float(config['base_speed']):.6f}",
                "connect_speed_reset",
            )
            self._rcon_command(
                f"attribute {self.player} minecraft:jump_strength base set "
                f"{float(config['base_jump']):.6f}",
                "connect_jump_reset",
            )
            self._rcon_command(
                f"effect clear {self.player} minecraft:jump_boost",
                "connect_effect_clear",
            )
        except Exception as exc:
            self.rcon = None
            self._warn_once(
                f"rcon_connect:{type(exc).__name__}",
                f"RCON 연결 실패, 2초 뒤 재시도: {exc}",
            )
            self._release_local_inputs(force_all=True)
            return False
        return self.rcon is not None

    def _send_numeric(
        self,
        key: str,
        value: float,
        command: str,
        now: float,
        *,
        force: bool = False,
    ) -> bool:
        config = self.tuning.config
        previous = self.tx_values.get(key)
        last_time = self.tx_times.get(key, -math.inf)
        if not force:
            if now - last_time < float(config["tx_min_interval_s"]):
                return False
            if (
                previous is not None
                and abs(value - previous) < float(config["tx_deadband"])
                and now - last_time < 1.0
            ):
                return False
        if self._rcon_command(command, key) is None and self.rcon is None:
            return False
        self.tx_values[key] = value
        self.tx_times[key] = now
        return True

    def _send_speed(self, value: float, now: float, *, force: bool = False) -> bool:
        return self._send_numeric(
            "movement_speed",
            value,
            f"attribute {self.player} minecraft:movement_speed base set {value:.6f}",
            now,
            force=force,
        )

    def _set_jump_power(self, intensity: float, now: float) -> bool:
        config = self.tuning.config
        if config["jump_mode"] == "attribute":
            value = _piecewise(intensity, config["jump_power_levels"])
            return self._send_numeric(
                "jump_power",
                value,
                f"attribute {self.player} minecraft:jump_strength base set {value:.6f}",
                now,
                force=True,
            )
        continuous = _piecewise(
            intensity, [float(value) for value in config["jump_boost_levels"]]
        )
        level = int(math.floor(continuous + 0.5))
        return self._send_numeric(
            "jump_boost",
            float(level),
            f"effect give {self.player} minecraft:jump_boost 1 {level} true",
            now,
            force=True,
        )

    def _press_space_for(self, hold_ms: float) -> None:
        """Space를 지정 시간 동안 실제 keyDown 상태로 유지한 뒤 해제한다."""
        hold_s = max(float(hold_ms), 1.0) / 1000.0
        self.direct.key_down("space")
        try:
            time.sleep(hold_s)
        finally:
            self.direct.key_up("space")

    def _enter_state(
        self,
        new_state: str,
        decision: Mapping[str, Any],
        now: float,
    ) -> bool:
        prior = self.state
        self.state = new_state
        if new_state == "jump":
            # 점프는 항상 앞으로 가도록 고정한다.
            self.jump_forward = True

            # Space보다 먼저 W를 확실히 누른 상태로 만든다.
            # 실제 입력은 "W hold + Space press"가 된다.
            self._set_key("shift", False)
            self._set_key("w", True)

            # 400 ms refractory 제한은 다시 적용한다.
            if now >= self.jump_refractory_until:
                intensity = _clamp(
                    self._value(decision, "jump_intensity", 0.0), 0.0, 1.0
                )
                if self._set_jump_power(intensity, now) and self.rcon is not None:
                    self._press_space_for(
                        float(self.tuning.config["jump_key_hold_ms"])
                    )
                    self.jump_refractory_until = now + (
                        float(self.tuning.config["jump_refractory_ms"]) / 1000.0
                    )
                    return True
        elif prior == "jump":
            self.jump_forward = False
        return False

    def _apply_motion(
        self,
        decision: Mapping[str, Any],
        dt: float,
        now: float,
        *,
        state_changed: bool,
    ) -> None:
        config = self.tuning.config
        base_speed = float(config["base_speed"])
        if self.state == "walk":
            intensity = _clamp(self._value(decision, "walk_intensity", 0.0), 0.0, 1.0)
            self.last_walk_intensity = intensity
            target = _piecewise(intensity, config["walk_speed_levels"])
            tau_ms = (
                float(config["attack_ms"])
                if target > self.walk_speed
                else float(config["release_ms"])
            )
            alpha = 1.0 - math.exp(-dt / (tau_ms / 1000.0))
            self.walk_speed += alpha * (target - self.walk_speed)
            if self._send_speed(self.walk_speed, now) or self.rcon is not None:
                self._set_key("shift", False)
                self._set_key("w", True)
            return

        if self.state == "jump" and self.jump_forward:
            if self._send_speed(self.walk_speed, now) or self.rcon is not None:
                self._set_key("shift", False)
                self._set_key("w", True)
            return

        # walk가 끝나는 순간 release smoothing을 무시하고 즉시 원복한다.
        self.walk_speed = base_speed
        self._send_speed(base_speed, now, force=state_changed)
        self._set_key("w", False)
        self._set_key("shift", self.state == "crouch")

    def _move_mouse_pixels(self, pixels: float) -> int:
        self.mouse_accum += pixels
        step = int(self.mouse_accum)
        self.mouse_accum -= step
        if step:
            self.direct.move_rel(step, 0)
        return step

    def _apply_turn(self, rate: float, dt: float, now: float) -> None:
        config = self.tuning.config
        self.last_mouse_step = 0
        if config["turn_mode"] == "mouse":
            self.last_mouse_step = self._move_mouse_pixels(
                rate * float(config["turn_gain"]) * dt
            )
            return

        self.turn_command_accum += rate * dt
        if now - self.last_turn_command < float(config["tx_min_interval_s"]):
            return
        heartbeat = now - self.last_turn_command >= 1.0
        if (
            abs(self.turn_command_accum) < float(config["tx_deadband"])
            and not heartbeat
        ):
            return
        step = self.turn_command_accum
        response = self._rcon_command(
            f"rotate {self.player} ~{step:.6f} ~0", "turn_command"
        )
        if response is not None or self.rcon is not None:
            self.turn_command_accum = 0.0
            self.last_turn_command = now

    def _apply_turn_sync(self, target_angle: float, now: float) -> None:
        config = self.tuning.config
        if (
            not config["turn_sync_enable"]
            or now - self.last_turn_sync < TURN_SYNC_PERIOD_S
        ):
            return
        self.last_turn_sync = now
        response = self._rcon_command(
            f"data get entity {self.player} Rotation", "turn_sync_query"
        )
        if not response:
            return
        match = _YAW_RE.search(response)
        if not match:
            self._warn_once("turn_sync_parse", f"Minecraft yaw 파싱 실패: {response}")
            return
        game_yaw = float(match.group(1))
        error = _wrap_degrees(target_angle - game_yaw)
        if abs(error) <= float(config["turn_sync_th_deg"]):
            return
        correction = error * float(config["turn_sync_gain"])
        if config["turn_invert"]:
            correction = -correction
        if config["turn_mode"] == "mouse":
            self.last_mouse_step += self._move_mouse_pixels(
                correction * float(config["turn_gain"])
            )
        else:
            self.turn_command_accum += correction

    def _reset_arm_burst(self) -> None:
        self.arm_burst_active = False
        self.arm_burst_started = 0.0
        self.arm_burst_peak = 0.0
        self.arm_hold_triggered = False

    def _apply_arm(self, arm_emg: float, now: float) -> None:
        config = self.tuning.config
        click_th = float(config["arm_click_th"])
        hold_th = float(config["arm_hold_th"])
        release_th = float(config["arm_hold_release_th"])

        if not self.arm_burst_active:
            if now >= self.arm_refractory_until and arm_emg >= click_th:
                self.arm_burst_active = True
                self.arm_burst_started = now
                self.arm_burst_peak = arm_emg
                self.arm_hold_triggered = False
            else:
                return

        self.arm_burst_peak = max(self.arm_burst_peak, arm_emg)
        if arm_emg >= hold_th and not self.arm_hold_triggered:
            self.arm_hold_triggered = True
            self._set_button("left", True)
        elif "left" in self.held_buttons and arm_emg < release_th:
            self._set_button("left", False)

        if arm_emg >= click_th:
            return
        if "left" in self.held_buttons:
            self._set_button("left", False)
        duration_ms = (now - self.arm_burst_started) * 1000.0
        if (
            not self.arm_hold_triggered
            and self.arm_burst_peak < hold_th
            and duration_ms <= float(config["arm_click_max_burst_ms"])
        ):
            self.direct.click("right")
        self.arm_refractory_until = now + (
            float(config["arm_click_refractory_ms"]) / 1000.0
        )
        self._reset_arm_burst()

    def _safe_stop(self, now: float, *, restore_remote: bool) -> None:
        self._release_local_inputs(force_all=True)
        self.state = "stand"
        self.jump_forward = False
        self._reset_arm_burst()
        self.mouse_accum = 0.0
        self.turn_command_accum = 0.0
        config = self.tuning.config
        self.walk_speed = float(config["base_speed"])
        if restore_remote and self.rcon is not None:
            self._send_speed(self.walk_speed, now, force=True)
            self._rcon_command(
                f"attribute {self.player} minecraft:jump_strength base set "
                f"{float(config['base_jump']):.6f}",
                "safe_jump_reset",
            )
            self._rcon_command(
                f"effect clear {self.player} minecraft:jump_boost",
                "safe_effect_clear",
            )

    def _display(
        self,
        confidence: float,
        turn_rate: float,
        arm_emg: float,
        jump_fired: bool,
    ) -> None:
        if not self.console:
            return
        stage = int(round(self.last_walk_intensity * 3.0)) + 1
        text = (
            f"{self.state}[{stage if self.state == 'walk' else '-'}] "
            f"spd={self.walk_speed:.3f} "
            f"W={'O' if 'w' in self.held_keys else '.'} "
            f"turn={turn_rate:+.0f}dps mouse={self.last_mouse_step:+d} "
            f"jump={'O' if jump_fired else '.'} arm={arm_emg:.2f} "
            f"LMB={'O' if 'left' in self.held_buttons else '.'} "
            f"conf={confidence:.2f}"
        )
        print(f"\r{text:<110}", end="", file=sys.stderr, flush=True)

    def update(self, decision: Mapping[str, Any] | None) -> None:
        """50 ms 판정 하나를 반영한다. 잘못된 필드는 기본값으로 대체한다."""
        now = self.clock()
        with self.lock:
            if self.closed:
                return
            self.last_update_at = now
            self.stale = False
            self.tuning.maybe_reload(now)

            if not isinstance(decision, Mapping):
                self._warn_once("decision_type", "decision이 dict가 아니어서 안전 정지")
                self._safe_stop(now, restore_remote=True)
                return
            if decision.get("valid") is False:
                self._safe_stop(now, restore_remote=True)
                return
            if not self._ensure_rcon(now):
                self._safe_stop(now, restore_remote=False)
                return

            dt = (
                0.050
                if self.last_decision_at is None
                else _clamp(now - self.last_decision_at, 0.0, WATCHDOG_TIMEOUT_S)
            )
            self.last_decision_at = now
            config = self.tuning.config
            confidence = _clamp(self._value(decision, "confidence", 0.0), 0.0, 1.0)
            new_state = self._state_value(decision)
            jump_fired = False
            prior_state = self.state
            jump_onset_accepted = (
                new_state == "jump" and decision.get("jump_started") is True
            )
            if (
                confidence >= float(config["min_confidence"])
                or jump_onset_accepted
            ):
                jump_fired = self._enter_state(new_state, decision, now)

            self._apply_motion(
                decision,
                dt,
                now,
                state_changed=self.state != prior_state,
            )
            if self.rcon is None:
                self._safe_stop(now, restore_remote=False)
                return
            turn_rate = self._turn_rate(decision, config)
            self._apply_turn(turn_rate, dt, now)
            if config["turn_sync_enable"]:
                turn_angle = self._value(decision, "turn_angle_deg", 0.0)
                self._apply_turn_sync(turn_angle, now)
            arm_emg = max(self._value(decision, "arm_emg", 0.0), 0.0)
            self._apply_arm(arm_emg, now)
            self._display(confidence, turn_rate, arm_emg, jump_fired)

    def _watchdog_check(self, now: float | None = None) -> bool:
        current = self.clock() if now is None else now
        with self.lock:
            if self.closed or self.stale:
                return False
            if current - self.last_update_at < WATCHDOG_TIMEOUT_S:
                return False
            self.stale = True
            self._safe_stop(current, restore_remote=True)
            self._warn_once("watchdog", "판정 dict가 200 ms 이상 없어 안전 정지")
            return True

    def _watchdog_loop(self) -> None:
        while not self.stop_event.wait(0.05):
            self._watchdog_check()

    def close(self) -> None:
        with self.lock:
            if self.closed:
                return
            self.closed = True
            self.stop_event.set()
            now = self.clock()
            self._safe_stop(now, restore_remote=True)
            client = self.rcon
            self.rcon = None
            if client is not None:
                try:
                    client.disconnect()
                except Exception as exc:
                    self._warn_once("rcon_close", f"RCON 종료 실패: {exc}")
        if (
            self.watchdog_thread is not None
            and self.watchdog_thread is not threading.current_thread()
        ):
            self.watchdog_thread.join(timeout=0.5)
        if self.console:
            print(file=sys.stderr)
        try:
            atexit.unregister(self.close)
        except Exception:
            pass

    def __enter__(self) -> "GameOutput":
        return self

    def __exit__(self, _exc_type, _exc, _traceback) -> None:
        self.close()


if hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError, OSError):
        pass
