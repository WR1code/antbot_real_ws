"""Small fail-closed lease gate used at motion transport boundaries."""

from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass
import threading
import time
from typing import Callable, Generic, TypeVar


@dataclass(frozen=True)
class AuthorityToken:
    owner: str
    epoch: int


class CommandAuthority:
    """Single-owner monotonic lease with epoch invalidation."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._token: AuthorityToken | None = None
        self._deadline = 0.0
        self._next_epoch = 1
        self._revoke_reason = "startup"

    def _expire(self, now: float) -> None:
        if self._token is not None and now >= self._deadline:
            self._token = None
            self._deadline = 0.0
            self._revoke_reason = "lease expired"

    def acquire(
        self, owner: str, *, now: float | None = None, lease_seconds: float
    ) -> AuthorityToken | None:
        now = time.monotonic() if now is None else float(now)
        if not owner or lease_seconds <= 0.0:
            raise ValueError("owner and a positive lease are required")
        with self._lock:
            self._expire(now)
            if self._token is not None:
                return None
            token = AuthorityToken(owner=owner, epoch=self._next_epoch)
            self._next_epoch += 1
            self._token = token
            self._deadline = now + float(lease_seconds)
            self._revoke_reason = ""
            return token

    def renew(
        self,
        token: AuthorityToken | None,
        *,
        now: float | None = None,
        lease_seconds: float,
    ) -> bool:
        now = time.monotonic() if now is None else float(now)
        if lease_seconds <= 0.0:
            raise ValueError("lease_seconds must be positive")
        with self._lock:
            self._expire(now)
            if token is None or token != self._token:
                return False
            self._deadline = now + float(lease_seconds)
            return True

    def validate(
        self, token: AuthorityToken | None, *, now: float | None = None
    ) -> bool:
        now = time.monotonic() if now is None else float(now)
        with self._lock:
            self._expire(now)
            return token is not None and token == self._token

    def token_for(
        self, owner: str, *, now: float | None = None
    ) -> AuthorityToken | None:
        now = time.monotonic() if now is None else float(now)
        with self._lock:
            self._expire(now)
            if self._token is not None and self._token.owner == owner:
                return self._token
            return None

    def release(self, token: AuthorityToken | None) -> bool:
        with self._lock:
            if token is None or token != self._token:
                return False
            self._token = None
            self._deadline = 0.0
            self._revoke_reason = "released"
            return True

    def revoke(self, reason: str) -> None:
        with self._lock:
            self._token = None
            self._deadline = 0.0
            self._next_epoch += 1
            self._revoke_reason = reason or "revoked"

    def owner(self, *, now: float | None = None) -> str | None:
        now = time.monotonic() if now is None else float(now)
        with self._lock:
            self._expire(now)
            return None if self._token is None else self._token.owner

    @contextmanager
    def hold(
        self,
        owner: str,
        *,
        now: Callable[[], float] = time.monotonic,
        lease_seconds: float,
    ):
        token = self.acquire(owner, now=now(), lease_seconds=lease_seconds)
        if token is None:
            raise RuntimeError("motion authority is owned by another source")
        try:
            yield token
        finally:
            self.release(token)


T = TypeVar("T")


class AuthoritativeWriter(Generic[T]):
    """Last check immediately before a transport callable is invoked."""

    def __init__(self, authority: CommandAuthority, transport: Callable[[T], None]):
        self._authority = authority
        self._transport = transport

    def write(
        self,
        token: AuthorityToken | None,
        command: T,
        *,
        now: float | None = None,
    ) -> bool:
        if not self._authority.validate(token, now=now):
            return False
        result = self._transport(command)
        return result is not False
