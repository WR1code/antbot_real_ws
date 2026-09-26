"""Fail-closed base command lease and final transport writer."""

from dataclasses import dataclass
import threading
import time


@dataclass(frozen=True)
class AuthorityToken:
    owner: str
    epoch: int


class CommandAuthority:
    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._token = None
        self._deadline = 0.0
        self._next_epoch = 1

    def _expire(self, now):
        if self._token is not None and now >= self._deadline:
            self._token = None
            self._deadline = 0.0

    def acquire(self, owner, *, now=None, lease_seconds):
        now = time.monotonic() if now is None else float(now)
        if not owner or lease_seconds <= 0.0:
            raise ValueError("owner and positive lease required")
        with self._lock:
            self._expire(now)
            if self._token is not None:
                return None
            token = AuthorityToken(str(owner), self._next_epoch)
            self._next_epoch += 1
            self._token = token
            self._deadline = now + float(lease_seconds)
            return token

    def renew(self, token, *, now=None, lease_seconds):
        now = time.monotonic() if now is None else float(now)
        with self._lock:
            self._expire(now)
            if token is None or token != self._token or lease_seconds <= 0.0:
                return False
            self._deadline = now + float(lease_seconds)
            return True

    def validate(self, token, *, now=None):
        now = time.monotonic() if now is None else float(now)
        with self._lock:
            self._expire(now)
            return token is not None and token == self._token

    def token_for(self, owner, *, now=None):
        now = time.monotonic() if now is None else float(now)
        with self._lock:
            self._expire(now)
            if self._token is not None and self._token.owner == owner:
                return self._token
            return None

    def release(self, token):
        with self._lock:
            if token is None or token != self._token:
                return False
            self._token = None
            self._deadline = 0.0
            return True

    def revoke(self, _reason="revoked"):
        with self._lock:
            self._token = None
            self._deadline = 0.0
            self._next_epoch += 1

    def owner(self, *, now=None):
        now = time.monotonic() if now is None else float(now)
        with self._lock:
            self._expire(now)
            return None if self._token is None else self._token.owner


class AuthoritativeWriter:
    def __init__(self, authority, transport):
        self._authority = authority
        self._transport = transport

    def write(self, token, command, *, now=None):
        if not self._authority.validate(token, now=now):
            return False
        result = self._transport(command)
        return result is not False
