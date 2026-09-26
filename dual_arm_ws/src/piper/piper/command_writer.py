"""Authoritative final wrapper around Piper vendor motion calls."""

from __future__ import annotations

from .command_authority import AuthorityToken, AuthoritativeWriter, CommandAuthority


class PiperCommandWriter:
    def __init__(self, sdk, authority: CommandAuthority) -> None:
        self._sdk = sdk
        self._writer = AuthoritativeWriter(authority, self._dispatch)
        self._pending = None

    def _dispatch(self, command) -> None:
        kind, values = command
        if kind == "joint":
            self._sdk.JointCtrl(*values)
        elif kind == "pose":
            self._sdk.EndPoseCtrl(*values)
        elif kind == "call":
            values()
        else:
            raise ValueError(f"unsupported Piper command kind: {kind}")

    def joints(self, token: AuthorityToken | None, values, *, now=None) -> bool:
        return self._writer.write(token, ("joint", tuple(values)), now=now)

    def pose(self, token: AuthorityToken | None, values, *, now=None) -> bool:
        return self._writer.write(token, ("pose", tuple(values)), now=now)

    def call(self, token: AuthorityToken | None, callback, *, now=None) -> bool:
        return self._writer.write(token, ("call", callback), now=now)

