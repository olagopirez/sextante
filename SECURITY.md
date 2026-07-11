# Security Policy

## Supported versions

| Version | Supported |
|---------|-----------|
| latest `master` | ✅ |
| older releases | ❌ |

## Reporting a vulnerability

Please **do not** open a public issue for security problems.

Report them privately through [GitHub Security Advisories](https://github.com/olagopirez/sextante/security/advisories/new) ("Report a vulnerability"). You should get an acknowledgement within a week.

## Scope notes

sextante is a hardware driver that talks to a local I2C bus; it opens no network sockets and parses no untrusted input by itself. Reports are nonetheless welcome for anything that could make a host application misbehave — e.g. unbounded resource usage in the sampling threads, or unsafe handling of values read from the bus.
