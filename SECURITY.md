# Security Policy

## Supported Versions

| Version | Supported |
|---------|-----------|
| main    | Yes       |

Tapestry is pre-1.0 software. Security fixes are applied to `main` only;
there are no backport releases at this stage.

## Reporting a Vulnerability

**Please do not open a public GitHub issue for security vulnerabilities.**

Report security issues by emailing **jim@tapestry-os.com** with the subject
line `[tapestry-security]`. Include:

- A description of the vulnerability and its potential impact.
- Steps to reproduce or a proof-of-concept (if available).
- Any suggested mitigations you have identified.

You can expect an acknowledgement within 72 hours and a status update within
7 days. We will coordinate a disclosure timeline with you before publishing
any fix.

## Threat Model

Tapestry is a distributed OS framework for physically co-located element
swarms. The current implementation (L3–L5) is designed for trusted,
contained environments — lab benches, research deployments, and known
hardware. It does **not** implement:

- Encrypted gossip transport (UDP broadcast is plaintext).
- Authentication of gossip messages (any peer can inject state).
- Isolation between elements sharing a broadcast domain.

These are known limitations appropriate for the current development stage.
Any findings related to them are welcome as issues rather than vulnerabilities.

Security-sensitive findings are those that could allow an attacker to:

- Subvert leader election or quorum from outside the broadcast domain.
- Cause persistent incorrect world-model state across a partition heal.
- Execute arbitrary code on an element via the gossip transport.
