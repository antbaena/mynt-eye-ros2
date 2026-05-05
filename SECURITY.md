# Security Policy

This project is a local hardware driver for ROS 2. It does not intentionally expose network services.

## Reporting

Please report security-sensitive issues privately to the maintainer listed in `package.xml` before opening a public issue.

## Scope

Security reports are most useful when they involve:

- unsafe parsing of device data;
- unexpected filesystem writes;
- command execution in setup scripts;
- privilege or udev guidance that could weaken the host system.

General camera calibration, image quality and ROS topic behavior should be reported as normal issues.
