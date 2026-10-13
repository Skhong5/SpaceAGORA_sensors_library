# Guidance, navigation and control packages

This directory records the intended home for optional GNC integrations. It does
not currently contain an installable EDG package. Energy-depletion guidance and
its simulation lifecycle remain in SpaceAGORA core.

A future extraction needs a supported, versioned service boundary, clear
ownership of guidance state and commands, and scientific qualification of the
implementation being moved. The [EDG boundary preparation](../../docs/src/maintainer/edg_adapter_boundary.md)
records those decisions and unresolved comparisons; creating this directory
does not resolve them or change the controller's behavior.

HYPR's source checkout is under [robotics/](../robotics/README.md), although it
also supplies spacecraft RPO planning. Its existing integration remains the
supported route for that capability. There is no `edg` target in the shared
optional-package command and no placeholder Julia package to install here.
