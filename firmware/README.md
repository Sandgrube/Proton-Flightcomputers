# Firmware (PROTON)

This folder contains the **firmware codebase** for the PROTON flight-computer ecosystem.

## Version / Naming Convention

You will see names like **PROTON-2** or **PROTON-3** in code, folders, branches, or build targets.

**Important:** The first number after the word **PROTON** represents the **generation of the firmware/code**, **not** the hardware/flight-computer generation.

Example:
- `PROTON-3` = third major iteration of the **software architecture**
- It may run on different hardware revisions over time.

## Development Status

🚧 **Under active development**  
Expect breaking changes, refactors, incomplete features, and evolving interfaces.

- APIs, pin mappings, and config formats may change without notice.
- Logs and telemetry formats are not stable yet.
- Anything safety-critical must be validated on a bench setup before field use.

## Structure (typical)

