# Contributing to OpenSpaceArch

Thank you for your interest in contributing to OpenSpaceArch.

## How to Contribute

1. **Fork** the repository
2. Create a **feature branch** (`git checkout -b feature/my-feature`)
3. **Commit** your changes with clear messages
4. **Push** to your fork
5. Open a **Pull Request**

## Code Standards

- All code in **C#**, targeting .NET 9.0
- All comments, variable names, and documentation in **English**
- Every `.cs` file must include the AGPL v3.0 copyright header
- Keep it simple — no unnecessary abstractions

## What We Need

- **New engine types**: aerospike, electric propulsion, hybrid motors
- **Better physics**: iterative thermal marching, NASA CEA integration, structural analysis
- **More propellants**: N2O/Ethanol, LOX/H2, solid propellants
- **Validation**: comparison with published engine data, unit tests
- **Documentation**: tutorials, API docs, engineering guides

## Copyright Header

Every new `.cs` file must begin with:

```csharp
// OpenSpaceArch — Open Computational Architecture for Aerospace Hardware
// Copyright (C) 2025-2026 ultrasurreality
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU Affero General Public License for more details.
//
// You should have received a copy of the GNU Affero General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.
```

## Contributor License Agreement

By submitting a pull request, you agree that your contributions are licensed under the same AGPL v3.0 license that covers the project.

## Export Control

Do NOT submit any material that is:
- Classified or restricted under ITAR/EAR
- Received under NDA
- Derived from defense contracts or restricted technical data

OpenSpaceArch implements only general scientific and engineering principles that are publicly available.

## Questions?

Open a [GitHub Issue](https://github.com/ultrasurreality/OpenSpaceArch/issues) or start a [Discussion](https://github.com/ultrasurreality/OpenSpaceArch/discussions).
