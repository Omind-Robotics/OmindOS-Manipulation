# Attribution and Project Lineage

OmindOS Manipulation Community is built from the existing `urmanipulation` codebase and preserves its open-source lineage.

The repository's MIT LICENSE identifies:

- **Copyright (c) 2020 Weiwei Wan**

The earlier project documentation states that the UR3e planning and control system originated from / was based on the WRS robotics software at Osaka University.

The Community edition reorganizes documentation and learning entry points around OmindOS while retaining the upstream license and attribution. It does not claim authorship of upstream code.

When redistributing this repository or substantial portions of it, retain the MIT license notice and applicable attribution.

## Community additions

New documentation, examples, fixes, adapters, and other contributions may be added over time by Community contributors. Their inclusion does not alter the licensing requirements attached to upstream material.

## RealMan RM75-B assets

`community/models/realman_rm75/` contains manufacturer URDF/STL model data from
RealManRobot/rm_models, Copyright 2024 realman-robotics, under Apache-2.0.
These assets retain their separate license; they are not relicensed under MIT.
See the [asset source, pinned revision and modification notice](community/models/realman_rm75/NOTICE.md).

## Navigation v0.1 optional runtime integration (0.2)

The Topic contract is documented by [OmindOS Navigation Community](https://github.com/maganrobotics-boop/OmindOS-Navigation-Community) at `ab456fdde986e7f3904d47fcfc87dc944a6bb719`. The optional external runtime archive is pinned by SHA256 in the integration guide and CI. Its implementation is not copied or redistributed in this source tree. The standalone grid planner and Topic adapter are new Community code. Runtime dependencies retain their own licenses and provenance; see the Navigation repository notices.
