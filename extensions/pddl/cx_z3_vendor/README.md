# cx_z3_vendor
Provides a specific version of the z3 solver (https://github.com/Z3Prover/z3), which works fine in combination with the NEXTFLAP planner.

Apparently, new versions of z3 are unable to solve the smt problems emitted by NEXTFLAP for finding temporal plans.
In order to identify the issue, changes of default parameters reverted and newer features were disabled without success.

Hence, this package provides a legacy version that is capable of solving the problems emitted by the planner.

The z3 solver itself is licensed under MIT, this vendor utilizes the Apache-2.0 license.

# Usage

## CLI
Simply run `z3`:
```bash
z3
```

## Shared Libraries

in CMake:

```cmake
find_package(cx_z3_vendor)
find_package(Z3)

...

target_link_libraries(<target> PUBLIC z3::libz3)
```



