# rc_rx200

This package is a staging wrapper for files imported from
`drive-download-20260411T054645Z-3-001.zip`.

Only files that are either:

- different from the current local workspace copy, or
- present in the zip but missing locally

are included here.

The imported files are stored under `upstream/` using their original package
paths. This avoids duplicate ROS package names in the workspace while keeping
the source layout recognizable.

Included upstream sources:

- `upstream/rx200_handbag_grasp/...`
- `upstream/lift_bridge/...`
- `upstream/lift_moveit_config/...`

Duplicate-package-name avoidance:

- The only package exported to colcon is `rc_rx200`.
- Original upstream manifests are preserved as `*.snapshot` files under
  `upstream/`; they are not separate workspace packages.
- If you later promote one upstream tree into a runnable package, rename the
  ROS package in its `package.xml` and any `find_package`, launch, or executable
  references at the same time.
