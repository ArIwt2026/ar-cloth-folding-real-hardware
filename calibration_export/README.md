# Calibration export

This export records only the transforms currently used by
`static_publishers.launch.py`. That launch file is the sole source of truth for
this export; other calibration YAML files are intentionally excluded.

## Frame convention

All transforms are parent-to-child TF transforms, with translation in metres and
quaternion order `x, y, z, w`.

The current `world` frame is not a permanent surveyed world frame. It is a
frozen snapshot of the table AprilTag, represented by:

```text
world -> lucid_triton_color_optical_frame
```

The Lucid Triton optical frame is therefore the calibration anchor. The Panda
and KUKA bases are positioned relative to that anchor.

## Active transform chain

```text
world
└── lucid_triton_color_optical_frame
    ├── iiwa7_link_0
    │   └── iiwa7_link_ee
    │       └── kuka_d455_link
    │           └── kuka_d455_color_optical_frame  (camera driver)
    └── panda_link0
        └── panda_link8
            └── panda_d455_link
                └── panda_d455_color_optical_frame (camera driver)
```

The camera-driver optical-frame transforms are not duplicated here; the
RealSense driver publishes them.

## Sources

- Active static transforms: `../static_publishers.launch.py`
- Sole source: `../static_publishers.launch.py`
- Triton → Helios camera transform: `../lucid_cameras/docker/publish_calibration.py`
