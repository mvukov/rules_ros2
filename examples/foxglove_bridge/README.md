# Foxglove bridge demonstration

Run

```sh
bazel //foxglove_bridge
```

to start the sample publisher and the Foxglove bridge node.
Then you can start using [Foxglove Studio](https://studio.foxglove.dev/).

As a data source you can choose `ws://localhost:8765` to start streaming data
from this example deployment.

Interesting panels to use with this example:

- [3D](https://foxglove.dev/docs/studio/panels/3d) to display a sample
  robot URDF model.
- [Log](https://foxglove.dev/docs/studio/panels/log) to display ROS log messages.
- [Plot](https://foxglove.dev/docs/studio/panels/plot) to e.g. plot data from
  `/point` topic.
