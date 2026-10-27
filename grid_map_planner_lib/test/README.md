# Planning safety regression tests

The default GTest suite checks invalid distance-transform seeds, preservation of
cached fields, and shortcut occupancy/clearance validation with synthetic maps.
These are direct library calls; no ROS master or robot is needed.

The opt-in `StraightLineExample.DISABLED_FailedRequestsPreserveRoutedPaths` test
uses the map, zones and four requests from `straight-line-repro.zip`. Keep the
customer fixture and generated configuration outside this public repository.
The loader applies ROS trinary thresholds and grid_map cell-centre polygons,
GO before NO_GO. It checks failed trigger requests, unchanged cached distances,
and both subsequent routes against fresh-planner paths. Every segment is also
checked against occupancy; pose count alone is not the safety assertion.

After building the test target with testing enabled, run these commands in the
same sourced ROS environment and filesystem namespace as the build:

```bash
python3 test/prepare_straight_line_example.py /path/to/straight-line-repro /private/repro.json
GRID_MAP_PLANNER_REPRO_CONFIG=/private/repro.json \
  rosrun grid_map_planner_lib grid_map_planner_lib_gtest \
  --gtest_also_run_disabled_tests --gtest_filter='StraightLineExample.*' \
  --gtest_output=xml:/private/straight-line-result.xml
```

Use the workspace's build/test instructions to configure and run the default
suite. The external test is disabled by default because the customer data is
not distributed with the package; explicitly enabling it without a config fails.
This checks the C++ planner, not the cloud service's transport/container image.
