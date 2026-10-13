# Mr Handy dashboard

This repository owns and builds its robocore_viz package. Docker no longer copies or
runs base101's rosboard package. The browser dashboard is maintained here.

Backend and legacy viewer sources were imported from robocore-labs/base101 at
commit 8cb3273223b1f7d06925d0e6819709d9ddbcc181 on 2026-10-07. Rosboard was
originally authored by Dheera Venkatraman: https://github.com/dheera/rosboard.
The original BSD license and third-party notices are retained in LICENSE.

Dashboard theme tokens originate in mod101/configurator/theme.css. Changes here
are independent of future base101 and mod101 updates.

The local fork was renamed to Robocore Viz (`robocore_viz`), including its
Python module, ROS package/executable and HTTP/WebSocket paths. Original upstream
authorship and license notices remain intact.

## Configurable vendored extraction

Extracted into XLeRobot on 2026-10-13 from the local Mr Handy package (repository
HEAD e68542f107aca3d913d3b22f10912296c8026e6a, including its working-tree dashboard).
XLeRobot owns this independent vendored copy; no edits were made to Mr Handy.
Robot-specific branding, topic bindings, telemetry and controller declarations
were moved into YAML profiles. The generic package supports trajectory actions
and omnidirectional teleoperation. Original BSD license and attribution remain.
