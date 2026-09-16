"""Shared schema helpers; no runtime nodes or transport.

Keep the source package and catkin-generated ``common_msgs_pkg.msg`` namespace
composable when both devel paths are present.
"""

from pkgutil import extend_path


__path__ = extend_path(__path__, __name__)
