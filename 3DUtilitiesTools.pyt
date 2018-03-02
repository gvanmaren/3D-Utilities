import scripts.gptools as gptools
import importlib
importlib.reload(gptools)  # force reload of the module
from scripts.gptools import *


class Toolbox(object):
    def __init__(self):
        """Define the toolbox (the name of the toolbox is the name of the
        .pyt file)."""
        self.label = "3D Utilities Tools"
        self.alias = "3D Utilities Tools"

        # List of tool classes associated with this toolbox
        self.tools = [Create3DGravityMains, Create3DLaterals, Create3DManholes, CreateSurfaceHole, CreateElevationTilePackage]
