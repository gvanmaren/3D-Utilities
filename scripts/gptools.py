import arcpy
import time
import os
import math
import sys


import scripts.create_3Dgravity_mains as create_3Dgravity_mains
import scripts.create_surface_hole as create_surface_hole
import scripts.create_3Dlaterals as create_3Dlaterals
import scripts.create_3Dmanholes as create_3Dmanholes
import scripts.create_elevation_tile_package as create_elevation_tile_package

import importlib
importlib.reload(create_3Dgravity_mains)  # force reload of the module
importlib.reload(create_3Dlaterals)  # force reload of the module
importlib.reload(create_3Dmanholes)  # force reload of the module
importlib.reload(create_surface_hole)  # force reload of the module
importlib.reload(create_elevation_tile_package)  # force reload of the module

import scripts.common_lib as common_lib
from scripts.common_lib import create_msg_body, msg, trace
from scripts.settings import *


class Create3DGravityMains(object):
    def __init__(self):
        """Define the tool (tool name is the name of the class)."""
        self.label = "Create 3D Gravity Mains"
        self.description = "Creates 3D gravity mains lines from 2D and 3D gravity mains " + \
                            "lines with start invert and end invert elevation attributes."
        self.canRunInBackground = False


    def getParameterInfo(self):
        """Define parameter definitions"""
        input_features = arcpy.Parameter(displayName="Input Features",
                                  name="InputFeatures",
                                  datatype=["DEFeatureClass", "GPLayer"],
                                  parameterType="Required",
                                  direction="Input")

        upper_elevation = arcpy.Parameter(displayName="Upper Invert Elevation",
                                  name="UpperInvertElevation",
                                  datatype = "GPString",
                                  parameterType="Required",
                                  direction="Input")

        lower_elevation = arcpy.Parameter(displayName="Lower Invert Elevation",
                                  name="LowerInvertElevation",
                                  datatype = "GPString",
                                  parameterType="Required",
                                  direction="Input")

        invert_unit = arcpy.Parameter(displayName="Invert Elevation Unit",
                                  name="InvertElevationUnit",
                                  datatype = "GPString",
                                  parameterType="Optional",
                                  direction="Input")

        diameter = arcpy.Parameter(displayName="Diameter",
                                  name="Diameter",
                                  datatype = "GPString",
                                  parameterType="Optional",
                                  direction="Input")

        diameter_unit = arcpy.Parameter(displayName="Diameter Unit",
                                  name="DiameterUnit",
                                  datatype = "GPString",
                                  parameterType="Required",
                                  direction="Input")

        default_diameter = arcpy.Parameter(displayName="Default Diameter",
                                  name="DefaultDiameter",
                                  datatype = "GPDouble",
                                  parameterType="Required",
                                  direction="Input")

        output_features = arcpy.Parameter(displayName="Output Features",
                                  name="OutputFeatures",
                                  datatype="DEFeatureClass",
                                  parameterType="Required",
                                  direction="Output")

        output_3dobjects = arcpy.Parameter(displayName="Output As 3D Objects",
                                  name="OutputAs3DObjects",
                                  datatype="GPBoolean",
                                  parameterType="Required",
                                  direction="Input")

        use_nearby_points= arcpy.Parameter(displayName="Use Nearby Points For Elevation",
                                  name="UseNearbyPointsForElevation",
                                  datatype="GPBoolean",
                                  parameterType="Optional",
                                  direction="Input")

        zero_as_error = arcpy.Parameter(displayName="Treat 0 as Error",
                                  name="Treat0asError",
                                  datatype="GPBoolean",
                                  parameterType="Optional",
                                  direction="Input")

        error_elevation = arcpy.Parameter(displayName="Error Elevation Value",
                                  name="ErrorElevationValue",
                                  datatype = "GPDouble",
                                  parameterType="Optional",
                                  direction="Input")

        interpolate_errors = arcpy.Parameter(displayName="Interpolate Errors",
                                  name="InterpolateErrors",
                                  datatype = "GPBoolean",
                                  parameterType="Optional",
                                  direction="Input")

        layer = arcpy.Parameter(displayName="layer",
                                  name="layer",
                                  datatype="GPFeatureLayer",
                                  parameterType="Derived", enabled=True,
                                  direction="Output")

        layer2 = arcpy.Parameter(displayName="layer2",
                                  name="layer2",
                                  datatype="GPFeatureLayer",
                                  parameterType="Derived", enabled=True,
                                  direction="Output")

        layer3 = arcpy.Parameter(displayName="layer3",
                                  name="layer3",
                                  datatype="GPFeatureLayer",
                                  parameterType="Derived", enabled=True,
                                  direction="Output")

        layer4 = arcpy.Parameter(displayName="layer4",
                                  name="layer4",
                                  datatype="GPFeatureLayer",
                                  parameterType="Derived", enabled=True,
                                  direction="Output")

        default_diameter.value = 1
        diameter_unit.enabled = False
        diameter_unit.value = UNDEFINED
        invert_unit.value = None
        use_nearby_points.enabled = False
        output_3dobjects.value = False
        zero_as_error.value = False
        interpolate_errors.value = False
        error_elevation.value = 9999

        zero_as_error.category = 'Error Handling'
        error_elevation.category = 'Error Handling'
        interpolate_errors.category = 'Error Handling'

        layer.parameterDependencies = [input_features.name]
        layer2.parameterDependencies = [input_features.name]
        layer3.parameterDependencies = [input_features.name]
        layer4.parameterDependencies = [input_features.name]

        aprx = arcpy.mp.ArcGISProject("CURRENT")
        layer_directory = aprx.homeFolder + "\\LayerFiles"

        layer.symbology = os.path.join(layer_directory, 'Line3DError.lyrx')
        layer2.symbology = os.path.join(layer_directory, 'Line3DError_meters.lyrx')
        layer3.symbology = os.path.join(layer_directory, 'LineObject3DError.lyrx')
        layer4.symbology = os.path.join(layer_directory, 'LineObject3DError_meters.lyrx')

        params = [input_features, upper_elevation, lower_elevation, invert_unit, diameter, diameter_unit, default_diameter,
                    output_features, output_3dobjects, use_nearby_points, zero_as_error, error_elevation, interpolate_errors,
                    layer, layer2, layer3, layer4]
        return params


    def isLicensed(self):
        """Set whether tool is licensed to execute."""
        return True


    def updateParameters(self, params):
        """Modify the values and properties of parameters before internal
        validation is performed.  This method is called whenever a parameter
        has been changed."""

        aprx = arcpy.mp.ArcGISProject("CURRENT")

        if params[0].value:
            if arcpy.Exists(params[0].value):
                fields = arcpy.ListFields(params[0].value)
                real_fields_list = []
                params[1].filter.list = []

                for f in fields:
                    if f.type == "Double" or f.type == "Integer" or f.type == "SmallInteger" or f.type == "Single":
                        real_fields_list.append(f.name)

                params[1].filter.list = sorted(set(real_fields_list))
                params[2].filter.list = sorted(set(real_fields_list))

                if params[1].value and params[2].value:
                    full_list = sorted(set(real_fields_list))
                    full_list.remove(params[1].value)
                    full_list.remove(params[2].value)
                    params[4].filter.list = full_list

        unitList1 = ["Inches", "Feet", "Millimeters", "Centimeters", "Meters"]
        unitList2 = [UNDEFINED, "Inches", "Feet", "Millimeters", "Centimeters", "Meters"]
        params[3].filter.list = unitList1
        params[5].filter.list = unitList2

        if params[4].value:
            params[5].enabled = True
        else:
            params[5].enabled = False

        return


    def updateMessages(self, params):
        """Modify the messages created by internal validation for each tool
        parameter.  This method is called after internal validation."""

        if params[4].value and not params[5].value:
            params[5].setErrorMessage('Diameter Unit is required if a diameter attribute has been selected!')
        return


    def execute(self, parameters, messages):

        class NoLayerFile(Exception):
            pass

        class NoOutput(Exception):
            pass

        try:
            """The source code of the tool."""
            (input_features, upper_elevation, lower_elevation, invert_unit, diameter, diameter_unit,
            default_diameter, output_features, output_3dobjects, use_nearby_points,
            zero_as_error, error_elevation, interpolate_errors) = [p.valueAsText for p in parameters[:-4]]

            if diameter_unit == UNDEFINED:
                diameter_unit = None

            # check if input exists
            if arcpy.Exists(parameters[0].value):
                lines_3d, objects_3d = create_3Dgravity_mains.main(input_layer=parameters[0].value, start_vertex_elevation=upper_elevation, end_vertex_elevation=lower_elevation,
                                                            vertex_elevation_unit=invert_unit, diameter=diameter, diameter_unit=diameter_unit,
                                                            default_diameter=parameters[6].value,
                                                            output_features=output_features,
                                                            output_as_3dobject=parameters[8].value,
                                                            use_nearby_points=parameters[9].value,
                                                            zero_as_error=parameters[10].value,
                                                            error_elevation=parameters[11].value,
                                                            interpolate_errors=parameters[12].value,
                                                            debug=0)

                if lines_3d:
                    arcpy.AddMessage("Adding: " + common_lib.get_name_from_feature_class(lines_3d))

                    if common_lib.get_z_unit(lines_3d, 0) == "Feet":
                        arcpy.SetParameter(13, lines_3d)
                    else:
                        arcpy.SetParameter(14, lines_3d)

                    if objects_3d:
                        if common_lib.get_z_unit(objects_3d, 0) == "Feet":
                            arcpy.SetParameter(15, objects_3d)
                        else:
                            arcpy.SetParameter(16, objects_3d)
                else:
                    raise NoOutput
            else:
                raise NoLayerFile

        except NoLayerFile:
            print("Can't find Layer file. Exiting...")
            arcpy.AddError("Can't find Layer file. Exiting...")

        except NoOutput:
            print("Can't create output. Exiting...")
            arcpy.AddError("Can't create output. Exiting...")


class Create3DLaterals(object):
    def __init__(self):
        """Define the tool (tool name is the name of the class)."""
        self.label = "Create 3D Laterals"
        self.description = "Creates 3D lateral lines from 2D and 3D laterals " + \
                            "using 3D gravity mains as input."
        self.canRunInBackground = False

    def getParameterInfo(self):
        """Define parameter definitions"""
        input_features = arcpy.Parameter(displayName="Input Features",
                                  name="InputFeatures",
                                  datatype=["DEFeatureClass", "GPLayer"],
                                  parameterType="Required",
                                  direction="Input")

        input_3Dmains = arcpy.Parameter(displayName="3D Gravity Mains",
                                  name="3DGravitymains",
                                  datatype=["DEFeatureClass", "GPLayer"],
                                  parameterType="Required",
                                  direction="Input")

        diameter = arcpy.Parameter(displayName="Diameter",
                                  name="Diameter",
                                  datatype = "GPString",
                                  parameterType="Optional",
                                  direction="Input")

        diameter_unit = arcpy.Parameter(displayName="Diameter Unit",
                                  name="DiameterUnit",
                                  datatype = "GPString",
                                  parameterType="Required",
                                  direction="Input")

        default_diameter = arcpy.Parameter(displayName="Default Diameter",
                                  name="DefaultDiameter",
                                  datatype = "GPDouble",
                                  parameterType="Required",
                                  direction="Input")

        slope = arcpy.Parameter(displayName="Slope",
                                  name="Slope",
                                  datatype = "GPString",
                                  parameterType="Optional",
                                  direction="Input")

        default_slope = arcpy.Parameter(displayName="Default Slope",
                                  name="DefaultSlope",
                                  datatype = "GPDouble",
                                  parameterType="Required",
                                  direction="Input")

        output_features = arcpy.Parameter(displayName="Output Features",
                                  name="OutputFeatures",
                                  datatype="DEFeatureClass",
                                  parameterType="Required",
                                  direction="Output")

        output_3dobjects = arcpy.Parameter(displayName="Output As 3D Objects",
                                  name="OutputAs3DObjects",
                                  datatype="GPBoolean",
                                  parameterType="Required",
                                  direction="Input")

        layer = arcpy.Parameter(displayName="layer",
                                  name="layer",
                                  datatype="GPFeatureLayer",
                                  parameterType="Derived", enabled=True,
                                  direction="Output")

        layer2 = arcpy.Parameter(displayName="layer2",
                                  name="layer2",
                                  datatype="GPFeatureLayer",
                                  parameterType="Derived", enabled=True,
                                  direction="Output")

        layer3 = arcpy.Parameter(displayName="layer3",
                                  name="layer3",
                                  datatype="GPFeatureLayer",
                                  parameterType="Derived", enabled=True,
                                  direction="Output")

        layer4 = arcpy.Parameter(displayName="layer4",
                                  name="layer4",
                                  datatype="GPFeatureLayer",
                                  parameterType="Derived", enabled=True,
                                  direction="Output")

        default_diameter.value = 0.5
        diameter_unit.enabled = False
        diameter_unit.value = UNDEFINED
        default_slope.value = 2
        output_3dobjects.value = False

        layer.parameterDependencies = [input_features.name]
        layer2.parameterDependencies = [input_features.name]
        layer3.parameterDependencies = [input_features.name]
        layer4.parameterDependencies = [input_features.name]

        aprx = arcpy.mp.ArcGISProject("CURRENT")
        layer_directory = aprx.homeFolder + "\\LayerFiles"

        layer.symbology = os.path.join(layer_directory, 'LateralLine3D.lyrx')
        layer2.symbology = os.path.join(layer_directory, 'LateralLine3D_meter.lyrx')
        layer3.symbology = os.path.join(layer_directory, 'LateralObject3D.lyrx')
        layer4.symbology = os.path.join(layer_directory, 'LateralObject3D_meter.lyrx')

        params = [input_features, input_3Dmains, diameter, diameter_unit, default_diameter,
                   slope, default_slope,
                   output_features, output_3dobjects,
                   layer, layer2, layer3, layer4]
        return params


    def isLicensed(self):
        """Set whether tool is licensed to execute."""
        return True


    def updateParameters(self, params):
        """Modify the values and properties of parameters before internal
        validation is performed.  This method is called whenever a parameter
        has been changed."""

        if params[0].value:
            if arcpy.Exists(params[0].value):
                fields = arcpy.ListFields(params[0].value)
                real_fields_list = []

                for f in fields:
                    if f.type == "Double" or f.type == "Integer" or f.type == "SmallInteger" or f.type == "Single":
                        real_fields_list.append(f.name)

                full_list = sorted(set(real_fields_list))
                params[2].filter.list = full_list

                if params[2].value:
                    full_list.remove(params[2].value)

                params[5].filter.list = full_list

        unitList = ["Undefined", "Inches", "Feet", "Millimeter", "Centimeter", "Meter"]
        params[3].filter.list = unitList

        if params[2].value:
            params[3].enabled = True

        return


    def updateMessages(self, parameters):
        """Modify the messages created by internal validation for each tool
        parameter.  This method is called after internal validation."""
        return


    def execute(self, parameters, messages):
        class NoLayerFile(Exception):
            pass

        class NoOutput(Exception):
            pass

        try:
            """The source code of the tool."""
            (input_features, input_3Dmains, diameter, diameter_unit, default_diameter, slope, default_slope,
                output_features, output_3dobjects) = [p.valueAsText for p in parameters[:-4]]

            if diameter_unit == UNDEFINED:
                diameter_unit = None

            # check if input exists
            if arcpy.Exists(parameters[0].value):
                lines_3d, objects_3d = create_3Dlaterals.main(input_layer=parameters[0].value,
                                                    input_3d_mains_layer=parameters[1].value,
                                                    diameter=diameter,
                                                    diameter_unit=diameter_unit,
                                                    default_diameter=parameters[3].value,
                                                    slope=slope,
                                                    default_slope=default_slope,
                                                    output_features=output_features,
                                                    output_as_3dobject=parameters[7].value,
                                                    debug=0)

                if lines_3d:
                    arcpy.AddMessage("Adding: " + common_lib.get_name_from_feature_class(lines_3d))

                    if common_lib.get_z_unit(lines_3d, 0) == "Feet":
                        arcpy.SetParameter(9, lines_3d)
                    else:
                        arcpy.SetParameter(10, lines_3d)

                    if objects_3d:
                        if common_lib.get_z_unit(objects_3d, 0) == "Feet":
                            arcpy.SetParameter(11, objects_3d)
                        else:
                            arcpy.SetParameter(12, objects_3d)
                else:
                    raise NoOutput
            else:
                raise NoLayerFile

        except NoLayerFile:
            print("Can't find Layer file. Exiting...")
            arcpy.AddError("Can't find Layer file. Exiting...")

        except NoOutput:
            print("Can't create output. Exiting...")
            arcpy.AddError("Can't create output. Exiting...")


class Create3DManholes(object):
    def __init__(self):
        """Define the tool (tool name is the name of the class)."""
        self.label = "Create 3D Manholes"
        self.description = "Creates 3D manhole points from 2D and 3D manholes " + \
                            "points with rim and invert elevation attributes."
        self.canRunInBackground = False

    def getParameterInfo(self):
        """Define parameter definitions"""
        input_features = arcpy.Parameter(displayName="Input Features",
                                  name="InputFeatures",
                                  datatype=["DEFeatureClass", "GPLayer"],
                                  parameterType="Required",
                                  direction="Input")

        upper_elevation = arcpy.Parameter(displayName="Upper Invert Elevation",
                                  name="UpperInvertElevation",
                                  datatype = "GPString",
                                  parameterType="Required",
                                  direction="Input")

        lower_elevation = arcpy.Parameter(displayName="Lower Invert Elevation",
                                  name="LowerInvertElevation",
                                  datatype = "GPString",
                                  parameterType="Required",
                                  direction="Input")

        invert_unit = arcpy.Parameter(displayName="Invert Elevation Unit",
                                  name="InvertElevationUnit",
                                  datatype = "GPString",
                                  parameterType="Optional",
                                  direction="Input")

        diameter = arcpy.Parameter(displayName="Diameter",
                                  name="Diameter",
                                  datatype = "GPString",
                                  parameterType="Optional",
                                  direction="Input")

        diameter_unit = arcpy.Parameter(displayName="Diameter Unit",
                                  name="DiameterUnit",
                                  datatype = "GPString",
                                  parameterType="Required",
                                  direction="Input")

        default_diameter = arcpy.Parameter(displayName="Default Diameter",
                                  name="DefaultDiameter",
                                  datatype = "GPDouble",
                                  parameterType="Required",
                                  direction="Input")

        output_features = arcpy.Parameter(displayName="Output Features",
                                  name="OutputFeatures",
                                  datatype="DEFeatureClass",
                                  parameterType="Required",
                                  direction="Output")

        output_3dobjects = arcpy.Parameter(displayName="Output As 3D Objects",
                                  name="OutputAs3DObjects",
                                  datatype="GPBoolean",
                                  parameterType="Required",
                                  direction="Input")

        zero_as_error = arcpy.Parameter(displayName="Treat 0 as Error",
                                  name="Treat0asError",
                                  datatype="GPBoolean",
                                  parameterType="Optional",
                                  direction="Input")

        error_elevation = arcpy.Parameter(displayName="Error Elevation Value",
                                  name="ErrorElevationValue",
                                  datatype = "GPDouble",
                                  parameterType="Optional",
                                  direction="Input")

        interpolate_errors = arcpy.Parameter(displayName="Interpolate Errors",
                                  name="InterpolateErrors",
                                  datatype = "GPBoolean",
                                  parameterType="Optional",
                                  direction="Input")

        input_raster = arcpy.Parameter(displayName="Terrain Surface",
                                  name="TerrainSurface",
                                  datatype="GPRasterLayer",
                                  parameterType="Optional",
                                  direction="Input")

        layer = arcpy.Parameter(displayName="layer",
                                  name="layer",
                                  datatype="GPFeatureLayer",
                                  parameterType="Derived", enabled=True,
                                  direction="Output")

        layer2 = arcpy.Parameter(displayName="layer2",
                                  name="layer2",
                                  datatype="GPFeatureLayer",
                                  parameterType="Derived", enabled=True,
                                  direction="Output")

        layer3 = arcpy.Parameter(displayName="layer3",
                                  name="layer3",
                                  datatype="GPFeatureLayer",
                                  parameterType="Derived", enabled=True,
                                  direction="Output")

        layer4 = arcpy.Parameter(displayName="layer4",
                                  name="layer4",
                                  datatype="GPFeatureLayer",
                                  parameterType="Derived", enabled=True,
                                  direction="Output")

        default_diameter.value = 1
        diameter_unit.enabled = False
        diameter_unit.value = UNDEFINED
        invert_unit.value = None
        output_3dobjects.value = False
        zero_as_error.value = False
        interpolate_errors.value = False
        error_elevation.value = 9999
        input_raster.value = None
        input_raster.enabled = False

        zero_as_error.category = 'Error Handling'
        error_elevation.category = 'Error Handling'
        interpolate_errors.category = 'Error Handling'
        input_raster.category = 'Error Handling'

        layer.parameterDependencies = [input_features.name]
        layer2.parameterDependencies = [input_features.name]
        layer3.parameterDependencies = [input_features.name]
        layer4.parameterDependencies = [input_features.name]

        aprx = arcpy.mp.ArcGISProject("CURRENT")
        layer_directory = aprx.homeFolder + "\\LayerFiles"

        layer.symbology = os.path.join(layer_directory, 'Point3DError.lyrx')
        layer2.symbology = os.path.join(layer_directory, 'Point3DError_meter.lyrx')
        layer3.symbology = os.path.join(layer_directory, 'PointObject3DError.lyrx')
        layer4.symbology = os.path.join(layer_directory, 'PointObject3DError_meter.lyrx')

        params = [input_features, upper_elevation, lower_elevation, invert_unit, diameter, diameter_unit, default_diameter,
                    output_features, output_3dobjects, zero_as_error, error_elevation, interpolate_errors, input_raster,
                    layer, layer2, layer3, layer4]
        return params


    def isLicensed(self):
        """Set whether tool is licensed to execute."""
        return True


    def updateParameters(self, params):
        """Modify the values and properties of parameters before internal
        validation is performed.  This method is called whenever a parameter
        has been changed."""

        aprx = arcpy.mp.ArcGISProject("CURRENT")

        if params[0].value:
            if arcpy.Exists(params[0].value):
                fields = arcpy.ListFields(params[0].value)
                real_fields_list = []
                params[1].filter.list = []

                for f in fields:
                    if f.type == "Double" or f.type == "Integer" or f.type == "SmallInteger" or f.type == "Single":
                        real_fields_list.append(f.name)

                params[1].filter.list = sorted(set(real_fields_list))
                params[2].filter.list = sorted(set(real_fields_list))

                if params[1].value and params[2].value:
                    full_list = sorted(set(real_fields_list))
                    full_list.remove(params[1].value)
                    full_list.remove(params[2].value)
                    params[4].filter.list = full_list

        unitList1 = ["Inches", "Feet", "Millimeters", "Centimeters", "Meters"]
        unitList2 = [UNDEFINED, "Inches", "Feet", "Millimeters", "Centimeters", "Meters"]
        params[3].filter.list = unitList1
        params[5].filter.list = unitList2

        if params[4].value:
            params[5].enabled = True
        else:
            params[5].enabled = False

        if params[11].value:
            params[12].enabled = True
        else:
            params[12].enabled = False

        return


    def updateMessages(self, params):
        """Modify the messages created by internal validation for each tool
        parameter.  This method is called after internal validation."""

        if params[4].value and not params[5].value:
            params[5].setErrorMessage('Diameter Unit is required if a diameter attribute has been selected!')

        if params[11].value and not params[12].value:
            params[12].setErrorMessage('Terrain Surface is required if Interpolate Errors is set!')

        return


    def execute(self, parameters, messages):
        class NoLayerFile(Exception):
            pass

        class NoOutput(Exception):
            pass

        try:
            """The source code of the tool."""
            (input_features, upper_elevation, lower_elevation, invert_unit, diameter, diameter_unit,
            default_diameter, output_features, output_3dobjects,
            zero_as_error, error_elevation, interpolate_errors, input_raster) = [p.valueAsText for p in parameters[:-4]]

            if diameter_unit == UNDEFINED:
                diameter_unit = None

            # check if input exists
            if arcpy.Exists(parameters[0].value):
                points_3d, objects_3d = create_3Dmanholes.main(input_layer=parameters[0].value, rim_elevation=upper_elevation, invert_elevation=lower_elevation,
                                                            vertex_elevation_unit=invert_unit, diameter=diameter, diameter_unit=diameter_unit,
                                                            default_diameter=parameters[6].value,
                                                            output_features=output_features,
                                                            output_as_3dobject=parameters[8].value,
                                                            zero_as_error=parameters[9].value,
                                                            error_elevation=parameters[10].value,
                                                            interpolate_errors=parameters[11].value,
                                                            terrain_surface=parameters[12].value,
                                                            debug=0)

                if points_3d:
                    arcpy.AddMessage("Adding: " + common_lib.get_name_from_feature_class(points_3d))

                    if common_lib.get_z_unit(points_3d, 0) == "Feet":
                        arcpy.SetParameter(13, points_3d)
                    else:
                        arcpy.SetParameter(14, points_3d)

                    if objects_3d:
                        if common_lib.get_z_unit(objects_3d, 0) == "Feet":
                            arcpy.SetParameter(15, objects_3d)
                        else:
                            arcpy.SetParameter(16, objects_3d)
                else:
                    raise NoOutput
            else:
                raise NoLayerFile

        except NoLayerFile:
            print("Can't find Layer file. Exiting...")
            arcpy.AddError("Can't find Layer file. Exiting...")

        except NoOutput:
            print("Can't create output. Exiting...")
            arcpy.AddError("Can't create output. Exiting...")


class CreateSurfaceHole(object):
    def __init__(self):
        """Define the tool (tool name is the name of the class)."""
        self.label = "Create Surface Hole"
        self.description = "Creates a surface that can be used to create a hole in the elevation " + \
                            "surface so that utilities can be viewed from below the surface."
        self.canRunInBackground = False

    def getParameterInfo(self):
        """Define parameter definitions"""
        input_raster = arcpy.Parameter(displayName="Input Surface",
                                  name="InputSurface",
                                  datatype="GPRasterLayer",
                                  parameterType="Optional",
                                  direction="Input")

        input_features = arcpy.Parameter(displayName="Input Features",
                                  name="InputFeatures",
                                  datatype=["DEFeatureClass", "GPLayer"],
                                  parameterType="Required",
                                  direction="Input")

        depth = arcpy.Parameter(displayName="Depth",
                                  name="Depth",
                                  datatype = "GPDouble",
                                  parameterType="Required",
                                  direction="Input")

        output_raster = arcpy.Parameter(displayName="Output Surface",
                                  name="OutputSurface",
                                  datatype="GPRasterLayer",
                                  parameterType="Optional",
                                  direction="Output")

        layer = arcpy.Parameter(displayName="layer",
                                  name="layer",
                                  datatype="GPLayer",
                                  parameterType="Derived", enabled=True,
                                  direction="Output")

        layer2 = arcpy.Parameter(displayName="layer",
                                  name="layer",
                                  datatype="GPFeatureLayer",
                                  parameterType="Derived", enabled=True,
                                  direction="Output")

        aprx = arcpy.mp.ArcGISProject("CURRENT")
        layer_directory = aprx.homeFolder + "\\LayerFiles"

        layer2.symbology = os.path.join(layer_directory, 'hole_texture2.lyrx')

        params = [input_raster, input_features, depth, output_raster, layer, layer2]
        return params

    def isLicensed(self):
        """Set whether tool is licensed to execute."""
        return True


    def updateParameters(self, params):
        """Modify the values and properties of parameters before internal
        validation is performed.  This method is called whenever a parameter
        has been changed."""

        return

    def updateMessages(self, params):
        """Modify the messages created by internal validation for each tool
        parameter.  This method is called after internal validation."""

        return

    def execute(self, parameters, messages):
        class NoLayerFile(Exception):
            pass

        class NoOutput(Exception):
            pass

        try:
            """The source code of the tool."""
            # check if input exists
            if arcpy.Exists(parameters[0].value):
                surface, polygon = create_surface_hole.main(input_raster=parameters[0].value,
                                                         input_layer=parameters[1].value,
                                                         depth=parameters[2].value,
                                                         output_raster=parameters[3].valueAsText,
                                                         debug=0)

                if surface:
                    arcpy.AddMessage("Created: " + common_lib.get_name_from_feature_class(surface))
                    arcpy.SetParameter(4, surface)

                    if polygon:
                        arcpy.AddMessage("Adding: " + common_lib.get_name_from_feature_class(polygon) + " as extent with texture.")
                        arcpy.SetParameter(5, polygon)
                else:
                    raise NoOutput
            else:
                raise NoLayerFile

        except NoLayerFile:
            print("Can't find Layer file. Exiting...")
            arcpy.AddError("Can't find Layer file. Exiting...")

        except NoOutput:
            print("Can't create output. Exiting...")
            arcpy.AddError("Can't create output. Exiting...")


class CreateElevationTilePackage(object):
    def __init__(self):
        """Define the tool (tool name is the name of the class)."""
        self.label = "Create Elevation Tile Package"
        self.description = "Creates an Elevation Tile Package (*.tpk) from elevation datasource."
        self.canRunInBackground = False

    def getParameterInfo(self):
        """Define parameter definitions"""
        input_raster = arcpy.Parameter(displayName="Input Elevation Source",
                                  name="InputElevationSource",
                                  datatype="GPRasterLayer",
                                  parameterType="Optional",
                                  direction="Input")

        scale = arcpy.Parameter(displayName="Minimum Cached Scale Level",
                                  name="MCSL",
                                  datatype = "GPLong",
                                  parameterType="Required",
                                  direction="Input")

        pixel_tolerance = arcpy.Parameter(displayName="Pixel Tolerance",
                                  name="PixelTolerance",
                                  datatype = "GPDouble",
                                  parameterType="Required",
                                  direction="Input")

        output_workspace = arcpy.Parameter(displayName="Output Cache Directory",
                                  name="OutputCacheDirectory",
                                  datatype="DEWorkspace",
                                  parameterType="Required",
                                  direction="Input")


        scale.filter.type = 'Range'
        scale.filter.list = [0,19]
        pixel_tolerance.filter.type = 'Range'
        pixel_tolerance.filter.list = [0,1]

        params = [input_raster, scale, pixel_tolerance, output_workspace]
        return params

    def isLicensed(self):
        """Set whether tool is licensed to execute."""
        return True


    def updateParameters(self, params):
        """Modify the values and properties of parameters before internal
        validation is performed.  This method is called whenever a parameter
        has been changed."""

        return

    def updateMessages(self, params):
        """Modify the messages created by internal validation for each tool
        parameter.  This method is called after internal validation."""

        return

    def execute(self, parameters, messages):
        class NoLayerFile(Exception):
            pass

        class NoOutput(Exception):
            pass

        try:
            """The source code of the tool."""
            # check if input exists
            if arcpy.Exists(parameters[0].value):
                arcpy.AddMessage("Creating tpk for: " + common_lib.get_name_from_feature_class(common_lib.get_name_from_feature_class(parameters[0].value)))
                cache = create_elevation_tile_package.main(input_raster=parameters[0].value,
                                                                minimum_scale_level= parameters[1].valueAsText,
                                                                pixel_tolerance=parameters[2].valueAsText,
                                                                output_ws=parameters[3].valueAsText,
                                                                debug=0)

                if cache:
                    arcpy.AddMessage("Elevation Tile Package created: " + parameters[3].valueAsText)
                else:
                    raise NoOutput
            else:
                raise NoLayerFile

        except NoLayerFile:
            print("Can't find Layer file. Exiting...")
            arcpy.AddError("Can't find Layer file. Exiting...")

        except NoOutput:
            print("Can't create output. Exiting...")
            arcpy.AddError("Can't create output. Exiting...")



# for debug only!
def main():

#    create_3Dgravity_mains.main("", "", "", "", "", "", "", "", "", "", "", "", "", 1)
#    create_3Dlaterals.main("", "", "", "", "", "", "", "", "", 1)
#    create_3Dmanholes.main("", "", "", "", "", "", "", "", "", "", "", "", "", 1)
#    create_surface_hole.main("", "", "", "", 1)
    create_elevation_tile_package.main("", "", "", "", 1)

if __name__ == "__main__":
    main()


