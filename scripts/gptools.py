import arcpy
import arcpy.cartography as CA
import time
import os
import scripts.CommonLib as CommonLib
from scripts.CommonLib import create_msg_body, msg, trace
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
                                  datatype = "GPString",
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
                                  datatype = "Double",
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
        use_nearby_points.enabled = False
        output_3dobjects.value = False
        zero_as_error.value = False
        interpolate_errors.value = False
        error_elevation.value = 9999

        zero_as_error.category = 'Error Handling'
        error_elevation.category = 'Error Handling'
        interpolate_errors.category = 'Error Handling'

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

        unitList = ["Inches", "Feet", "Millimeters", "Centimeters", "Meters"]
        params[3].filter.list = unitList
        params[5].filter.list = unitList

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

    @staticmethod
    def run(input_features, upper_elevation, lower_elevation,
                invert_unit, diameter, diameter_unit, default_diameter,
                output_features, output_3dobjects, use_nearby_points,
                zero_as_error, error_elevation, interpolate_errors,
                debug):

        """The source code of the tool."""
        # error classes

        class NoNoDataError(Exception):
            pass

        class LicenseError3D(Exception):
            pass

        class LicenseErrorSpatial(Exception):
            pass

        class SchemaLock(Exception):
            pass

        class NotSupported(Exception):
            pass

        class NoLayerFile(Exception):
            pass

        class FunctionError(Exception):

            pass

        try:
            # Get Attributes from User
            if debug == 0:
                # script variables
                aprx = arcpy.mp.ArcGISProject("CURRENT")
                home_directory = aprx.homeFolder
                tiff_directory = home_directory + "\\Tiffs"
                tin_directory = home_directory + "\\Tins"
                scripts_directory = aprx.homeFolder + "\\Scripts"
                rule_directory = aprx.homeFolder + "\\RulePackages"
                log_directory = aprx.homeFolder + "\\Logs"
                layer_directory = home_directory + "\\LayerFiles"
                project_ws = aprx.defaultGeodatabase

                enableLogging = True
                DeleteIntermediateData = True
                verbose = 0
                in_memory_switch = True
            else:
                # debug
                input_raster = r'D:\Gert\Work\Esri\Solutions\3D FloodImpact\work2.1\3DFloodImpact\3DFloodImpact.gdb\c2ft_inundation_Clip'
                output_polygons = r'D:\Gert\Work\Esri\Solutions\3D FloodImpact\work2.1\3DFloodImpact\Results.gdb\FloodPolys'
                no_flood_value = "0" # "NoData"  # value or "NoData"
                outward_buffer = 0
                sea_level_rise = 2
                home_directory = r'D:\Gert\Work\Esri\Solutions\3D FloodImpact\work2.1\3DFloodImpact'
                tiff_directory = home_directory + "\\Tiffs"
                tin_directory = home_directory + "\\Tins"
                scripts_directory = home_directory + "\\Scripts"
                rule_directory = home_directory + "\\RulePackages"
                log_directory = home_directory + "\\Logs"
                layer_directory = home_directory + "\\LayerFiles"
                project_ws = home_directory + "\\Results.gdb"

                enableLogging = False
                DeleteIntermediateData = True
                verbose = 1
                in_memory_switch = False

            scratch_ws = CommonLib.create_gdb(home_directory, "Intermediate.gdb")
            arcpy.env.workspace = scratch_ws
            arcpy.env.overwriteOutput = True

            if not os.path.exists(tiff_directory):
                os.makedirs(tiff_directory)

            if not os.path.exists(tin_directory):
                os.makedirs(tin_directory)

            CommonLib.set_up_logging(log_directory, TOOLNAME1)
            start_time = time.clock()

            if arcpy.CheckExtension("3D") == "Available":
                arcpy.CheckOutExtension("3D")

                if arcpy.CheckExtension("Spatial") == "Available":
                    arcpy.CheckOutExtension("Spatial")

                    flood_level_layer_mp = None

                    arcpy.AddMessage("Processing input features: " + CommonLib.get_name_from_feature_class(input_features))


                    end_time = time.clock()
                    msg_body = create_msg_body("Create 3D Flood Leveles completed successfully.", start_time, end_time)

                else:
                    raise LicenseErrorSpatial
            else:
                raise LicenseError3D

            arcpy.ClearWorkspaceCache_management()

            msg(msg_body)

        except NoLayerFile:
            print("Can't find Layer file. Exiting...")
            arcpy.AddError("Can't find Layer file. Exiting...")

        except LicenseError3D:
            print("3D Analyst license is unavailable")
            arcpy.AddError("3D Analyst license is unavailable")

        except LicenseErrorSpatial:
            print("Spatial Analyst license is unavailable")
            arcpy.AddError("Spatial Analyst license is unavailable")

        except NoNoDataError:
            print("Input raster does not have NODATA values")
            arcpy.AddError("Input raster does not have NODATA values")

        except ValueError:
            print("Input no flood value is not a number.")
            arcpy.AddError("Input no flood value is not a number.")

        except arcpy.ExecuteError:
            line, filename, synerror = trace()
            msg("Error on %s" % line, ERROR)
            msg("Error in file name:  %s" % filename, ERROR)
            msg("With error message:  %s" % synerror, ERROR)
            msg("ArcPy Error Message:  %s" % arcpy.GetMessages(2), ERROR)

        except FunctionError as f_e:
            messages = f_e.args[0]
            msg("Error in function:  %s" % messages["function"], ERROR)
            msg("Error on %s" % messages["line"], ERROR)
            msg("Error in file name:  %s" % messages["filename"], ERROR)
            msg("With error message:  %s" % messages["synerror"], ERROR)
            msg("ArcPy Error Message:  %s" % messages["arc"], ERROR)

        except:
            line, filename, synerror = trace()
            msg("Error on %s" % line, ERROR)
            msg("Error in file name:  %s" % filename, ERROR)
            msg("with error message:  %s" % synerror, ERROR)

        finally:
            arcpy.CheckInExtension("3D")
            arcpy.CheckInExtension("Spatial")

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

            # check if input exists
            if arcpy.Exists(input_features):
                full_path_features= CommonLib.get_full_path_from_layer(input_features)
            else:
                raise NoLayerFile

            layer, layer2, layer3, layer4 = self.run(input_features=full_path_features, upper_elevation=upper_elevation, lower_elevation=lower_elevation,
                                                        invert_unit=invert_unit, diameter=diameter, diameter_unit=diameter_unit, default_diameter=default_diameter,
                                                        output_features=output_features, output_3dobjects=output_3dobjects, use_nearby_points=use_nearby_points,
                                                        zero_as_error=zero_as_error, error_elevation=error_elevation, interpolate_errors=interpolate_errors,
                                                        debug=0)

            if layer:
                arcpy.SetParameter(13, layer)
            else:
                raise NoOutput

            if layer2:
                arcpy.SetParameter(14, layer)
            else:
                raise NoOutput

            if layer3:
                arcpy.SetParameter(15, layer)
            else:
                raise NoOutput

            if layer4:
                arcpy.SetParameter(16, layer)
            else:
                raise NoOutput

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
        self.description = "Create3DLaterals"
        self.canRunInBackground = False

    def getParameterInfo(self):
        """Define parameter definitions"""
        params = None
        return params

    def isLicensed(self):
        """Set whether tool is licensed to execute."""
        return True

    def updateParameters(self, parameters):
        """Modify the values and properties of parameters before internal
        validation is performed.  This method is called whenever a parameter
        has been changed."""
        return

    def updateMessages(self, parameters):
        """Modify the messages created by internal validation for each tool
        parameter.  This method is called after internal validation."""
        return

    def execute(self, parameters, messages):
        """The source code of the tool."""
        return


# for debug only!
def main():
    tool = Create3DGravityMains()
    tool.run("", "", "", "", "", "", "", "", "", "", "", "", "", 1)

if __name__ == "__main__":
    main()
