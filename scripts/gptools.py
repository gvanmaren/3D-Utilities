import arcpy
import time
import os
import math
import sys
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
        layer2.symbology = os.path.join(layer_directory, 'Line3DError_Meters.lyrx')
        layer3.symbology = os.path.join(layer_directory, 'LineObject3DError.lyrx')
        layer4.symbology = os.path.join(layer_directory, 'LineObject3DError_Meters.lyrx')

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

    @staticmethod
    def run(input_layer, start_vertex_elevation, end_vertex_elevation,
                vertex_elevation_unit, diameter, diameter_unit, default_diameter,
                output_features, output_as_3dobject, use_nearby_points,
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

        class NoFeatures(Exception):
            pass

        # used functions
        def GetNearestElevValueForXY(cs, xy, compare_points, elev_attribute, lc_error_elevation, lc_zero_as_error):
            try:
                elev = error_elevation
                x_list = []
                y_list = []

                x, y = xy

                # get a list of all x and y coordinates, compare xy point with closest end or start points, see if it has an elevation attribute
                with arcpy.da.SearchCursor(compare_points, [elev_attribute, "SHAPE@XY"]) as f_cursor:
                    for f_row in f_cursor:
                        fx, fy = f_row[1]

                        if abs(fx - x) < 1 and abs(fy - y) < 1:
                            if f_row[0] is None:
                                elev = error_elevation
                            else:
                                if f_row[0] > 0:
                                    arcpy.AddMessage("Fixed value...")
                                elev = f_row[0]
                            break

                if elev == 0 and lc_zero_as_error:
                    elev = lc_error_elevation

                return elev

            except arcpy.ExecuteWarning:
                print((arcpy.GetMessages(1)))
                arcpy.AddWarning(arcpy.GetMessages(1))

            except arcpy.ExecuteError:
                print((arcpy.GetMessages(2)))
                arcpy.AddError(arcpy.GetMessages(2))

            # Return any other type of error
            except:
                # By default any other errors will be caught here
                #
                e = sys.exc_info()[1]
                print((e.args[0]))
                arcpy.AddError(e.args[0])

        def interpolate_3Dvalues_along_2Dline(workspace, input_features, upper_elevation_field, lower_elevation_field,
                                              diameter_field,
                                              lc_elevation_field, line_id_field, line_order_field, error_value, zero_as_error):
            try:
                # calculate correct start / end vertex elevations from invert elevation attribute. error elevation is set for Null values and zero if user input
                with arcpy.da.UpdateCursor(input_features,
                                           [upper_elevation_field, lower_elevation_field, diameter_field]) as u_cursor:
                    for u_row in u_cursor:
                        if u_row[0] is not None:
                            if u_row[0] == 0 and zero_as_error:
                                u_row[0] = error_value  # zero == error: set error elevation on attribute
                            else:
                                u_row[0] = u_row[0] + u_row[2] / 2  # adjust for invert elevation
                        else:
                            u_row[0] = error_value  # Null: set error elevation

                        if u_row[1] is not None:
                            if u_row[1] == 0 and zero_as_error:  # zero == error: set error elevation on attribute
                                u_row[1] = error_value
                            else:
                                u_row[1] = u_row[1] + u_row[2] / 2  # adjust for invert elevation
                        else:
                            u_row[1] = error_value  # Null: set error elevation

                        u_cursor.updateRow(u_row)

                # For each line feature: get points, get measurement, interpolate elevation based on start and end elevation and set point order
                LinePoints = os.path.join(workspace, "line_2Dpoints")
                if arcpy.Exists(LinePoints):
                    arcpy.Delete_management(LinePoints)

                sr = arcpy.Describe(input_features).spatialReference
                lineOID = arcpy.Describe(input_features).OIDFieldName
                lineOID_field = line_id_field

                # copy line OBJECTID to a new field
                arcpy.AddField_management(input_features, lineOID_field, "LONG")
                arcpy.CalculateField_management(input_features, lineOID_field, "!" + lineOID + "!", "PYTHON_9.3")

                flds_in = ("SHAPE@", lineOID_field, upper_elevation_field, lower_elevation_field)
                fld_Number = line_order_field  # point number from start point
                fld_Z = lc_elevation_field  # Elevation
                fld_Chainage = "Chainage"  # Distance m from start of polyline

                # create the output featureclass
                geometry_type = "POINT"
                template = ""
                has_m = "DISABLED"  # you could enable M values...
                has_z = "ENABLED"
                ws_path, fc_out_name = os.path.split(LinePoints)
                arcpy.CreateFeatureclass_management(workspace, fc_out_name, geometry_type, template, has_m, has_z, sr)

                # add the fields to the point featureclass
                arcpy.AddField_management(LinePoints, lineOID_field, "LONG")
                arcpy.AddField_management(LinePoints, fld_Number, "LONG")
                arcpy.AddField_management(LinePoints, fld_Z, "DOUBLE")
                arcpy.AddField_management(LinePoints, fld_Chainage, "DOUBLE")

                # fields for insert cursor on output points
                flds_out = ("SHAPE@", lineOID_field, fld_Number, fld_Z, fld_Chainage)

                arcpy.AddMessage("Interpolating elevation values for vertices along line segments...")

                # start insert cursor for output points
                with arcpy.da.InsertCursor(LinePoints, flds_out) as curs_out:
                    # start search cursor on lines
                    with arcpy.da.SearchCursor(input_features, flds_in) as curs:
                        for row in curs:
                            number = 0
                            polyline = row[0]
                            line_ID = row[1]
                            for part in polyline:
                                for pnt in part:
                                    number += 1
                                    if pnt:
                                        ptGeom = arcpy.PointGeometry(pnt, sr)
                                        line_length = polyline.length
                                        chainage = polyline.measureOnLine(ptGeom)

                                        # we assume that the start elevation is the UPPER elevation
                                        if chainage == 0:  # start point
                                            if row[2] == error_value:
                                                elevation = error_value
                                            else:
                                                elevation = row[2]
                                        elif chainage - line_length == 0:  # end point
                                            if row[3] == error_value:
                                                elevation = error_value
                                            else:
                                                elevation = row[3]
                                        else:  # in between points
                                            if row[2] == error_value or row[3] == error_value:
                                                elevation = error_value
                                            else:
                                                elevation_delta = (row[2] - row[3])
                                                distance_percentage = chainage / line_length
                                                elevation = row[2] - (elevation_delta * distance_percentage)

                                        curs_out.insertRow((ptGeom, line_ID, number, elevation, chainage))

                return LinePoints

            except arcpy.ExecuteWarning:
                print((arcpy.GetMessages(1)))
                arcpy.AddWarning(arcpy.GetMessages(1))

            except arcpy.ExecuteError:
                print((arcpy.GetMessages(2)))
                arcpy.AddError(arcpy.GetMessages(2))

            # Return any other type of error
            except:
                # By default any other errors will be caught here
                #
                e = sys.exc_info()[1]
                print((e.args[0]))
                arcpy.AddError(e.args[0])

        def GetAttributeRange(local_input_features, attribute):
            try:
                # cycle through features, get minimum and maximum value
                # create a list of unique "Attribute" values
                unique_field_values = CommonLib.unique_values(local_input_features, attribute)

                return [unique_field_values[0], unique_field_values[len(unique_field_values) - 1]]

            except arcpy.ExecuteWarning:
                print((arcpy.GetMessages(1)))
                arcpy.AddWarning(arcpy.GetMessages(1))

            except arcpy.ExecuteError:
                print((arcpy.GetMessages(2)))
                arcpy.AddError(arcpy.GetMessages(2))

            # Return any other type of error
            except:
                # By default any other errors will be caught here
                #
                e = sys.exc_info()[1]
                print((e.args[0]))
                arcpy.AddError(e.args[0])

        def GetUnitVector(v):
            # Normalize a vector.
            # This input vector is not expected to be normalized but the output vector is.
            # Both input and output vectors' XYZ components are contained in tuples.

            magnitude = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
            x = v[0] / magnitude
            y = v[1] / magnitude
            z = v[2] / magnitude
            return x, y, z

        def GetDistance(v1, v2):
            distance = math.sqrt(
                math.pow((v1[0] - v2[0]), 2) + math.pow((v1[1] - v1[1]), 2) + math.pow((v1[2] - v1[2]), 2))

            return distance

        def GetSlope(vect1, vect2):

            uv1 = GetUnitVector(vect1)
            uv2 = GetUnitVector(vect2)

            dist_a = GetDistance(uv1, uv2)
            dist_o = uv1[2] - uv2[2]

            if dist_o > 0:
                slope = math.degrees(math.sin(dist_o / dist_a))
            else:
                slope = 0

            return slope

        def Create3DlineFromLineAttributes(out_ws, ws, out_name, tin_ws, input_fc, upper_invert_elevation_field,
                                           lower_invert_elevation_field, lc_diameter, lc_default_diameter,
                                           lc_use_nearby_points, zero_as_error, error_elevation, lc_interpolate_errors, verbose):

            try:
                lineOID_field = "line_objectid"
                line_order_field = "line_order"  # point number from start point
                elevation_field = "elevation"
                start_elevation_field = "upper_line_elevation"
                end_elevation_field = "lower_line_elevation"
                line_fieldtype = "SHORT"
                elevation_fieldtype = "DOUBLE"
                field_list = ["elevation"]
                error_field = "error"

                # create 3D lines from 2D lines
                arcpy.AddMessage("Extracting Line Points...")

                # set all diameter values on input fc
                # check if diameter attribute exists

                CommonLib.delete_add_field(input_fc, DIAMETER_FIELD, "DOUBLE")

                if lc_diameter:
                    if CommonLib.check_fields(input_fc, [lc_diameter], False, verbose) == 0:
                        arcpy.CalculateField_management(input_fc, DIAMETER_FIELD, "!" + lc_diameter + "!", "PYTHON_9.3")
                        CommonLib.set_null_or_negative_to_value_in_fields(input_fc, [DIAMETER_FIELD],
                                                                          [lc_default_diameter],
                                                                          True, verbose)
                    else:  # create a default attribute
                        arcpy.CalculateField_management(input_fc, DIAMETER_FIELD, lc_default_diameter, "PYTHON_9.3")
                else:
                    arcpy.CalculateField_management(input_fc, DIAMETER_FIELD, lc_default_diameter, "PYTHON_9.3")

                # copy upper and lower elevation attributes so we can modify them
                CommonLib.delete_add_field(input_fc, start_elevation_field, "DOUBLE")
                arcpy.CalculateField_management(input_fc, start_elevation_field,
                                                "!" + upper_invert_elevation_field + "!",
                                                "PYTHON_9.3")
                CommonLib.delete_add_field(input_fc, end_elevation_field, "DOUBLE")
                arcpy.CalculateField_management(input_fc, end_elevation_field, "!" + lower_invert_elevation_field + "!",
                                                "PYTHON_9.3")

                Points2D_interpolated = interpolate_3Dvalues_along_2Dline(ws, input_fc, start_elevation_field,
                                                                          end_elevation_field, DIAMETER_FIELD,
                                                                          elevation_field, lineOID_field,
                                                                          line_order_field,
                                                                          error_elevation, zero_as_error)

                # use elevation surface through good point to interpolate bad values
                if lc_interpolate_errors:
                    Z_field = "Z"
                    surface = CommonLib.create_surface_from_points(ws, tin_ws, Points2D_interpolated, elevation_field,
                                                                   error_elevation)

                    if surface:
                        arcpy.AddSurfaceInformation_3d(Points2D_interpolated, surface, Z_field, "BILINEAR", 1, 1, 0,
                                                       None)
                    else:
                        raise NoFeatures

                    with arcpy.da.UpdateCursor(Points2D_interpolated, [elevation_field, Z_field]) as cursor:
                        for row in cursor:
                            if row[1]:
                                if zero_as_error:
                                    if row[0] == 0 or row[0] == error_elevation:
                                        row[0] = row[1]
                                else:
                                    if row[0] == error_elevation:
                                        row[0] = row[1]

                            cursor.updateRow(row)

                # create 3D points
                points3D = os.path.join(ws, "points_3D")
                if arcpy.Exists(points3D):
                    arcpy.Delete_management(points3D)

                arcpy.FeatureTo3DByAttribute_3d(Points2D_interpolated, points3D, elevation_field)

                # create 3D lines
                lines3D = os.path.join(output_ws, out_name + "_3Dlines", )
                if arcpy.Exists(lines3D):
                    arcpy.Delete_management(lines3D)

                arcpy.AddMessage("Joining original attributes...")
                arcpy.PointsToLine_management(points3D, lines3D, lineOID_field, line_order_field)
                arcpy.JoinField_management(lines3D, lineOID_field, input_fc, lineOID_field)

                # calculate error field
                CommonLib.delete_add_field(lines3D, error_field, line_fieldtype)
                arcpy.AddMessage("Calculating errors ...")

                s = 0

                z_property = "Z_MAX"
                arcpy.AddZInformation_3d(lines3D, z_property)

                with arcpy.da.UpdateCursor(lines3D,
                                           [start_elevation_field, end_elevation_field, error_field,
                                            z_property]) as cursor:
                    for row in cursor:
                        if zero_as_error:  # if zero is error
                            if row[0] == error_elevation or row[1] == error_elevation:  # we have a error value
                                if abs(row[3]) == error_elevation:
                                    row[2] = int(1)  # NULL values set to user error elevation
                                else:
                                    row[2] = int(2)  # fixed it earlier
                            else:
                                row[2] = int(0)
                        else:
                            if row[0] == error_elevation or row[1] == error_elevation:
                                if abs(row[3]) == error_elevation:
                                    row[2] = int(1)  # NULL values set to user error elevation
                                else:
                                    row[2] = int(2)  # fixed it earlier
                            else:
                                row[2] = int(0)

                        cursor.updateRow(row)
                        s += 1

                # cleaning up
                CommonLib.delete_fields(input_fc, [start_elevation_field, end_elevation_field])

                return lines3D

            except arcpy.ExecuteWarning:
                print((arcpy.GetMessages(1)))
                arcpy.AddWarning(arcpy.GetMessages(1))

            except arcpy.ExecuteError:
                print((arcpy.GetMessages(2)))
                arcpy.AddError(arcpy.GetMessages(2))

            # Return any other type of error
            except:
                # By default any other errors will be caught here
                #
                e = sys.exc_info()[1]
                print((e.args[0]))
                arcpy.AddError(e.args[0])

        try:
            # Get Attributes from User
            if debug == 0:
                # script variables
                aprx = arcpy.mp.ArcGISProject("CURRENT")
                home_directory = aprx.homeFolder
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
                input_layer = r'D:\Gert\Work\Esri\Solutions\Utilities\work2.1\3DUtilities\Local_Scene.gdb\mains_2d_test1'
                start_vertex_elevation = "UPELEV"
                end_vertex_elevation = "DOWNELEV"
                vertex_elevation_unit = "Feet"
                diameter = "DIAMETER"
                diameter_unit = "Inches"
                default_diameter = 3
                output_features = r'D:\Gert\Work\Esri\Solutions\Utilities\work2.1\3DUtilities\Local_Scene.gdb\mains_2d_test3D_1'
                output_as_3dobject = True
                use_nearby_points = True
                zero_as_error = True
                error_elevation = 1000
                interpolate_errors = True

                # Create and set workspace location in same directory as input feature class gdb
                home_directory = r'D:\Gert\Work\Esri\Solutions\Utilities\work2.1\3DUtilities'
                rule_directory = home_directory + "\RulePackages"
                layer_directory = home_directory + "\LayerFiles"
                project_ws = home_directory + "\\Results.gdb"
                tin_directory = home_directory + "\TINs"
                scripts_directory = home_directory + "\\Scripts"
                log_directory = home_directory + "\\Logs"
                project_ws = home_directory + "\\Results.gdb"

                enableLogging = False
                DeleteIntermediateData = True
                verbose = 1
                in_memory_switch = False

            # set data paths for packing tool so all additional data is stored in the package - ESRI packing only!
            data_directory_pack = ""
            geodatabase = ""
            feature_class = ""
            model_directory_pack = ""
            model_file = ""
            rule_directory_pack = "RulePackages"
            rule_file = "ExtrudePolygon.rpk"
            # note: rename all *.lyrx to *.txt first. This is only needed for packaging.
            layer_directory_pack = "LayerFiles"
            layer_file = "Line3DError.lyrx"

            CommonLib.set_data_paths_for_packaging(data_directory_pack, geodatabase, feature_class, model_directory_pack,
                                                   model_file, rule_directory_pack, rule_file, layer_directory_pack, layer_file)

            if not os.path.exists(tin_directory):
                os.makedirs(tin_directory)

            CommonLib.set_up_logging(log_directory, TOOLNAME1)
            start_time = time.clock()

            ORIG_FID = "ORIG_FID"
            avg_height_field = "avg_height"
            unique_id_field = "unique_id"
            esri_upper_elevation_field = "esri_upper_elev"
            esri_lower_elevation_field = "esri_lower_elev"
            esri_diameter_field = "esri_diameter"
            slope_field = "calc_slope"
            z_field = "Z"

            scratch_ws = CommonLib.create_gdb(home_directory, "Intermediate.gdb")
            output_ws = os.path.dirname(output_features)

            if arcpy.Exists(output_ws):
                arcpy.env.workspace = scratch_ws
                arcpy.env.overwriteOutput = True

                if arcpy.CheckExtension("3D") == "Available":
                    arcpy.CheckOutExtension("3D")

                    if arcpy.CheckExtension("Spatial") == "Available":
                        arcpy.CheckOutExtension("Spatial")

                        arcpy.AddMessage("Processing input features: " + CommonLib.get_name_from_feature_class(input_layer))

                        objects3D = None
                        objects3D_layer = None
                        Line3D = None
                        Line3D_layer = None

                        # make a copy of the input feature class
                        input_fc = os.path.join(scratch_ws,
                                                CommonLib.get_name_from_feature_class(input_layer) + "_copy")
                        if arcpy.Exists(input_fc):
                            arcpy.Delete_management(input_fc)

                        # write to fc
                        arcpy.AddMessage(
                            "Copying " + CommonLib.get_name_from_feature_class(input_layer) + " to " + input_fc)
                        arcpy.CopyFeatures_management(input_layer, input_fc)

                        # just because of this schema lock
                        input_layer = input_fc

                        # create 3D line
                        zValues = arcpy.Describe(input_layer).hasZ

                        arcpy.AddMessage("Creating 3D lines...")

                        # check for output directory
                        if not os.path.exists(tin_directory):
                            os.makedirs(tin_directory)

                        # create unique ObjectID attribute
                        lineOID = arcpy.Describe(input_layer).OIDFieldName
                        arcpy.AddField_management(input_layer, unique_id_field, "LONG")
                        arcpy.CalculateField_management(input_layer, unique_id_field, "!" + lineOID + "!", "PYTHON_9.3")

                        # create start and end elevation attributes in segment elevation units
                        layer_unit = CommonLib.get_xy_unit(input_layer, verbose)

                        CommonLib.delete_add_field(input_layer, esri_upper_elevation_field, "DOUBLE")
                        CommonLib.delete_add_field(input_layer, esri_lower_elevation_field, "DOUBLE")

                        if not vertex_elevation_unit:
                            vertex_elevation_unit = layer_unit
                            arcpy.AddMessage(
                                "No invert elevation unit detected. Using  XY units instead: " + vertex_elevation_unit)

                        conversion_factor = CommonLib.unitConversion(layer_unit, vertex_elevation_unit, verbose)
                        CommonLib.calculate_field_from_other_field(input_layer, input_fc, start_vertex_elevation,
                                                                   esri_upper_elevation_field,
                                                                   "multiply", conversion_factor, verbose)
                        CommonLib.calculate_field_from_other_field(input_layer, input_fc, end_vertex_elevation,
                                                                   esri_lower_elevation_field,
                                                                   "multiply", conversion_factor, verbose)

                        # check if error elevation is larger than max elevation in the data
                        maxValue = arcpy.SearchCursor(input_layer, "", "", "",
                                                      esri_upper_elevation_field + " D").next().getValue(
                            esri_upper_elevation_field)  # Get 1st row in ascending cursor sort

                        if maxValue > error_elevation:
                            error_elevation += maxValue
                            arcpy.AddMessage(
                                "Maximum value of " + start_vertex_elevation + " attribute is larger than the error elevation value")
                            arcpy.AddMessage("Setting the error elevation value to: " + str(error_elevation))

                        # create diameter attribute in segment elevation units
                        CommonLib.delete_add_field(input_layer, esri_diameter_field, "DOUBLE")

                        if not diameter_unit:
                            diameter_unit = layer_unit
                            arcpy.AddMessage("No Diameter Unit detected. Using  XY units instead: " + diameter_unit)

                        if diameter:
                            conversion_factor = CommonLib.unitConversion(layer_unit, diameter_unit, verbose)
                            CommonLib.calculate_field_from_other_field(input_layer, input_fc, diameter,
                                                                       esri_diameter_field,
                                                                       "multiply", conversion_factor, verbose)
                        else:
                            arcpy.CalculateField_management(input_layer, esri_diameter_field, default_diameter,
                                                            "PYTHON_9.3")

                        output_name = str(os.path.basename(output_features))
                        Line3D = Create3DlineFromLineAttributes(output_ws, scratch_ws, output_name, tin_directory,
                                                                input_layer, esri_upper_elevation_field,
                                                                esri_lower_elevation_field, esri_diameter_field,
                                                                default_diameter, use_nearby_points,
                                                                zero_as_error, error_elevation,
                                                                interpolate_errors, debug)

                        Line3D_layer = CommonLib.get_name_from_feature_class(Line3D)
                        arcpy.MakeFeatureLayer_management(Line3D, Line3D_layer)

                        if CommonLib.get_z_unit(Line3D_layer, 0) == "Feet":
                            SymbologyLayer = layer_directory + "\\Line3DError.lyrx"
                        else:
                            SymbologyLayer = layer_directory + "\\Line3DError_Meters.lyrx"

                        if not arcpy.Exists(SymbologyLayer):
                            arcpy.AddWarning("Can't find: " + SymbologyLayer + ". Symbolize features by error attribute to see data errors.")

                        # convert 3D Points to 3D objects
                        if output_as_3dobject:
                            objects3D = os.path.join(output_ws, output_name + "_3Dobjects")
                            if arcpy.Exists(objects3D):
                                arcpy.Delete_management(objects3D)

                            # we must remove self intersections
                            # Check out extension

                            arcpy.AddMessage("Checking for self intersections (OGC Validation)...")
                            arcpy.RepairGeometry_management(Line3D, "#", "OGC")

                            arcpy.AddMessage("Buffering: " + CommonLib.get_name_from_feature_class(Line3D))
                            arcpy.AddMessage("This might take some time depending on the number of lines.")

                            CommonLib.delete_add_field(Line3D, RADIUS_FIELD, "DOUBLE")
                            arcpy.CalculateField_management(Line3D, RADIUS_FIELD, "!" + DIAMETER_FIELD + "! / 2",
                                                            "PYTHON_9.3")

                            arcpy.Buffer3D_3d(Line3D, objects3D, RADIUS_FIELD, 'Straight', '10')

                            objects3D_layer = CommonLib.get_name_from_feature_class(objects3D)
                            arcpy.MakeFeatureLayer_management(objects3D, objects3D_layer)

                            if CommonLib.get_z_unit(objects3D_layer, 0) == "Feet":
                                SymbologyLayer = layer_directory + "\\LineObject3DError.lyrx"
                            else:
                                SymbologyLayer = layer_directory + "\\LineObject3DError_Meters.lyrx"

                            if not arcpy.Exists(SymbologyLayer):
                                arcpy.AddWarning("Can't find: " + SymbologyLayer + ". Symbolize features by error attribute to see data errors.")

                            # check if any of the lines failed buffering
                            org_line_ids_line = CommonLib.get_row_values_for_fields(None, Line3D, [unique_id_field],
                                                                                    None, "no_expression")
                            org_line_ids_object = CommonLib.get_row_values_for_fields(None, objects3D,
                                                                                      [unique_id_field], None,
                                                                                      "no_expression")

                            difference = list(set(org_line_ids_line) - set(org_line_ids_object))

                            if len(difference) > 0:
                                arcpy.AddWarning("Buffering failed for lines with the following OBJECTIDs: " + str(
                                    difference) + " Check geometries!")

                        if DeleteIntermediateData:
                            fcs = CommonLib.listFcsInGDB(scratch_ws)

                            msg_prefix = "Deleting intermediate data..."

                            msg_body = CommonLib.create_msg_body(msg_prefix, 0, 0)
                            CommonLib.msg(msg_body)

                            for fc in fcs:
                                arcpy.Delete_management(fc)

                        arcpy.ClearWorkspaceCache_management()

                        end_time = time.clock()
                        msg_body = create_msg_body("Create 3D Gravity Mains completed successfully.", start_time, end_time)
                        msg(msg_body)

                        return Line3D_layer, objects3D_layer

                    else:
                        raise LicenseErrorSpatial
                else:
                    raise LicenseError3D

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

            if diameter_unit == UNDEFINED:
                diameter_unit = None

            # check if input exists
            if arcpy.Exists(parameters[0].value):
                lines_3d, objects_3d = self.run(input_layer=parameters[0].value, start_vertex_elevation=upper_elevation, end_vertex_elevation=lower_elevation,
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
                    arcpy.AddMessage("Adding: " + CommonLib.get_name_from_feature_class(lines_3d))

                    if CommonLib.get_z_unit(lines_3d, 0) == "Feet":
                        arcpy.SetParameter(13, lines_3d)
                    else:
                        arcpy.SetParameter(14, lines_3d)

                    if objects_3d:
                        if CommonLib.get_z_unit(objects_3d, 0) == "Feet":
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

    @staticmethod
    def run(input_layer, input_3d_mains_layer, diameter, diameter_unit, default_diameter,
                slope, default_slope, output_features, output_as_3dobject,
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

        class NoFeatures(Exception):
            pass

        class No3DFeatures(Exception):
            pass

        def calculateStartZPointfromSlope(local_start_points, local_end_points, local_elevation_field, local_sort_field,
                                          local_slope_field, local_verbose):

            if local_verbose == 1:
                msg("--------------------------")
                msg("Executing calculateStartZPointfromSlope...")

            start_time = time.clock()

            try:
                i = 0
                msg_prefix = ""
                failed = True

                # step through start points
                with arcpy.da.UpdateCursor(local_start_points, [local_sort_field, local_elevation_field, "SHAPE@XY",
                                                                local_slope_field]) as cursor:
                    for row in cursor:
                        line_id_start = row[0]

                        with arcpy.da.SearchCursor(local_end_points,
                                                   [local_sort_field, local_elevation_field, "SHAPE@XY"]) as f_cursor:
                            for f_row in f_cursor:  # find the accompanying end point and get Z
                                if line_id_start == f_row[0]:  # we have the same line
                                    z_end = f_row[1]  # we have the end Z

                                    sx, sy = row[2]
                                    ex, ey = f_row[2]

                                    # get distance between the points
                                    distance = math.hypot((sx - ex), (sy - ey))

                                    # calculate Z difference based on slope and distance
                                    slope_radians = math.radians(row[3])
                                    tan_value = math.tan(slope_radians)
                                    Z_diff = tan_value * distance

                                    row[1] = z_end + Z_diff

                                    cursor.updateRow(row)

                                    break

                msg_prefix = "Function calculateZStartPointfromSlope completed successfully."
                failed = False
                return 0

            except:
                line, filename, synerror = trace()
                failed = True
                msg_prefix = ""
                raise FunctionError(
                    {
                        "function": "calculateStartZPointfromSlope",
                        "line": line,
                        "filename": filename,
                        "synerror": synerror,
                        "arc": str(arcpy.GetMessages(2))
                    }
                )

            finally:
                end_time = time.clock()
                msg_body = create_msg_body(msg_prefix, start_time, end_time)
                if failed:
                    msg(msg_body, ERROR)
                else:
                    if local_verbose == 1:
                        msg(msg_body)
                    pass

        def create_laterals3DfromTIN(cl3D_output_ws, cl3D_ws, cl3D_tin, cl3D_laterals, cl3D_diameter,
                                     cl3D_default_diameter, cl3D_slope, cl3D_default_slope, cl3D_building_fp,
                                     cl3D_outname, cl3D_verbose):

            if cl3D_verbose == 1:
                msg("--------------------------")
                msg("Executing create_laterals3D...")

            start_time = time.clock()

            try:
                i = 0
                msg_prefix = ""
                failed = True

                line_field = "line_order"
                elevation_field = "elevation"
                start_elevation_field = "start_elevation"
                end_elevation_field = "end_elevation"
                line_fieldtype = "SHORT"
                elevation_fieldtype = "DOUBLE"
                field_list = ["elevation"]
                sort_field = "ORIG_FID"

                # make a copy of the input feature class
                input_fc = os.path.join(cl3D_ws, CommonLib.get_name_from_feature_class(cl3D_laterals) + "_copy")
                if arcpy.Exists(input_fc):
                    arcpy.Delete_management(input_fc)

                # write to fc
                arcpy.AddMessage("Copying " + CommonLib.get_name_from_feature_class(cl3D_laterals) + " to " + input_fc)
                arcpy.CopyFeatures_management(cl3D_laterals, input_fc)

                # create 3D lines from 2D lines (note only end points of lines are used to created 3D lines!)
                LineStartPoints = os.path.join(cl3D_ws, "lateral_startpoints")
                if arcpy.Exists(LineStartPoints):
                    arcpy.Delete_management(LineStartPoints)

                LineEndPoints = os.path.join(cl3D_ws, "lateral_endpoints")
                if arcpy.Exists(LineEndPoints):
                    arcpy.Delete_management(LineEndPoints)

                arcpy.AddMessage("Extracting Start Points...")

                CommonLib.delete_add_field(input_fc, DIAMETER_FIELD, "DOUBLE")

                # set diameter values
                if cl3D_diameter:
                    if CommonLib.check_fields(input_fc, [cl3D_diameter], True, cl3D_verbose) == 0:
                        CommonLib.set_null_or_negative_to_value_in_fields(input_fc, [cl3D_diameter],
                                                                          [cl3D_default_diameter], True, cl3D_verbose)
                        arcpy.CalculateField_management(input_fc, DIAMETER_FIELD, "!" + cl3D_diameter + "!",
                                                        "PYTHON_9.3")
                    else:  # create a default attribute
                        arcpy.CalculateField_management(input_fc, DIAMETER_FIELD, cl3D_default_diameter, "PYTHON_9.3")
                else:
                    arcpy.CalculateField_management(input_fc, DIAMETER_FIELD, cl3D_default_diameter, "PYTHON_9.3")

                CommonLib.delete_add_field(input_fc, SLOPE_FIELD, "DOUBLE")

                # set slope values
                if cl3D_slope:
                    if CommonLib.check_fields(input_fc, [cl3D_slope], True, cl3D_verbose) == 0:
                        CommonLib.set_null_or_negative_to_value_in_fields(input_fc, [cl3D_slope], [cl3D_default_slope],
                                                                          True, cl3D_verbose)
                        arcpy.CalculateField_management(input_fc, SLOPE_FIELD, "!" + cl3D_slope + "!", "PYTHON_9.3")
                    else:  # create a default attribute
                        arcpy.CalculateField_management(input_fc, SLOPE_FIELD, cl3D_default_slope, "PYTHON_9.3")
                else:
                    arcpy.CalculateField_management(input_fc, SLOPE_FIELD, cl3D_default_slope, "PYTHON_9.3")

                # get start and end points and set line order and elevation attribute
                arcpy.AddMessage("Calculating End Point elevations")
                arcpy.FeatureVerticesToPoints_management(input_fc, LineEndPoints, "END")
                CommonLib.delete_add_field(LineEndPoints, elevation_field, elevation_fieldtype)
                arcpy.AddSurfaceInformation_3d(LineEndPoints, cl3D_tin, "Z", "BILINEAR")

                arcpy.CalculateField_management(LineEndPoints, elevation_field, "!Z!", "PYTHON_9.3", None)
                CommonLib.set_null_to_value_in_fields(LineEndPoints, [elevation_field], [0], True, cl3D_verbose)

                CommonLib.delete_add_field(LineEndPoints, line_field, line_fieldtype)
                arcpy.CalculateField_management(LineEndPoints, line_field, "2", "PYTHON_9.3", None)

                arcpy.AddMessage("Calculating Start Point elevations")
                arcpy.FeatureVerticesToPoints_management(input_fc, LineStartPoints, "START")
                CommonLib.delete_add_field(LineStartPoints, elevation_field, elevation_fieldtype)

                # join slope field based on sort_field
                arcpy.JoinField_management(LineStartPoints, sort_field, input_fc, arcpy.Describe(input_fc).OIDFieldName,
                                           [SLOPE_FIELD])

                # if building footprints use these to find the start elevation, else we use the slope variables
                if cl3D_building_fp:
                    arcpy.CalculateField_management(LineStartPoints, elevation_field, "!Z!", "PYTHON_9.3", None)
                    arcpy.AddSurfaceInformation_3d(LineStartPoints, cl3D_tin, "Z", "BILINEAR")
                else:
                    calculateStartZPointfromSlope(LineStartPoints, LineEndPoints, elevation_field, sort_field,
                                                  SLOPE_FIELD, cl3D_verbose)

                CommonLib.delete_add_field(LineStartPoints, line_field, line_fieldtype)
                arcpy.CalculateField_management(LineStartPoints, line_field, "1", "PYTHON_9.3", None)

                # merge start and end points
                merged_fc = os.path.join(cl3D_ws, "merged_lateral_points")
                if arcpy.Exists(merged_fc):
                    arcpy.Delete_management(merged_fc)

                arcpy.Merge_management([LineStartPoints, LineEndPoints], merged_fc)

                # create 3D points
                points3D = os.path.join(cl3D_ws, "lateral_points_3D")
                if arcpy.Exists(points3D):
                    arcpy.Delete_management(points3D)

                arcpy.FeatureTo3DByAttribute_3d(merged_fc, points3D, elevation_field)

                # create 3D lines
                lines3D = os.path.join(cl3D_output_ws, cl3D_outname + "_3Dlines", )
                if arcpy.Exists(lines3D):
                    arcpy.Delete_management(lines3D)

                arcpy.AddMessage("Joining original attributes...")
                arcpy.PointsToLine_management(points3D, lines3D, sort_field, line_field)
                join_field = arcpy.Describe(input_fc).OIDFieldName
                arcpy.JoinField_management(lines3D, sort_field, input_fc, join_field)

                msg_prefix = "Function create_laterals3D completed successfully."
                failed = False

                return lines3D

            except:
                line, filename, synerror = trace()
                failed = True
                msg_prefix = ""
                raise FunctionError(
                    {
                        "function": "create_laterals3D",
                        "line": line,
                        "filename": filename,
                        "synerror": synerror,
                        "arc": str(arcpy.GetMessages(2))
                    }
                )

            finally:
                end_time = time.clock()
                msg_body = create_msg_body(msg_prefix, start_time, end_time)
                if failed:
                    msg(msg_body, ERROR)
                else:
                    if cl3D_verbose == 1:
                        msg(msg_body)
                    pass

        def create_laterals(out_ws, ws, tin_ws, lc_laterals, lc_3d_mains, lc_building_fp, lc_diameter,
                            lc_default_diameter, lc_slope, lc_default_slope, lc_outputname, local_verbose):

            if local_verbose == 1:
                msg("--------------------------")
                msg("Executing create_laterals...")

            start_time = time.clock()

            try:
                i = 0
                msg_prefix = ""
                failed = True

                mains_full_name = CommonLib.get_full_path_from_layer(lc_3d_mains)

                if lc_building_fp:
                    tin_string = "{} Shape.Z Hard_Line <None>;{} Shape.Z  Hardvalue_Fill <None>".format(mains_full_name,
                                                                                                        lc_building_fp)
                else:
                    tin_string = "{} Shape.Z Hard_Line <None>".format(mains_full_name)

                out_tin = os.path.join(tin_ws, "LateralTin")
                if arcpy.Exists(out_tin):
                    arcpy.Delete_management(out_tin)

                arcpy.CreateTin_3d(out_tin, arcpy.Describe(lc_laterals).spatialReference, tin_string, "DELAUNAY")

                # create 3D Lines
                Line3D = create_laterals3DfromTIN(out_ws, ws, out_tin, lc_laterals, lc_diameter, lc_default_diameter,
                                                  lc_slope, lc_default_slope, lc_building_fp, lc_outputname,
                                                  local_verbose)

                msg_prefix = "Function create_laterals completed successfully."
                failed = False
                return Line3D

            except:
                line, filename, synerror = trace()
                failed = True
                msg_prefix = ""
                raise FunctionError(
                    {
                        "function": "create_laterals",
                        "line": line,
                        "filename": filename,
                        "synerror": synerror,
                        "arc": str(arcpy.GetMessages(2))
                    }
                )

            finally:
                end_time = time.clock()
                msg_body = create_msg_body(msg_prefix, start_time, end_time)
                if failed:
                    msg(msg_body, ERROR)
                else:
                    if local_verbose == 1:
                        msg(msg_body)
                    pass

        try:
            # Get Attributes from User
            if debug == 0:
                # script variables
                aprx = arcpy.mp.ArcGISProject("CURRENT")
                home_directory = aprx.homeFolder
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
                input_layer = r'D:\Gert\Work\Esri\Solutions\Utilities\work2.1\3DUtilities\Local_Scene.gdb\lateral_test1'
                input_3d_mains_layer = r'D:\Gert\Work\Esri\Solutions\Utilities\work2.1\3DUtilities\Local_Scene.gdb\mains_3d_test1'
                diameter = "DIAMETER"
                diameter_unit = "Inches"
                default_diameter = 3
                slope = "Slope"
                default_slope = 45
                output_features = r'D:\Gert\Work\Esri\Solutions\Utilities\work2.1\3DUtilities\Local_Scene.gdb\lateral_test1_3D'
                output_as_3dobject = True

                # Create and set workspace location in same directory as input feature class gdb
                home_directory = r'D:\Gert\Work\Esri\Solutions\Utilities\work2.1\3DUtilities'
                rule_directory = home_directory + "\RulePackages"
                layer_directory = home_directory + "\LayerFiles"
                project_ws = home_directory + "\\Results.gdb"
                tin_directory = home_directory + "\TINs"
                scripts_directory = home_directory + "\\Scripts"
                log_directory = home_directory + "\\Logs"
                project_ws = home_directory + "\\Results.gdb"

                enableLogging = False
                DeleteIntermediateData = True
                verbose = 1
                in_memory_switch = False

            # set data paths for packing tool so all additional data is stored in the package - ESRI packing only!
            data_directory_pack = ""
            geodatabase = ""
            feature_class = ""
            model_directory_pack = ""
            model_file = ""
            rule_directory_pack = "RulePackages"
            rule_file = "ExtrudePolygon.rpk"
            # note: rename all *.lyrx to *.txt first. This is only needed for packaging.
            layer_directory_pack = "LayerFiles"
            layer_file = "Line3DError.lyrx"

            CommonLib.set_data_paths_for_packaging(data_directory_pack, geodatabase, feature_class, model_directory_pack,
                                                   model_file, rule_directory_pack, rule_file, layer_directory_pack,
                                                   layer_file)

            esri_diameter_field = "esri_diameter"

            if not os.path.exists(tin_directory):
                os.makedirs(tin_directory)

            CommonLib.set_up_logging(log_directory, TOOLNAME2)
            start_time = time.clock()

            scratch_ws = CommonLib.create_gdb(home_directory, "Intermediate.gdb")
            output_ws = os.path.dirname(output_features)

            if arcpy.Exists(output_ws):
                arcpy.env.workspace = scratch_ws
                arcpy.env.overwriteOutput = True

                if arcpy.CheckExtension("3D") == "Available":
                    arcpy.CheckOutExtension("3D")

                    if arcpy.CheckExtension("Spatial") == "Available":
                        arcpy.CheckOutExtension("Spatial")

                        arcpy.AddMessage("Processing input features: " + CommonLib.get_name_from_feature_class(input_layer))

                        objects3D = None
                        objects3D_layer = None
                        Line3D = None
                        Line3D_layer = None

                        # check if line feature class has Z values. If not generate 3D line from 2D line using attributes
                        line_zValues = arcpy.Describe(input_3d_mains_layer).hasZ

                        if line_zValues:
                            input_building_fp = None

                            arcpy.AddMessage("Creating 3D laterals...")

                            output_name = str(os.path.basename(output_features))

                            # create diameter attribute in segment elevation units
                            layer_unit = CommonLib.get_xy_unit(input_layer, verbose)

                            CommonLib.delete_add_field(input_layer, esri_diameter_field, "DOUBLE")

                            if not diameter_unit:
                                diameter_unit = layer_unit
                                arcpy.AddMessage(
                                    "No Diameter Unit detected. Using  XY units instead: " + diameter_unit)

                            if diameter:
                                expression = "!" + diameter + "! * " + str(
                                    CommonLib.unitConversion(layer_unit, diameter_unit, verbose))
                                arcpy.CalculateField_management(input_layer, esri_diameter_field, expression,
                                                                "PYTHON_9.3")
                            else:
                                arcpy.CalculateField_management(input_layer, esri_diameter_field,
                                                                default_diameter,
                                                                "PYTHON_9.3")

                            Line3D = create_laterals(output_ws, scratch_ws, tin_directory, input_layer,
                                                     input_3d_mains_layer, input_building_fp, esri_diameter_field,
                                                     default_diameter,
                                                     slope, default_slope, output_name, verbose)

                            Line3D_layer = CommonLib.get_name_from_feature_class(Line3D)
                            arcpy.MakeFeatureLayer_management(Line3D, Line3D_layer)

                            if CommonLib.get_z_unit(Line3D_layer, 0) == "Feet":
                                SymbologyLayer = layer_directory + "\\LateralLine3D.lyrx"
                            else:
                                SymbologyLayer = layer_directory + "\\LateralLine3D_Meters.lyrx"

                            if not arcpy.Exists(SymbologyLayer):
                                arcpy.AddWarning(
                                    "Can't find: " + SymbologyLayer + ". Symbolize features by error attribute to see data errors.")

                            if output_as_3dobject:
                                objects3D = os.path.join(output_ws, output_name + "_3Dobjects")
                                if arcpy.Exists(objects3D):
                                    arcpy.Delete_management(objects3D)

                                arcpy.AddMessage("Buffering: " + CommonLib.get_name_from_feature_class(Line3D))
                                arcpy.AddMessage("This might take some time depending on the number of lines.")

                                CommonLib.delete_add_field(Line3D, RADIUS_FIELD, "DOUBLE")
                                arcpy.CalculateField_management(Line3D, RADIUS_FIELD,
                                                                "!" + DIAMETER_FIELD + "! / 2",
                                                                "PYTHON_9.3")

                                arcpy.Buffer3D_3d(Line3D, objects3D, RADIUS_FIELD, 'Straight', '10')

                                objects3D_layer = CommonLib.get_name_from_feature_class(objects3D)
                                arcpy.MakeFeatureLayer_management(objects3D, objects3D_layer)

                                if CommonLib.get_z_unit(objects3D_layer, 0) == "Feet":
                                    SymbologyLayer = layer_directory + "\\LateralObject3D.lyrx"
                                else:
                                    SymbologyLayer = layer_directory + "\\LateralObject3D_meter.lyrx"

                                if not arcpy.Exists(SymbologyLayer):
                                    arcpy.AddWarning(
                                        "Can't find: " + SymbologyLayer + ". Symbolize features by error attribute to see data errors.")

                            end_time = time.clock()
                            msg_body = create_msg_body("Create Laterals completed successfully.", start_time,
                                                       end_time)

                            if DeleteIntermediateData:
                                fcs = CommonLib.listFcsInGDB(scratch_ws)

                                msg_prefix = "Deleting intermediate data..."

                                msg_body = CommonLib.create_msg_body(msg_prefix, 0, 0)
                                CommonLib.msg(msg_body)

                                for fc in fcs:
                                    arcpy.Delete_management(fc)

                            arcpy.ClearWorkspaceCache_management()

                            end_time = time.clock()
                            msg_body = create_msg_body("Create 3D Gravity Mains completed successfully.", start_time,
                                                       end_time)
                            msg(msg_body)

                            return Line3D_layer, objects3D_layer
                        else:
                            raise No3DFeatures
                    else:
                        raise LicenseErrorSpatial
                else:
                    raise LicenseError3D

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
            (input_features, input_3Dmains, diameter, diameter_unit, default_diameter, slope, default_slope,
                output_features, output_3dobjects) = [p.valueAsText for p in parameters[:-4]]

            if diameter_unit == UNDEFINED:
                diameter_unit = None

            # check if input exists
            if arcpy.Exists(parameters[0].value):
                lines_3d, objects_3d = self.run(input_layer=parameters[0].value,
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
                    arcpy.AddMessage("Adding: " + CommonLib.get_name_from_feature_class(lines_3d))

                    if CommonLib.get_z_unit(lines_3d, 0) == "Feet":
                        arcpy.SetParameter(9, lines_3d)
                    else:
                        arcpy.SetParameter(10, lines_3d)

                    if objects_3d:
                        if CommonLib.get_z_unit(objects_3d, 0) == "Feet":
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


# for debug only!
def main():
    tool = Create3DLaterals()
    tool.run("", "", "", "", "", "", "", "", "", 1)

if __name__ == "__main__":
    main()
