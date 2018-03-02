import arcpy
import time
import os
import math
import scripts.common_lib as common_lib
from scripts.common_lib import create_msg_body, msg, trace
from scripts.settings import *


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
        input_fc = os.path.join(cl3D_ws, common_lib.get_name_from_feature_class(cl3D_laterals) + "_copy")
        if arcpy.Exists(input_fc):
            arcpy.Delete_management(input_fc)

        # write to fc
        arcpy.AddMessage("Copying " + common_lib.get_name_from_feature_class(cl3D_laterals) + " to " + input_fc)
        arcpy.CopyFeatures_management(cl3D_laterals, input_fc)

        # create 3D lines from 2D lines (note only end points of lines are used to created 3D lines!)
        LineStartPoints = os.path.join(cl3D_ws, "lateral_startpoints")
        if arcpy.Exists(LineStartPoints):
            arcpy.Delete_management(LineStartPoints)

        LineEndPoints = os.path.join(cl3D_ws, "lateral_endpoints")
        if arcpy.Exists(LineEndPoints):
            arcpy.Delete_management(LineEndPoints)

        arcpy.AddMessage("Extracting Start Points...")

        common_lib.delete_add_field(input_fc, DIAMETER_FIELD, "DOUBLE")

        # set diameter values
        if cl3D_diameter:
            if common_lib.check_fields(input_fc, [cl3D_diameter], True, cl3D_verbose) == 0:
                common_lib.set_null_or_negative_to_value_in_fields(input_fc, [cl3D_diameter],
                                                                   [cl3D_default_diameter], True, cl3D_verbose)
                arcpy.CalculateField_management(input_fc, DIAMETER_FIELD, "!" + cl3D_diameter + "!",
                                                "PYTHON_9.3")
            else:  # create a default attribute
                arcpy.CalculateField_management(input_fc, DIAMETER_FIELD, cl3D_default_diameter, "PYTHON_9.3")
        else:
            arcpy.CalculateField_management(input_fc, DIAMETER_FIELD, cl3D_default_diameter, "PYTHON_9.3")

        common_lib.delete_add_field(input_fc, SLOPE_FIELD, "DOUBLE")

        # set slope values
        if cl3D_slope:
            if common_lib.check_fields(input_fc, [cl3D_slope], True, cl3D_verbose) == 0:
                common_lib.set_null_or_negative_to_value_in_fields(input_fc, [cl3D_slope], [cl3D_default_slope],
                                                                   True, cl3D_verbose)
                arcpy.CalculateField_management(input_fc, SLOPE_FIELD, "!" + cl3D_slope + "!", "PYTHON_9.3")
            else:  # create a default attribute
                arcpy.CalculateField_management(input_fc, SLOPE_FIELD, cl3D_default_slope, "PYTHON_9.3")
        else:
            arcpy.CalculateField_management(input_fc, SLOPE_FIELD, cl3D_default_slope, "PYTHON_9.3")

        # get start and end points and set line order and elevation attribute
        arcpy.AddMessage("Calculating End Point elevations")
        arcpy.FeatureVerticesToPoints_management(input_fc, LineEndPoints, "END")
        common_lib.delete_add_field(LineEndPoints, elevation_field, elevation_fieldtype)
        arcpy.AddSurfaceInformation_3d(LineEndPoints, cl3D_tin, "Z", "BILINEAR")

        arcpy.CalculateField_management(LineEndPoints, elevation_field, "!Z!", "PYTHON_9.3", None)
        common_lib.set_null_to_value_in_fields(LineEndPoints, [elevation_field], [0], True, cl3D_verbose)

        common_lib.delete_add_field(LineEndPoints, line_field, line_fieldtype)
        arcpy.CalculateField_management(LineEndPoints, line_field, "2", "PYTHON_9.3", None)

        arcpy.AddMessage("Calculating Start Point elevations")
        arcpy.FeatureVerticesToPoints_management(input_fc, LineStartPoints, "START")
        common_lib.delete_add_field(LineStartPoints, elevation_field, elevation_fieldtype)

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

        common_lib.delete_add_field(LineStartPoints, line_field, line_fieldtype)
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

        mains_full_name = common_lib.get_full_path_from_layer(lc_3d_mains)

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


def main(input_layer, input_3d_mains_layer, diameter, diameter_unit, default_diameter,
                slope, default_slope, output_features, output_as_3dobject,
                debug):
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

        common_lib.set_data_paths_for_packaging(data_directory_pack, geodatabase, feature_class, model_directory_pack,
                                                model_file, rule_directory_pack, rule_file, layer_directory_pack,
                                                layer_file)

        esri_diameter_field = "esri_diameter"

        if not os.path.exists(tin_directory):
            os.makedirs(tin_directory)

        common_lib.set_up_logging(log_directory, TOOLNAME2)
        start_time = time.clock()

        scratch_ws = common_lib.create_gdb(home_directory, "Intermediate.gdb")
        output_ws = os.path.dirname(output_features)

        if arcpy.Exists(output_ws):
            arcpy.env.workspace = scratch_ws
            arcpy.env.overwriteOutput = True

            if arcpy.CheckExtension("3D") == "Available":
                arcpy.CheckOutExtension("3D")

                if arcpy.CheckExtension("Spatial") == "Available":
                    arcpy.CheckOutExtension("Spatial")

                    arcpy.AddMessage(
                        "Processing input features: " + common_lib.get_name_from_feature_class(input_layer))

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
                        layer_unit = common_lib.get_xy_unit(input_layer, verbose)

                        common_lib.delete_add_field(input_layer, esri_diameter_field, "DOUBLE")

                        if not diameter_unit:
                            diameter_unit = layer_unit
                            arcpy.AddMessage(
                                "No Diameter Unit detected. Using  XY units instead: " + diameter_unit)

                        if diameter:
                            expression = "!" + diameter + "! * " + str(
                                common_lib.unitConversion(layer_unit, diameter_unit, verbose))
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

                        Line3D_layer = common_lib.get_name_from_feature_class(Line3D)
                        arcpy.MakeFeatureLayer_management(Line3D, Line3D_layer)

                        if common_lib.get_z_unit(Line3D_layer, 0) == "Feet":
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

                            arcpy.AddMessage("Buffering: " + common_lib.get_name_from_feature_class(Line3D))
                            arcpy.AddMessage("This might take some time depending on the number of lines.")

                            common_lib.delete_add_field(Line3D, RADIUS_FIELD, "DOUBLE")
                            arcpy.CalculateField_management(Line3D, RADIUS_FIELD,
                                                            "!" + DIAMETER_FIELD + "! / 2",
                                                            "PYTHON_9.3")

                            arcpy.Buffer3D_3d(Line3D, objects3D, RADIUS_FIELD, 'Straight', '10')

                            objects3D_layer = common_lib.get_name_from_feature_class(objects3D)
                            arcpy.MakeFeatureLayer_management(objects3D, objects3D_layer)

                            if common_lib.get_z_unit(objects3D_layer, 0) == "Feet":
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
                            fcs = common_lib.listFcsInGDB(scratch_ws)

                            msg_prefix = "Deleting intermediate data..."

                            msg_body = common_lib.create_msg_body(msg_prefix, 0, 0)
                            common_lib.msg(msg_body)

                            for fc in fcs:
                                arcpy.Delete_management(fc)

                        arcpy.ClearWorkspaceCache_management()

                        end_time = time.clock()
                        msg_body = create_msg_body("Create 3D Laterals completed successfully.", start_time,
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


# for debug only!
if __name__ == "__main__":
    main("", "", "", "", "", "", "", "", "", 1)