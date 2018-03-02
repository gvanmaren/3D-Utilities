import arcpy
import time
import os
import scripts.common_lib as common_lib
from scripts.common_lib import create_msg_body, msg, trace
from scripts.settings import *

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


def Create3DPointFromPointAttributes(out_ws, ws, out_name, tin_ws, lc_input_layer,
                                     upper_rim_elevation_field, lower_invert_elevation_field,
                                     export_invert_elevation_field,
                                     lc_diameter, lc_default_diameter, export_diameter_field,
                                     export_height_field,
                                     dtm,
                                     error_elevation, lc_interpolate_errors, lc_zero_error, local_verbose):
    if local_verbose == 1:
        msg("--------------------------")
        msg("Executing Create3DPointFromPointAttributes...")

    start_time = time.clock()

    try:
        i = 0
        msg_prefix = ""
        failed = True
        rimelev_field = "util_rimelev"
        error_field = "error"
        point_fieldtype = "SHORT"

        # set all diameter values
        # check if diameter attribute exists
        if lc_diameter:
            if common_lib.check_fields(lc_input_layer, [lc_diameter], False, local_verbose) == 0:
                common_lib.set_null_or_negative_to_value_in_fields(lc_input_layer, [lc_diameter],
                                                                   [lc_default_diameter], True, local_verbose)
                common_lib.delete_add_field(lc_input_layer, export_diameter_field, "DOUBLE")
                arcpy.CalculateField_management(lc_input_layer, export_diameter_field, "!" + lc_diameter + "!",
                                                "PYTHON_9.3")
            else:  # create a default attribute
                common_lib.delete_add_field(lc_input_layer, export_diameter_field, "DOUBLE")
                arcpy.CalculateField_management(lc_input_layer, export_diameter_field, lc_default_diameter,
                                                "PYTHON_9.3")
                lc_diameter = export_diameter_field
        else:
            common_lib.delete_add_field(lc_input_layer, export_diameter_field, "DOUBLE")
            arcpy.CalculateField_management(lc_input_layer, export_diameter_field, lc_default_diameter,
                                            "PYTHON_9.3")
            lc_diameter = export_diameter_field

        if common_lib.get_xy_unit(lc_input_layer, local_verbose) == "Feet":
            conv_factor = 1
        else:
            conv_factor = 0.3048

        min_depth = conv_factor * 1
        max_depth = conv_factor * 100

        # copy attributes to default util ones
        common_lib.delete_add_field(lc_input_layer, export_invert_elevation_field, "DOUBLE")
        common_lib.delete_add_field(lc_input_layer, rimelev_field, "DOUBLE")
        common_lib.delete_add_field(lc_input_layer, export_height_field, "DOUBLE")

        arcpy.CalculateField_management(lc_input_layer, export_invert_elevation_field,
                                        "!" + lower_invert_elevation_field + "!", "PYTHON_9.3")
        arcpy.CalculateField_management(lc_input_layer, rimelev_field, "!" + upper_rim_elevation_field + "!",
                                        "PYTHON_9.3")

        # create surface from good values
        if lc_interpolate_errors:
            Z_field = "Z"
            invertZ_field = "invertZ"

            # interpolate invert elevations
            surface = common_lib.create_surface_from_points(ws, tin_ws, lc_input_layer,
                                                            export_invert_elevation_field, error_elevation)

            if surface:
                arcpy.AddSurfaceInformation_3d(lc_input_layer, surface, Z_field, "BILINEAR", 1, 1, 0, None)
                common_lib.delete_add_field(lc_input_layer, invertZ_field, "DOUBLE")
                arcpy.CalculateField_management(lc_input_layer, invertZ_field, "!" + Z_field + "!",
                                                "PYTHON_9.3")

                # interpolate rim elevations
                if arcpy.Exists(dtm):
                    common_lib.delete_fields(lc_input_layer, [Z_field])
                    arcpy.AddSurfaceInformation_3d(lc_input_layer, dtm, Z_field, "BILINEAR", 1, 1, 0, None)

                # check invert and rim elevation values
                with arcpy.da.UpdateCursor(lc_input_layer,
                                           [export_invert_elevation_field, invertZ_field, rimelev_field,
                                            Z_field]) as cursor:
                    for row in cursor:
                        if lc_zero_error:
                            if row[0] is None or row[0] == 0 or row[
                                0] == error_elevation:  # error with invert elevation
                                if row[1]:
                                    row[0] = row[1]
                                else:
                                    row[0] = error_elevation
                            if row[2] is None or row[2] == 0 or row[
                                2] == error_elevation:  # error with rim elevation
                                if row[3]:
                                    row[2] = row[3]
                                else:
                                    row[2] = error_elevation
                        else:
                            if row[0] is None or row[0] == error_elevation:  # error with invert elevation
                                if row[1]:
                                    row[0] = row[1]
                                else:
                                    row[0] = error_elevation
                            if row[2] is None or row[2] == error_elevation:  # error with rim elevation
                                if row[3]:
                                    row[2] = row[3]
                                else:
                                    row[2] = error_elevation

                        cursor.updateRow(row)
            else:
                arcpy.AddWarning("Can't interpolate values; not enough good points to create surface.")

        # recalculate NULL values to error value
        arcpy.AddMessage("Recalculating NULL values to " + str(error_elevation))

        s = 0

        with arcpy.da.UpdateCursor(lc_input_layer, [export_invert_elevation_field, rimelev_field,
                                                    export_height_field]) as cursor:
            for row in cursor:
                # set invert attribute
                if row[0] is None:
                    row[0] = int(error_elevation)
                else:
                    if lc_zero_error:
                        if row[0] == 0:
                            row[0] = int(error_elevation)

                # set rim attribute
                if row[1] is None:
                    row[1] = int(error_elevation)
                else:
                    if lc_zero_error:
                        if row[1] == 0:
                            row[1] = int(error_elevation)

                # set height attribute
                if row[0] and row[1]:
                    if row[1] > (row[0] + min_depth) and row[1] - row[0] < max_depth:
                        #                    if (row[0] + min_depth) < row[1] - row[0] < max_depth:       # assume max manhole depth is less than 100 and more than 1
                        if lc_zero_error:
                            if row[0] == 0 or row[1] == 0:
                                row[2] = error_elevation - row[0]
                            else:
                                row[2] = row[1] - row[0]
                        else:
                            row[2] = row[1] - row[0]
                    else:
                        row[2] = error_elevation - row[0]
                else:
                    row[2] = error_elevation

                cursor.updateRow(row)
                s += 1

        # create 3D points
        points3D = os.path.join(out_ws, out_name + "_3Dpoints")
        if arcpy.Exists(points3D):
            arcpy.Delete_management(points3D)

        arcpy.FeatureTo3DByAttribute_3d(lc_input_layer, points3D, export_invert_elevation_field)

        # calculate error field
        common_lib.delete_add_field(points3D, error_field, point_fieldtype)
        arcpy.AddMessage("Calculating errors ...")

        s = 0

        z_property = "Z"
        arcpy.AddZInformation_3d(points3D, z_property)

        # set error_field against original attributes
        with arcpy.da.UpdateCursor(points3D,
                                   [lower_invert_elevation_field, error_field, z_property, export_height_field,
                                    upper_rim_elevation_field, rimelev_field]) as cursor:
            for row in cursor:
                if lc_zero_error:  # if zero is error
                    if row[4] == 0 or row[4] is None:
                        if row[5] == error_elevation:
                            row[1] = int(1)
                        else:
                            row[1] = int(2)  # fixed it earlier
                    else:
                        if row[0] == 0 or row[0] is None:
                            if abs(row[2]) == error_elevation:
                                row[1] = int(1)  # NULL values set to user error elevation
                            else:
                                row[1] = int(2)  # fixed it earlier
                        else:
                            row[1] = int(0)
                else:
                    if row[4] is None:
                        if row[5] == error_elevation:
                            row[1] = int(1)
                        else:
                            row[1] = int(2)
                    else:
                        if row[0] is None:
                            if abs(row[2]) == error_elevation:
                                row[1] = int(1)  # NULL values set to user error elevation
                            else:
                                row[1] = int(2)  # fixed it earlier
                        else:
                            row[1] = int(0)

                if row[3] > max_depth:  # assume max manhole depth is less than 100 and larger than 1
                    row[1] = int(1)

                # height error
                #                if row[3] == error_elevation:
                #                    row[1] = int(1)

                cursor.updateRow(row)
                s += 1

        msg_prefix = "Create3DPointFromPointAttributes completed successfully."
        failed = False

        return points3D

    except:
        line, filename, synerror = trace()
        failed = True
        msg_prefix = ""
        raise FunctionError(
            {
                "function": "Create3DPointFromPointAttributes",
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


def main(input_layer, rim_elevation, invert_elevation,
                vertex_elevation_unit, diameter, diameter_unit, default_diameter,
                output_features, output_as_3dobject,
                zero_as_error, error_elevation, interpolate_errors, terrain_surface,
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
            input_layer = r'D:\Gert\Work\Esri\Solutions\Utilities\work2.1\3DUtilities\Local_Scene.gdb\manHoles_test1'
            rim_elevation = "RIMELEV"
            invert_elevation = "INVERTELEV"
            vertex_elevation_unit = "Feet"
            diameter = "diameter"
            diameter_unit = "Inches"
            default_diameter = 1
            output_features = r'D:\Gert\Work\Esri\Solutions\Utilities\work2.1\3DUtilities\Local_Scene.gdb\manHoles3D_test1'
            output_as_3dobject = True
            zero_as_error = True
            error_elevation = 1000
            interpolate_errors = True
            terrain_surface = r'D:\Gert\Work\Esri\Solutions\Utilities\work2.1\3DUtilities\Naperville.gdb\DEM_clip_feet'

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

        if not os.path.exists(tin_directory):
            os.makedirs(tin_directory)

        common_lib.set_up_logging(log_directory, TOOLNAME3)
        start_time = time.clock()

        esri_upper_elevation_field = "esri_upper_elev"
        esri_lower_elevation_field = "esri_lower_elev"
        esri_diameter_field = "esri_diameter"

        extrude_rpk = rule_directory + "\\ExtrudePolygon.rpk"

        scratch_ws = common_lib.create_gdb(home_directory, "Intermediate.gdb")
        output_ws = os.path.dirname(output_features)

        arcpy.env.workspace = scratch_ws
        arcpy.env.overwriteOutput = True

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
                    Points3D = None
                    Points3D_layer = None

                    # create 3D points
                    # check if point feature class has Z values. If not generate 3D points from 2D points using attributes
                    zValues = arcpy.Describe(input_layer).hasZ

                    # make a copy of the input feature class
                    input_fc = os.path.join(scratch_ws, common_lib.get_name_from_feature_class(input_layer) + "_copy")
                    if arcpy.Exists(input_fc):
                        arcpy.Delete_management(input_fc)

                    # write to fc
                    arcpy.AddMessage(
                        "Copying " + common_lib.get_name_from_feature_class(input_layer) + " to " + input_fc)
                    arcpy.CopyFeatures_management(input_layer, input_fc)

                    # just because of this schema lock
                    input_layer = input_fc

                    arcpy.AddMessage("Creating 3D points...")

                    # check for output directory
                    if not os.path.exists(tin_directory):
                        os.makedirs(tin_directory)

                    # create start and end elevation attributes in segment elevation units
                    layer_unit = common_lib.get_xy_unit(input_layer, verbose)

                    common_lib.delete_add_field(input_layer, esri_upper_elevation_field, "DOUBLE")
                    common_lib.delete_add_field(input_layer, esri_lower_elevation_field, "DOUBLE")

                    if not vertex_elevation_unit:
                        vertex_elevation_unit = layer_unit
                        arcpy.AddMessage(
                            "No invert elevation unit detected. Using  XY units instead: " + vertex_elevation_unit)

                    conversion_factor = common_lib.unitConversion(layer_unit, vertex_elevation_unit, verbose)
                    common_lib.calculate_field_from_other_field(input_layer, input_fc, rim_elevation,
                                                                esri_upper_elevation_field,
                                                                "multiply", conversion_factor, verbose)
                    common_lib.calculate_field_from_other_field(input_layer, input_fc, invert_elevation,
                                                                esri_lower_elevation_field,
                                                                "multiply", conversion_factor, verbose)

                    # check if error elevation is larger than max elevation in the data
                    maxValue = arcpy.SearchCursor(input_layer, "", "", "",
                                                  esri_upper_elevation_field + " D").next().getValue(
                        esri_upper_elevation_field)  # Get 1st row in ascending cursor sort

                    if maxValue > error_elevation:
                        error_elevation += maxValue
                        arcpy.AddMessage(
                            "Maximum value of " + rim_elevation + " attribute is larger than the error elevation value")
                        arcpy.AddMessage("Setting the error elevation value to: " + str(error_elevation))

                    # create diameter attribute in segment elevation units
                    common_lib.delete_add_field(input_layer, esri_diameter_field, "DOUBLE")

                    if not diameter_unit:
                        diameter_unit = layer_unit
                        arcpy.AddMessage("No Diameter Unit detected. Using  XY units instead: " + diameter_unit)

                    if diameter:
                        conversion_factor = common_lib.unitConversion(layer_unit, diameter_unit, verbose)
                        common_lib.calculate_field_from_other_field(input_layer, input_fc, diameter,
                                                                    esri_diameter_field,
                                                                    "multiply", conversion_factor, verbose)
                    else:
                        arcpy.CalculateField_management(input_layer, esri_diameter_field, default_diameter,
                                                        "PYTHON_9.3")

                    output_name = str(os.path.basename(output_features))

                    # if not zValues:
                    Points3D = Create3DPointFromPointAttributes(output_ws, scratch_ws, output_name, tin_directory,
                                                                input_layer,
                                                                esri_upper_elevation_field, esri_lower_elevation_field,
                                                                INVERTELEV_FIELD,
                                                                esri_diameter_field, default_diameter, DIAMETER_FIELD,
                                                                HEIGHT_FIELD,
                                                                terrain_surface,
                                                                error_elevation, interpolate_errors, zero_as_error,
                                                                verbose)

                    Points3D_layer = common_lib.get_name_from_feature_class(Points3D)
                    arcpy.MakeFeatureLayer_management(Points3D, Points3D_layer)

                    if common_lib.get_z_unit(Points3D_layer, 0) == "Feet":
                        SymbologyLayer = layer_directory + "\\Point3DError.lyrx"
                    else:
                        SymbologyLayer = layer_directory + "\\Point3DError_meters.lyrx"

                    if not arcpy.Exists(SymbologyLayer):
                        arcpy.AddWarning(
                            "Can't find: " + SymbologyLayer + ". Symbolize features by error attribute to see data errors.")

                    if output_as_3dobject:
                        objects3D = os.path.join(output_ws, output_name + "_3Dobjects")
                        if arcpy.Exists(objects3D):
                            arcpy.Delete_management(objects3D)

                        # convert 3D Points to 3D objects
                        arcpy.AddMessage("Buffering: " + common_lib.get_name_from_feature_class(Points3D))

                        common_lib.delete_add_field(Points3D, RADIUS_FIELD, "DOUBLE")
                        arcpy.CalculateField_management(Points3D, RADIUS_FIELD, "!" + DIAMETER_FIELD + "! / 2",
                                                        "PYTHON_9.3")

                        output3d_objects = common_lib.Point3DToObject(scratch_ws, extrude_rpk, Points3D,
                                                                      INVERTELEV_FIELD,
                                                                      RADIUS_FIELD, HEIGHT_FIELD, objects3D, verbose)

                        objects3D_layer = common_lib.get_name_from_feature_class(output3d_objects)
                        arcpy.MakeFeatureLayer_management(output3d_objects, objects3D_layer)

                        if common_lib.get_z_unit(objects3D_layer, 0) == "Feet":
                            SymbologyLayer = layer_directory + "\\PointObject3DError.lyrx"
                        else:
                            SymbologyLayer = layer_directory + "\\PointObject3DError_meter.lyrx"

                        if not arcpy.Exists(SymbologyLayer):
                            arcpy.AddWarning(
                                "Can't find: " + SymbologyLayer + ". Symbolize features by error attribute to see data errors.")

                    if DeleteIntermediateData:
                        fcs = common_lib.listFcsInGDB(scratch_ws)
                        arcpy.AddMessage("Deleting intermediate data...")

                        for fc in fcs:
                            arcpy.Delete_management(fc)

                    # here goes all the other if/else
                    end_time = time.clock()
                    msg_body = create_msg_body("PointTo3DManHole completed successfully.", start_time, end_time)
                    msg(msg_body)

                    return Points3D_layer, objects3D_layer

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
    main("", "", "", "", "", "", "", "", "", "", "", "", "", 1)