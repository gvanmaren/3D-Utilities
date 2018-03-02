import arcpy
import time
import os
import scripts.common_lib as common_lib
from scripts.common_lib import create_msg_body, msg, trace
from scripts.settings import *


class FunctionError(Exception):
    pass

def create_hole_in_surface(local_sw, local_input_surface, local_input_features, local_depth, local_output_surface, local_verbose):

    if local_verbose == 1:
        msg("--------------------------")
        msg("Executing create_hole_in_surface...")

    start_time = time.clock()

    try:
        i = 0
        msg_prefix = ""
        failed = True

        # get extent of input features
        msg_body = create_msg_body("Creating extent polygon...", 0, 0)
        msg(msg_body)
        extent_poly = common_lib.get_extent_feature(local_sw, local_input_features)

        msg_body = create_msg_body("Clipping terrain...", 0, 0)
        msg(msg_body)

        # clip the input surface
        clipTerrain = local_sw + "\\terrain_clip"
        if arcpy.Exists(clipTerrain):
            arcpy.Delete_management(clipTerrain)

        # clip terrain to extent
        arcpy.Clip_management(local_input_surface, "#", clipTerrain, extent_poly)

        common_lib.get_name_from_feature_class(extent_poly)

        # subtract depth
        msg_body = create_msg_body("Creating hole...", 0, 0)
        msg(msg_body)
        depthTerrain = local_sw + "\\terrain_depth"
        if arcpy.Exists(depthTerrain):
            arcpy.Delete_management(depthTerrain)

        arcpy.Minus_3d(clipTerrain, local_depth, depthTerrain)

        # find IsNull values
        arcpy.env.extent = common_lib.get_full_path_from_layer(local_input_surface)

        outIsNull = os.path.join(local_sw, "outIsNull")
        if arcpy.Exists(outIsNull):
            arcpy.Delete_management(outIsNull)
        outIsNullRaster = arcpy.sa.IsNull(clipTerrain)
        outIsNullRaster.save(outIsNull)

        # mod the input surface.
        # Create modified raster
        if arcpy.Exists(local_output_surface):
            arcpy.Delete_management(local_output_surface)
        outConRaster = arcpy.sa.Con(outIsNull, common_lib.get_full_path_from_layer(local_input_surface), depthTerrain)
        outConRaster.save(local_output_surface)

        arcpy.ResetEnvironments()
        arcpy.env.workspace = local_sw
        arcpy.env.overwriteOutput = True

        msg_prefix = "Function create_hole_in_surface completed successfully."
        failed = False
        return extent_poly, local_output_surface

    except:
        line, filename, synerror = trace()
        failed = True
        msg_prefix = ""
        raise FunctionError(
            {
                "function": "create_hole_in_surface",
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


def main(input_raster, input_layer, depth, output_raster, debug):
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
            input_raster = r'D:\Gert\Work\Esri\Solutions\Utilities\work2.1\3DUtilities\Naperville.gdb\DEM_clip_feet'
            input_layer = r'D:\Gert\Work\Esri\Solutions\Utilities\work2.1\3DUtilities\Local_Scene.gdb\manHoles_test1'
            depth = 500
            output_raster = r'D:\Gert\Work\Esri\Solutions\Utilities\work2.1\3DUtilities\Local_Scene.gdb\surface_mod_test'

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

        scratch_ws = common_lib.create_gdb(home_directory, "Intermediate.gdb")

        arcpy.env.workspace = scratch_ws
        arcpy.env.overwriteOutput = True

        if arcpy.CheckExtension("3D") == "Available":
            arcpy.CheckOutExtension("3D")

            if arcpy.CheckExtension("Spatial") == "Available":
                arcpy.CheckOutExtension("Spatial")

                arcpy.AddMessage("Processing input raster: " + common_lib.get_name_from_feature_class(input_raster))

                # make a copy of the input feature class
                input_fc = os.path.join(scratch_ws, common_lib.get_name_from_feature_class(input_layer) + "_copy")
                if arcpy.Exists(input_fc):
                    arcpy.Delete_management(input_fc)

                # write to fc
                arcpy.AddMessage(
                    "Copying " + common_lib.get_name_from_feature_class(input_layer) + " to " + input_fc)
                arcpy.CopyFeatures_management(input_layer, input_fc)

                polygon, raster = create_hole_in_surface(scratch_ws, input_raster, input_fc, float(depth),
                                                         output_raster, verbose)

                # add polygon for bottom of hole with mulch texture
                SymbologyLayer = layer_directory + "\\hole_texture2.lyrx"

                if arcpy.Exists(SymbologyLayer):
                    output_layer = common_lib.get_name_from_feature_class(polygon)
                    arcpy.MakeFeatureLayer_management(polygon, output_layer)
                else:
                    msg_body = create_msg_body("Can't find" + SymbologyLayer + " in " + layer_directory, 0, 0)
                    msg(msg_body, WARNING)

                end_time = time.clock()
                msg_body = create_msg_body("create_usrface_hole completed successfully.", start_time, end_time)

                return raster, output_layer

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
    main("", "", "", "", 1)