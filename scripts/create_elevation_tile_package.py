#-------------------------------------------------------------------------------
# Name:        CreateElevationTilePackageForAGOL
# Purpose:
#
# Author:      Gert van Maren
#
# Created:     27/07/2016
# Copyright:   (c) Esri 2016
# updated:
# updated:
# updated:
#-------------------------------------------------------------------------------

import arcpy
import os
import sys
import shutil
import re
import time
import scripts.common_lib as common_lib
from scripts.common_lib import create_msg_body, msg, trace
from scripts.settings import *

class LicenseError3D(Exception):
    pass

class LicenseErrorSpatial(Exception):
    pass

class NoFeatures(Exception):
    pass

class No3DFeatures(Exception):
    pass


def getNameFromFeatureClass(feature_class):
    descFC = arcpy.Describe(feature_class)
    return(descFC.name)

# Get Workspace from Building feature class location
def getWorkSpaceFromFeatureClass(feature_class, get_gdb):
    dirname = os.path.dirname(arcpy.Describe(feature_class).catalogPath)
    desc = arcpy.Describe(dirname)

    if hasattr(desc, "datasetType") and desc.datasetType == 'FeatureDataset':
        dirname = os.path.dirname(dirname)

    if (get_gdb == "yes"):
        return(dirname)
    else:                   # directory where gdb lives
        return(os.path.dirname(dirname))

def GenerateLERCTilingScheme(input_layer, lc_scheme_dir, error):
    try:
        # variables
        method = "PREDEFINED"
        numscales = "#"
        predefScheme = lc_scheme_dir+"\\ArcGIS_Online_Bing_Maps_Google_Maps.xml"
        outputTilingScheme = lc_scheme_dir+"\\"+getNameFromFeatureClass(input_layer)+"_tiling_lerc.xml"
        scales = "#"
        scaleType = "#"
        tileOrigin = "#"
        dpi = "96"
        tileSize = "256 x 256"
        tileFormat = "LERC"
        compQuality = "75"
        storageFormat = "COMPACT"
        lerc_error = error

        if arcpy.Exists(predefScheme):
            arcpy.GenerateTileCacheTilingScheme_management(input_layer, outputTilingScheme, method, numscales, predefScheme,
                                                        scales, scaleType, tileOrigin, dpi, tileSize, tileFormat, compQuality, storageFormat, lerc_error)
        else:
            arcpy.AddWarning(
                "Can't find: " + predefScheme + ". Can't creat Tile Package. Exciting.")
            raise FileNotFoundError

        # return obstruction FC
        return (outputTilingScheme)

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


def ManageTileCache(input_layer, cache_directory, output_scheme, scale_level):
    try:
        # variables
        scales = [1128.497176,2256.994353,4513.988705,9027.977411,18055.954822,36111.909643,72223.819286,144447.638572,
                  288895.277144,577790.554289,1155581.108577,2311162.217155,4622324.434309,9244648.868618,18489297.737236,
                  36978595.474472,73957190.948944,147914381.897889,295828763.795777,591657527.591555]

        list_length = len(scales)

        folder = cache_directory
        mode = "RECREATE_ALL_TILES"
        cacheName = getNameFromFeatureClass(input_layer) + "_cache"
        dataSource = input_layer
        method = "IMPORT_SCHEME"
        tilingScheme = output_scheme
        scale_default = "#"
        areaofinterest = "#"
        maxcellsize = "#"
        maxcachedscale = str(scales[0])
        mincachedscale = str(scales[list_length - 1 - scale_level])

        #  check if directory is present
        if arcpy.Exists(folder+"\\"+cacheName):
            shutil.rmtree(folder+"\\"+cacheName)
            arcpy.AddMessage("Deleted old cache directory: "+folder+"\\"+cacheName)

        arcpy.AddMessage("Creating Tile Cache with "+str(list_length - scale_level)+" levels: L"+str(scale_level)+":"+mincachedscale+" down to L:"+str(list_length - 1)+":"+maxcachedscale)

        result = arcpy.ManageTileCache_management(
            folder, mode, cacheName, dataSource, method, tilingScheme,
            scale_default, areaofinterest, maxcellsize, mincachedscale, maxcachedscale)

        ##arcpy.AddMessage(result.status)

        # return obstruction FC
        return (folder+"\\"+cacheName)

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


def ExportTileCache(input_layer, cache_directory, tile_cache):
    try:
        cacheSource = tile_cache
        cacheFolder = cache_directory
        cachePackage = getNameFromFeatureClass(input_layer)
        cacheType = "TILE_PACKAGE"

        arcpy.AddMessage("Creating Tile Package: " + cacheFolder + "\\" +cachePackage+".tpk. This might take some time...")
        arcpy.GetMessages()
        arcpy.ExportTileCache_management(cacheSource, cacheFolder, cachePackage,
                                         cacheType)

        return (cacheFolder + "\\" + cachePackage)

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


def main(input_raster, minimum_scale_level, pixel_tolerance, output_ws, debug):
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
            log_directory = aprx.homeFolder + "\\Logs"
            scheme_directory = home_directory + "\TilingSchemes"
            project_ws = aprx.defaultGeodatabase

            enableLogging = True
            DeleteIntermediateData = True
            verbose = 0
            in_memory_switch = True
        else:
            # debug
            input_raster = r'D:\Gert\Work\Esri\Solutions\Utilities\work2.1\3DUtilities\Naperville.gdb\DEM_clip_feet'
            minimum_scale_level = str(12)
            pixel_tolerance = str(0.5)
            output_ws = r'D:\Gert\Work\Esri\Solutions\Utilities\work2.1\3DUtilities\Cache'

            # Create and set workspace location in same directory as input feature class gdb
            home_directory = r'D:\Gert\Work\Esri\Solutions\Utilities\work2.1\3DUtilities'
            scheme_directory = home_directory + "\TilingSchemes"
            project_ws = home_directory + "\\Results.gdb"
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

        if not os.path.exists(output_ws):
            os.makedirs(output_ws)

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

                lercError = float(re.sub(",", ".", pixel_tolerance))
                scaleLevel = re.sub(",", ".", minimum_scale_level)

                outputTilingScheme = GenerateLERCTilingScheme(input_raster, scheme_directory, lercError)
                arcpy.AddMessage("Created LERC Tiling Scheme with LERC error: " + str(lercError))

                tileCache = ManageTileCache(input_raster, output_ws, outputTilingScheme, int(scaleLevel))
                arcpy.AddMessage("Created Tile Cache...")

                arcpy.AddMessage("Exporting to Tile Package...")
                tilePackage = ExportTileCache(input_raster, output_ws, tileCache)

                return tilePackage

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
