/* * writemha.cpp * * Created on: Oct 7, 2015 * Author: pingge */

#include "writemha.h" 
#include <stdlib.h> 
#include <stdio.h> 
#include "volume.h" 
#include <string> 
void write_mha(char* filename, volume* vol)
{
    FILE* fp;
    const char* mha_header =
    "ObjectType = Image\n"
    "NDims = 3\n"
    "BinaryData = True\n"
    "BinaryDataByteOrderMSB = False\n"
    "TransformMatrix = %g %g %g %g %g %g %g %g %g\n"
    "Offset = %g %g %g\n"
    "CenterOfRotation = 0 0 0\n"
    "ElementSpacing = %g %g %g\n"
    "DimSize = %d %d %d\n"
    "AnatomicalOrientation = RAI\n"
    "%s"
    "ElementType = %s\n"
    "ElementDataFile = LOCAL\n";
    const char* element_type;

    fp = fopen (filename,"wb");
    if (!fp) {
        fprintf (stderr, "Can't open file %s for write\n", filename);
        return;
    }
    switch (vol->pix_type) {
        case PT_UCHAR:
            element_type = "MET_UCHAR";
            break;
        case PT_SHORT:
            element_type = "MET_SHORT";
            break;
        case PT_UINT32:
            element_type = "MET_UINT";
            break;
        case PT_FLOAT:
            element_type = "MET_FLOAT";
            break;
        case PT_VF_FLOAT_INTERLEAVED:
            element_type = "MET_FLOAT";
            break;
        case PT_VF_FLOAT_PLANAR:
        default:
            fprintf (stderr, "Unhandled type in write_mha().\n");
            exit (-1);
        }
        fprintf (fp, mha_header,
            vol->direction_cosines[0],
            vol->direction_cosines[3],
            vol->direction_cosines[6],
            vol->direction_cosines[1],
            vol->direction_cosines[4],
            vol->direction_cosines[7],
            vol->direction_cosines[2],
            vol->direction_cosines[5],
            vol->direction_cosines[8],
            vol->offset[0], vol->offset[1], vol->offset[2],
            vol->pix_spacing[0], vol->pix_spacing[1], vol->pix_spacing[2],
            vol->dim[0], vol->dim[1], vol->dim[2],
            (vol->pix_type == PT_VF_FLOAT_INTERLEAVED)
            ? "ElementNumberOfChannels = 3\n" : "",
            element_type);
        fflush (fp);s

    if (vol->pix_type == PT_VF_FLOAT_INTERLEAVED) {
        fwrite (vol->img, sizeof(float), 3 * vol->npix, fp);
    } else {
        fwrite (vol->img, vol->pix_size, vol->npix, fp);
    }
    fclose (fp);
}