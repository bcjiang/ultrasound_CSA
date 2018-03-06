/* * readmha.cpp * * Created on: Mar 15, 2016 * Author: pinggejiang */

#include "readmha.h" 
#include <stdio.h> 
#include <stdlib.h> 
#include <string.h> 
#include <ctype.h> 
#include "volume.h" 
volume* read_mha(const char* filename)
{
    size_t rc;
    char linebuf[512];
    volume* vol = new volume();
    int tmp;
    FILE* fp;
    unsigned int a, b, c;
    fp = fopen(filename, "rb");
    fprintf(stdout, "reading %s\n", filename);
    vol->pix_size = -1;
    vol->pix_type = PT_UNDEFINED;
    while (fgets(linebuf, 512, fp)){
        string_util_rtrim_whitespace(linebuf);
        if (strcmp (linebuf, "ElementDataFile = LOCAL") == 0) {
            break;
        }
        if (sscanf (linebuf, "DimSize = %d %d %d", &a, &b, &c) == 3) {
            vol->dim[0] = a;
            vol->dim[1] = b;
            vol->dim[2] = c;
            vol->npix = vol->dim[0] * vol->dim[1] * vol->dim[2];
            continue;
        }
        if (sscanf (linebuf, "Offset = %g %g %g",
            &vol->offset[0],
            &vol->offset[1],
            &vol->offset[2]) == 3) {
            continue;
        }
        if (sscanf (linebuf, "ElementSpacing = %g %g %g",
            &vol->pix_spacing[0],
            &vol->pix_spacing[1],
            &vol->pix_spacing[2]) == 3) {
            continue;
        }
        if (sscanf (linebuf, "ElementNumberOfChannels = %d", &tmp) == 1) {
            if (vol->pix_type == PT_UNDEFINED || vol->pix_type == PT_FLOAT) {
                vol->pix_type = PT_VF_FLOAT_INTERLEAVED;
                vol->pix_size = 3*sizeof(float);
            }
            continue;
        }
        if (sscanf(linebuf, "TransformMatrix =%g %g %g %g %g %g %g %g %g",
            &vol->direction_cosines[0], &vol->direction_cosines[3], &vol->direction_cosines[6],
            &vol->direction_cosines[1], &vol->direction_cosines[4], &vol->direction_cosines[7],
            &vol->direction_cosines[2], &vol->direction_cosines[5], &vol->direction_cosines[8])==9)
        {
            continue;
        }
        if (strcmp (linebuf, "ElementType = MET_FLOAT") == 0) {
            if (vol->pix_type == PT_UNDEFINED) {
                vol->pix_type = PT_FLOAT;
                vol->pix_size = sizeof(float);
            }
            continue;
        }
        if (strcmp (linebuf, "ElementType = MET_SHORT") == 0) {
            vol->pix_type = PT_SHORT;
            vol->pix_size = sizeof(short);
            continue;
        }
        if (strcmp (linebuf, "ElementType = MET_UCHAR") == 0) {
            vol->pix_type = PT_UCHAR;
            vol->pix_size = sizeof(unsigned char);
            continue;
        }
    }

    if (vol->pix_size <= 0) {
        printf ("Oops, couldn't interpret mha data type\n");
        exit (-1);
    }
    vol->img = malloc (vol->pix_size*vol->npix);
    if (!vol->img) {
        printf ("Oops, out of memory\n");
        exit (-1);
    }

    rc = fread (vol->img, vol->pix_size, vol->npix, fp);
    if (rc != (size_t) vol->npix) {
        printf ("Oops, bad read from file (%u)\n", (unsigned int) rc);
        exit (-1);
    }

    fclose (fp);
    return vol;
}

void
string_util_rtrim_whitespace (char *s)
{
    int len = (int)strlen (s);
    while (len > 0 && isspace(s[len-1])) {
        s[len-1] = 0;
        len--;
    }
}