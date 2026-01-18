#include <tiffio.h>
#include <iostream>

void readTiffMetadata(const char* filename) {
    TIFF* tif = TIFFOpen(filename, "r");
    if (tif) {
        uint32 width, height;
        TIFFGetField(tif, TIFFTAG_IMAGEWIDTH, &width);
        TIFFGetField(tif, TIFFTAG_IMAGELENGTH, &height);
        std::cout << "Image Width: " << width << ", Height: " << height << std::endl;

        char* description;
        if (TIFFGetField(tif, TIFFTAG_IMAGEDESCRIPTION, &description)) {
            std::cout << "Description: " << description << std::endl;
        }

        TIFFClose(tif);
    } else {
        std::cerr << "Could not open TIFF file" << std::endl;
    }
}
