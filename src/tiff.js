const fs = require('fs');
const GeoTIFF = require('geotiff');
const exif = require('exif-reader');

async function readTiff(filePath) {
  try {
    // Read the file into an ArrayBuffer
    const data = fs.readFileSync(filePath);
    const arrayBuffer = data.buffer;

    let metadata = exif(arrayBuffer);
    console.dir(metadata);

    // Parse the TIFF file
    const tiff = await GeoTIFF.fromArrayBuffer(arrayBuffer);

    // Get the first image (or loop through tiff.count() for multi-page files)
    const image = await tiff.getImage(0);

    // Access image data
    console.log('Width:', image.getWidth());
    console.log('Height:', image.getHeight());

    // Read raster data as an array (e.g., RGBA values)
    const raster = await image.readRasters();
    console.log('Raster data (first few pixels):', raster[0].slice(0, 10));

    // For GeoTIFFs, access geospatial metadata
    const geoMetadata = image.getGeoKeys();
    console.log('Geospatial metadata:', geoMetadata);

  } catch (error) {
    console.error('Error reading TIFF file:', error);
  }
}

const inputDir = `${__dirname}/../.data/input`;

let filenames = fs.readdirSync(inputDir);
for (let filename of filenames) {
  if (!filename.toLowerCase().includes('.tif')) continue;
  console.log('read:', filename);
  readTiff(`${inputDir}/${filename}`);
}
