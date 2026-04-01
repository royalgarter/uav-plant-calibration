const fs = require('fs');
const GeoTIFF = require('geotiff');

async function inspect(filePath) {
    const buffer = fs.readFileSync(filePath).buffer;
    const tiff = await GeoTIFF.fromArrayBuffer(buffer);
    const image = await tiff.getImage();
    
    console.log("File:", filePath);
    // console.log("File Directory:", image.fileDirectory);
    
    // Iterate over tags in fileDirectory
    for (const [key, value] of Object.entries(image.fileDirectory)) {
        // Tag keys are often numbers. 
        // 700 is XMP in standard TIFF often, but can vary.
        console.log(`Tag ${key}:`, typeof value === 'object' && value.length > 20 ? '[Large Data]' : value);
        
        // Check if value looks like XMP
        if (typeof value === 'string' && value.includes('<x:xmpmeta')) {
            console.log("FOUND XMP in Tag", key);
            console.log(value);
        }
        if (value instanceof Uint8Array || value instanceof Int8Array) {
             const str = new TextDecoder().decode(value);
             if (str.includes('<x:xmpmeta') || str.includes('RelativeOpticalCenter')) {
                 console.log(`FOUND XMP/Data in Tag ${key} (decoded)`);
                 console.log(str);
             }
        }
    }
}

inspect('example/input/DJI_0081.TIF').catch(console.error);
