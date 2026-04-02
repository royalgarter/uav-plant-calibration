#!/usr/bin/env node
const fisheye = require('bindings')('fisheye');

module.exports = fisheye;

if (require.main === module) {
  const fs = require('fs');
  const path = require('path');
  const { promisify } = require('util');

  const args = process.argv.slice(2);
  if (args.length !== 5) {
    console.log('Usage: fisheye <src_image> <dest_image> <samples_dir> <checkboard_width> <checkboard_height>');
    process.exit(1);
  }

  const [src, dest, samplesDir, width, height] = args;

  (async () => {
    try {
      const files = fs.readdirSync(samplesDir)
        .filter(f => f.match(/\.(jpg|jpeg|png|webp)$/i))
        .map(f => path.join(samplesDir, f));

      if (files.length === 0) {
        throw new Error(`No images found in ${samplesDir}`);
      }

      console.log(`Loading ${files.length} samples from ${samplesDir}...`);
      const imgs = await Promise.all(files.map(f => promisify(fs.readFile)(f)));
      
      console.log('Calibrating...');
      const { K, D } = fisheye.calibrate(imgs, parseInt(width), parseInt(height));
      
      console.log(`Undistorting ${src}...`);
      const originImg = await promisify(fs.readFile)(src);
      const undistorted = fisheye.undistort(originImg, K, D, {
        extname: path.extname(src),
        scale: 1
      });
      
      await promisify(fs.writeFile)(dest, undistorted);
      console.log(`Saved to ${dest}`);
    } catch (err) {
      console.error('Error:', err.message);
      process.exit(1);
    }
  })();
}

