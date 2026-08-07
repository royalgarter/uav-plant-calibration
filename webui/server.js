const http = require('http');
const fs = require('fs');
const path = require('path');
const { spawn } = require('child_process');

const PORT = process.env.PORT || 11918;

const MIME_TYPES = {
	'.html': 'text/html',
	'.js': 'text/javascript',
	'.css': 'text/css',
	'.json': 'application/json',
	'.png': 'image/png',
	'.jpg': 'image/jpeg',
	'.jpeg': 'image/jpeg',
	'.tif': 'image/tiff',
	'.tiff': 'image/tiff',
	'.gif': 'image/gif',
	'.csv': 'text/csv',
	'.txt': 'text/plain',
};

const server = http.createServer((req, res) => {
	// Basic request logging
	// console.log(`${req.method} ${req.url}`);

	// Handle CLI command execution
	if (req.method === 'POST' && req.url === '/run') {
		let body = '';
		req.on('data', chunk => body += chunk.toString());
		req.on('end', () => {
			try {
				const config = JSON.parse(body);
				executeCalibration(config, res);
			} catch (error) {
				res.writeHead(400, { 'Content-Type': 'application/json' });
				res.end(JSON.stringify({ error: error.toString() }));
			}
		});
		return;
	}

	// Handle directory listing
	if (req.method === 'GET' && req.url.startsWith('/list?dir=')) {
		const urlParams = new URL(req.url, `http://${req.headers.host}`);
		const dir = urlParams.searchParams.get('dir') || '.output';

		// Security: prevent directory traversal
		const safePath = path.normalize(dir).replace(/^(\.\.[\/\\])+/, '');
		const absolutePath = path.join(process.cwd(), safePath);

		fs.readdir(absolutePath, { withFileTypes: true }, (err, files) => {
			if (err) {
				res.writeHead(500, { 'Content-Type': 'application/json' });
				res.end(JSON.stringify({ error: err.message }));
				return;
			}

			const list = files.map(file => ({
				name: file.name,
				isDirectory: file.isDirectory(),
				ext: path.extname(file.name).toLowerCase()
			}));

			res.writeHead(200, { 'Content-Type': 'application/json' });
			res.end(JSON.stringify(list));
		});
		return;
	}

	// Serve static files
	let filePath = req.url === '/' ? '/index.html' : req.url;


	// Security: prevent directory traversal
	let safePath = path.normalize(filePath).replace(/^(\.\.[\/\\])+/, '');
	let absolutePath = path.join(__dirname, safePath);
	let absolutePath2 = path.join(process.cwd(), safePath);

	if (!fs.existsSync(absolutePath) && fs.existsSync(absolutePath2)) absolutePath = absolutePath2;

	fs.readFile(absolutePath, (err, content) => {
		if (err) {
			if (err.code === 'ENOENT') {
				res.writeHead(404, { 'Content-Type': 'text/plain' });
				res.end('404 Not Found');
			} else {
				res.writeHead(500, { 'Content-Type': 'text/plain' });
				res.end(`Server Error: ${err.code}`);
			}
		} else {
			const ext = path.extname(absolutePath).toLowerCase();
			res.writeHead(200, { 'Content-Type': MIME_TYPES[ext] || 'application/octet-stream' });
			res.end(content);
		}
	});
});

function executeCalibration(config, res) {
	const {
		inDir = '.input',
		outDir = '.output',
		doRadio = false,
		autoRadioThickness = -1,
		radioRefFile = '',
		radioTemplatePath = '',
		greenCentroidRadius = '',
		preprocess = {},
		optimize = false,
		calc = {}
	} = config;

	// Preprocessing defaults mirror C++ GreenMaskParams defaults
	const pre = {
		textureThresh: 32.0,
		ndviThresh: 0.12,
		minAreaRatio: 0.0015,
		solidityThresh: 0.50,
		medianBlur: 5,
		openKernel: 7,
		closeKernel: 5,
		...preprocess
	};

	// Build arguments list for ./calib
	// Positional arguments first: inDir, outDir
	const args = [inDir, outDir];

	if (doRadio) {
		args.push('--radio');
		if (radioRefFile) {
			args.push('--ref', radioRefFile);
		}
	}

	if (autoRadioThickness >= 0) {
		args.push('--auto');
		if (autoRadioThickness > 0) {
			args.push(autoRadioThickness.toString());
		}
	}

	if (radioTemplatePath) {
		args.push('--template', radioTemplatePath);
	}

	if (optimize) {
		args.push('--optimize');
	}

	if (greenCentroidRadius) {
		args.push('--green-centroid-radius', greenCentroidRadius);
	}

	if (pre.textureThresh !== 32.0) args.push('--texture-thresh', String(pre.textureThresh));
	if (pre.ndviThresh !== 0.12) args.push('--ndvi-thresh', String(pre.ndviThresh));
	if (pre.minAreaRatio !== 0.0015) args.push('--min-area-ratio', String(pre.minAreaRatio));
	if (pre.solidityThresh !== 0.50) args.push('--solidity-thresh', String(pre.solidityThresh));
	if (pre.medianBlur !== 5) args.push('--median-blur', String(pre.medianBlur));
	if (pre.openKernel !== 7) args.push('--open-kernel', String(pre.openKernel));
	if (pre.closeKernel !== 5) args.push('--close-kernel', String(pre.closeKernel));

	const indices = Object.keys(calc).filter(k => calc[k]).map(k => k.toLowerCase());
	if (indices.length > 0) {
		args.push('--veg-idx', indices.join(','));
	}

	const executable = process.platform.includes('win') ? 'window_build\\calib.exe' : './calib';

	console.log('Executing command:', executable, args.join(' '));

	// Remove existing output directory
	try {
		if (fs.existsSync(outDir)) {
			fs.rmSync(outDir, { recursive: true, force: true });
		}
	} catch (e) {
		console.error('Failed to remove outDir:', e);
	}

	let exePath = path.join(process.cwd(), executable);
	if (!fs.existsSync(exePath) && !process.platform.includes('win')) {
		// Try without ./ if it fails
		exePath = 'calib';
	}

	res.writeHead(200, {
		'Content-Type': 'application/x-ndjson',
		'Cache-Control': 'no-cache',
		'Connection': 'keep-alive'
	});

	const child = spawn(exePath, args);

	child.stdout.on('data', data => {
		res.write(JSON.stringify({ stdout: data.toString() }) + '\n');
		console.log(`STDOUT: ${data.toString().trim()}`);
	});

	child.stderr.on('data', data => {
		res.write(JSON.stringify({ stderr: data.toString() }) + '\n');
		console.error(`STDERR: ${data.toString().trim()}`);
	});

	child.on('close', code => {
		res.write(JSON.stringify({
			code,
			command: `${executable} ${args.join(' ')}`
		}) + '\n');
		res.end();
	});

	child.on('error', err => {
		res.write(JSON.stringify({
			error: `Failed to start process: ${err.message}`,
			command: `${executable} ${args.join(' ')}`
		}) + '\n');
		res.end();
	});
}

server.listen(PORT, () => {
	console.log(`Server running at http://localhost:${PORT}/`);
	console.log(`Press Ctrl+C to stop`);
});
