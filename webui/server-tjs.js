import path from 'tjs:path';

const PORT = Number(tjs.env.PORT || 11918);
const HOST = '0.0.0.0';

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

async function exists(p) {
    try {
        await tjs.stat(p);
        return true;
    } catch {
        return false;
    }
}

async function executeCalibration(config) {
    const {
        inDir = '.input',
        outDir = '.output',
        doRadio = false,
        autoRadioThickness = -1,
        radioRefFile = '',
        radioTemplatePath = '',
        optimize = false
    } = config;

    const args = [inDir, outDir];
    if (doRadio) {
        args.push('--radio');
        if (radioRefFile) args.push('--ref', radioRefFile);
    }
    if (autoRadioThickness >= 0) {
        args.push('--auto');
        if (autoRadioThickness > 0) args.push(autoRadioThickness.toString());
    }
    if (radioTemplatePath) args.push('--template', radioTemplatePath);
    if (optimize) args.push('--optimize');

    let executable = tjs.platform === 'windows' ? 'calib.exe' : './calib';
    executable = `window_build/calib.exe`;

    console.log('Executing command:', executable, args.join(' '));

    await tjs.remove(path.join(tjs.cwd, outDir));

    let exePath = path.join(tjs.cwd, executable);
    
    if (!(await tjs.stat(exePath)) && tjs.platform !== 'windows') {
        exePath = 'calib';
    }

    try {
        const proc = tjs.spawn([exePath, ...args], {stdout: 'pipe', stderr: 'pipe'});

        const status = await proc.wait();
        let stdout = await proc.stdout?.text?.();
        let stderr = await proc.stderr?.text?.();

        return new Response(JSON.stringify({
            command: `${executable} ${args.join(' ')}`,
            code: status?.exitCode || 0,
            stdout,
            stderr,
        }), {
            headers: { 'Content-Type': 'application/json' }
        });
    } catch (err) {
        return new Response(JSON.stringify({
            error: `Failed to start process: ${err.message}`,
            command: `${exePath} ${args.join(' ')}`
        }), {
            status: 500,
            headers: { 'Content-Type': 'application/json' }
        });
    }
}

const server = tjs.serve({
    fetch: async (req) => {
        try {
            console.log(req.url);

            const url = new URL(req.url);
            const pathname = url.pathname;
            const method = req.method;

            // Handle CLI command execution
            if (method === 'POST' && pathname === '/run') {
                try {
                    const config = await req.json();
                    return await executeCalibration(config);
                } catch (err) {
                    return new Response(JSON.stringify({ error: 'Invalid JSON body: ' + err.message }), {
                        status: 400,
                        headers: { 'Content-Type': 'application/json' }
                    });
                }
            }

            // Handle directory listing
            if (method === 'GET' && pathname === '/list') {
                const dir = url.searchParams.get('dir') || '.output';
                const safePath = path.normalize(dir).replace(/^(\.\.[\/\\])+/, '');
                const absolutePath = path.join(tjs.cwd, safePath);
                try {
                    const dirIter = await tjs.readDir(absolutePath);
                    let list = [];
                    for await (const file of dirIter) {
                        list.push({
                            name: file.name,
                            isDirectory: !file.name.includes('.'),
                            ext: path.extname(file.name).toLowerCase()
                        })
                    }

                    return new Response(JSON.stringify(list), {
                        headers: { 'Content-Type': 'application/json' }
                    });
                } catch (err) {
                    console.log(err)
                    return new Response(JSON.stringify({ error: err.message }), {
                        status: 500,
                        headers: { 'Content-Type': 'application/json' }
                    });
                }
            }

            // Serve static files
            let filePath = pathname === '/' ? '/index.html' : pathname;
            let safePath = path.normalize(filePath).replace(/^(\.\.[\/\\])+/, '');

            let scriptPath = new URL(import.meta.url).pathname;
            if (tjs.platform === 'windows' && scriptPath.startsWith('/')) {
                scriptPath = scriptPath.slice(1);
            }
            const __dirname = path.dirname(scriptPath);

            let absolutePath = path.join(__dirname, safePath);
            let absolutePath2 = path.join(tjs.cwd, safePath);

            if (!(await exists(absolutePath)) && (await exists(absolutePath2))) {
                absolutePath = absolutePath2;
            }

            const content = await tjs.readFile(absolutePath);
            const ext = path.extname(absolutePath).toLowerCase();
            return new Response(content, {
                status: 200,
                headers: { 'Content-Type': MIME_TYPES[ext] || 'application/octet-stream' }
            });
        } catch (err) {
            console.log(err)
            return new Response('404 Not Found', { status: 404,  });
        }
    },
    port: PORT
});

console.log(`Server running at http://localhost:${PORT}/`);
console.log(`Press Ctrl+C to stop`);
