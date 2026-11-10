import adapter from '@sveltejs/adapter-static';
import { sveltekit } from '@sveltejs/kit/vite';
import { defineConfig } from 'vite';

/**
 * The development server proxies the robot, it does not allow cross origin requests:
 * RRC_ROBOT (host, default 127.0.0.1), RRC_COMMAND_PORT (TransportHttp, 7001),
 * RRC_TELEMETRY_PORT (TransportWebSocket, 7002).
 */
function port(name: string, fallback: number): number {
	const value = process.env[name] ?? String(fallback);
	const number = Number(value);
	if (!/^\d+$/.test(value) || number < 1 || number > 65535) {
		throw new Error(`${name}=${value} is not a port number (1-65535)`);
	}
	return number;
}

const robot = process.env.RRC_ROBOT ?? '127.0.0.1';
const commandTarget = `http://${robot}:${port('RRC_COMMAND_PORT', 7001)}`;
const telemetryTarget = `ws://${robot}:${port('RRC_TELEMETRY_PORT', 7002)}`;

export default defineConfig({
	plugins: [
		sveltekit({
			compilerOptions: {
				// Force runes mode for the project, except for libraries. Can be removed in svelte 6.
				runes: ({ filename }) =>
					filename.split(/[/\\]/).includes('node_modules') ? undefined : true
			},
			// the robot serves the page from any folder (e.g. /html/index.html): route by hash and inline
			// everything into one index.html, so no absolute asset paths are needed
			router: { type: 'hash' },
			output: { bundleStrategy: 'inline' },
			adapter: adapter({
				pages: 'build',
				assets: 'build',
				fallback: 'index.html',
				strict: true
			})
		})
	],
	build: {
		// the Material Symbols font is inlined too: the page stays one file that works offline
		assetsInlineLimit: (file) => (file.includes('material-symbols-outlined') ? true : undefined)
	},
	resolve: {
		// urdf-viewer, threlte and the app must share one three.js
		dedupe: ['three']
	},
	server: {
		proxy: {
			'/api': commandTarget,
			'/model': commandTarget,
			'/telemetry': { target: telemetryTarget, ws: true, rewrite: () => '/' }
		}
	}
});
