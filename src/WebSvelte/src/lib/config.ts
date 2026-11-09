/**
 * Connection settings from the url of the page. Invalid parameters throw, the page shows the error
 * instead of connecting somewhere unexpected.
 *
 * | parameter     | default                                                          |
 * | ------------- | ---------------------------------------------------------------- |
 * | commandport   | port of the page (commands go to the server that served the page) |
 * | telemetryport | 7002 (development server: proxied through /telemetry)            |
 */

export const DEFAULT_TELEMETRY_PORT = 7002;

export class ConfigError extends Error {}

export interface ConnectionConfig {
	commandUrl: string;
	telemetryUrl: string;
}

function parsePort(name: string, value: string): number {
	const port = Number(value);
	if (!/^\d+$/.test(value) || port < 1 || port > 65535) {
		throw new ConfigError(`url parameter ${name}=${value} is not a port number (1-65535)`);
	}
	return port;
}

/**
 * @param dev the development server proxies the robot (see vite.config.ts)
 */
export function readConnectionConfig(location: Location, dev: boolean): ConnectionConfig {
	const params = new URLSearchParams(location.search);
	const host = location.hostname || '127.0.0.1';
	const wsScheme = location.protocol === 'https:' ? 'wss:' : 'ws:';

	const commandPort = params.get('commandport');
	const commandUrl =
		commandPort !== null
			? `${location.protocol}//${host}:${parsePort('commandport', commandPort)}/api/command`
			: `${location.origin}/api/command`;

	const telemetryPort = params.get('telemetryport');
	let telemetryUrl: string;
	if (telemetryPort !== null) {
		telemetryUrl = `${wsScheme}//${host}:${parsePort('telemetryport', telemetryPort)}`;
	} else if (dev) {
		telemetryUrl = `${wsScheme}//${location.host}/telemetry`;
	} else {
		telemetryUrl = `${wsScheme}//${host}:${DEFAULT_TELEMETRY_PORT}`;
	}
	return { commandUrl, telemetryUrl };
}
