/**
 * Browser client for the robot_remote_control web transports:
 * - commands are POSTed as ControlMessages to the http server of a TransportHttp (/api/command)
 * - telemetry is received from a TransportWebSocket in SERVER_TEXT mode
 * Framework independent, the session wraps it into reactive state.
 */
import { telemetryTypeName } from './protocol';
import { parseJson } from './util';

export type TelemetryCallback = (data: unknown, channel: number) => void;
export type AnyTelemetryCallback = (type: string, data: unknown, channel: number) => void;

export interface ConnectionStatus {
	telemetry: 'connecting' | 'connected' | 'disconnected';
	commands: 'unknown' | 'online' | 'offline';
	/** heartbeat round trip time in ms */
	latency: number | null;
}

export class RobotRemoteControl {
	private readonly listeners = new Map<string, Set<TelemetryCallback>>();
	private readonly anyListeners = new Set<AnyTelemetryCallback>();
	private readonly statusListeners = new Set<(status: ConnectionStatus) => void>();
	private socket: WebSocket | null = null;
	private reconnectTimer: ReturnType<typeof setTimeout> | undefined;
	private heartbeatTimer: ReturnType<typeof setInterval> | undefined;
	readonly status: ConnectionStatus = { telemetry: 'connecting', commands: 'unknown', latency: null };

	/**
	 * @param commandUrl url of /api/command of the TransportHttp (the robot does not allow cross origin requests)
	 * @param telemetryUrl url of the TransportWebSocket
	 */
	constructor(
		readonly commandUrl: string,
		readonly telemetryUrl: string
	) {}

	// ------------------------------------------------------------ commands

	/**
	 * send a ControlMessage
	 * @param type ControlMessageType name, e.g. 'TWIST_COMMAND'
	 * @param payload the command message, stringified into ControlMessage.json
	 * @returns the reply text (ControlMessageReply, telemetry or plain text for version requests)
	 */
	async command(type: string, payload?: unknown, timeout = 5000): Promise<string> {
		const body: { type: string; json?: string } = { type };
		if (payload !== undefined) {
			body.json = JSON.stringify(payload);
		}
		const abort = new AbortController();
		const timer = setTimeout(() => abort.abort(), timeout);
		try {
			const response = await fetch(this.commandUrl, {
				method: 'POST',
				headers: { 'Content-Type': 'application/json' },
				body: JSON.stringify(body),
				signal: abort.signal
			});
			const text = await response.text();
			if (!response.ok) {
				throw new Error(`${type} failed: ${response.status} ${text}`);
			}
			this.setStatus({ commands: 'online' });
			return text;
		} catch (error) {
			this.setStatus({ commands: 'offline' });
			throw error;
		} finally {
			clearTimeout(timer);
		}
	}

	async commandJson<T>(type: string, payload?: unknown, timeout?: number): Promise<T | null> {
		return parseJson<T>(await this.command(type, payload, timeout));
	}

	/**
	 * @returns the latest telemetry of this type stored by the robot, null if there is none
	 */
	requestTelemetry<T>(type: string, channel = 0, timeout = 5000): Promise<T | null> {
		return this.commandJson<T>('TELEMETRY_REQUEST', { type, channel }, timeout);
	}

	// ------------------------------------------------------------ telemetry

	connectTelemetry() {
		clearTimeout(this.reconnectTimer);
		const socket = new WebSocket(this.telemetryUrl);
		this.socket = socket;
		socket.onopen = () => this.setStatus({ telemetry: 'connected' });
		socket.onclose = () => {
			if (this.socket !== socket) {
				return;
			}
			this.setStatus({ telemetry: 'disconnected' });
			this.reconnectTimer = setTimeout(() => this.connectTelemetry(), 2000);
		};
		socket.onmessage = (event) => this.handleTelemetry(String(event.data));
	}

	private handleTelemetry(text: string) {
		// TelemetryMessage: {type, channel, json}, json is the stringified telemetry
		const message = parseJson<{ type?: number | string; channel?: number; json?: string }>(text);
		if (!message || message.type === undefined) {
			return;
		}
		const type = telemetryTypeName(message.type);
		const channel = message.channel ?? 0;
		const data = parseJson(message.json);
		if (data === null) {
			return;
		}
		this.listeners.get(type)?.forEach((callback) => callback(data, channel));
		this.anyListeners.forEach((callback) => callback(type, data, channel));
	}

	/**
	 * @param type TelemetryMessageType name, e.g. 'CURRENT_POSE'
	 * @returns function to remove the callback
	 */
	on<T>(type: string, callback: (data: T, channel: number) => void): () => void {
		let callbacks = this.listeners.get(type);
		if (!callbacks) {
			callbacks = new Set();
			this.listeners.set(type, callbacks);
		}
		const wrapped = callback as TelemetryCallback;
		callbacks.add(wrapped);
		return () => callbacks.delete(wrapped);
	}

	onAny(callback: AnyTelemetryCallback): () => void {
		this.anyListeners.add(callback);
		return () => this.anyListeners.delete(callback);
	}

	// ------------------------------------------------------------ connection state

	onStatus(callback: (status: ConnectionStatus) => void): () => void {
		this.statusListeners.add(callback);
		callback({ ...this.status });
		return () => this.statusListeners.delete(callback);
	}

	private setStatus(changes: Partial<ConnectionStatus>) {
		const keys = Object.keys(changes) as (keyof ConnectionStatus)[];
		if (keys.some((key) => this.status[key] !== changes[key])) {
			Object.assign(this.status, changes);
			this.statusListeners.forEach((callback) => callback({ ...this.status }));
		}
	}

	/**
	 * Send heartbeats, the robot detects a lost connection (e.g. to stop) when they are missing.
	 * @param interval seconds between heartbeats
	 * @param duration seconds until the robot expects the next heartbeat
	 */
	startHeartbeat(interval = 0.5, duration = 1.0) {
		let busy = false;
		clearInterval(this.heartbeatTimer);
		this.heartbeatTimer = setInterval(async () => {
			if (busy) {
				return;
			}
			busy = true;
			const start = performance.now();
			try {
				await this.command('HEARTBEAT', { heartBeatDuration: duration }, 2000);
				this.setStatus({ latency: Math.round(performance.now() - start) });
			} catch {
				this.setStatus({ latency: null });
			} finally {
				busy = false;
			}
		}, interval * 1000);
	}

	close() {
		clearInterval(this.heartbeatTimer);
		clearTimeout(this.reconnectTimer);
		const socket = this.socket;
		this.socket = null;
		socket?.close();
	}
}

/**
 * Sends values one request at a time and in order, values set while a request is running are
 * replaced by newer ones (e.g. twist or joint commands from continuous user input).
 */
export class LatestValueSender<T> {
	private pending: T | undefined;
	private hasPending = false;
	private busy = false;

	constructor(private readonly send: (value: T) => Promise<unknown>) {}

	set(value: T) {
		this.pending = value;
		this.hasPending = true;
		if (!this.busy) {
			void this.flush();
		}
	}

	private async flush() {
		this.busy = true;
		while (this.hasPending) {
			const value = this.pending as T;
			this.hasPending = false;
			try {
				await this.send(value);
			} catch (error) {
				console.warn(error);
			}
		}
		this.busy = false;
	}
}
