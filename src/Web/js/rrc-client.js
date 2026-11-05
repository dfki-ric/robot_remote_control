/**
 * Browser client for the robot_remote_control web transports:
 * - commands are POSTed as ControlMessages to the http server of a TransportHttp (/api/command)
 * - telemetry is received from a TransportWebSocket in SERVER_TEXT mode
 * Both use the protobuf json format: lowerCamelCase field names, enums as names, default values are omitted.
 */

// TelemetryMessageType enum of RobotRemoteControl.proto, the index is the value
export const TELEMETRY_TYPES = [
    'NO_TELEMETRY_DATA', 'CURRENT_POSE', 'JOINT_STATE', 'CONTROLLABLE_JOINTS', 'SIMPLE_ACTIONS', 'COMPLEX_ACTIONS',
    'ROBOT_NAME', 'ROBOT_STATE', 'LOG_MESSAGE', 'VIDEO_STREAMS', 'SIMPLE_SENSOR', 'WRENCH_STATE', 'MAP', 'POSES',
    'TRANSFORMS', 'PERMISSION_REQUEST', 'POINTCLOUD', 'IMU_VALUES', 'CONTACT_POINTS', 'CURRENT_TWIST',
    'CURRENT_ACCELERATION', 'CAMERA_INFORMATION', 'IMAGE', 'IMAGE_LAYERS', 'ODOMETRY', 'CONTROLLABLE_FRAMES',
    'FILE_DEFINITION', 'ROBOT_MODEL_INFORMATION', 'INTERFACE_OPTIONS', 'CHANNELS_DEFINITION', 'LASER_SCAN',
];

export const LOG_LEVELS = {0: 'NONE', 1: 'FATAL', 2: 'ERROR', 3: 'WARN', 4: 'INFO', 5: 'DEBUG', 20: 'CUSTOM'};

export function telemetryTypeName(type) {
    return typeof type === 'number' ? (TELEMETRY_TYPES[type] || String(type)) : type;
}

/**
 * @returns the parsed json, null for empty or invalid text (e.g. telemetry that was never set by the robot)
 */
export function parseJson(text) {
    if (!text) {
        return null;
    }
    try {
        return JSON.parse(text);
    } catch (error) {
        return null;
    }
}

export function base64ToBytes(base64) {
    const binary = atob(base64 || '');
    const bytes = new Uint8Array(binary.length);
    for (let i = 0; i < binary.length; ++i) {
        bytes[i] = binary.charCodeAt(i);
    }
    return bytes;
}

export class RobotRemoteControl {
    /**
     * @param host host of the robot
     * @param commandPort port of the TransportHttp
     * @param telemetryPort port of the TransportWebSocket
     */
    constructor(host, commandPort, telemetryPort) {
        this.commandUrl = `http://${host}:${commandPort}/api/command`;
        this.telemetryUrl = `ws://${host}:${telemetryPort}`;
        this.listeners = new Map();  // type name -> Set of callback(data, channel, message)
        this.anyListeners = new Set();
        this.statusListeners = new Set();
        this.status = {telemetry: 'connecting', commands: 'unknown', latency: null};
        this.socket = null;
        this.reconnectTimer = null;
        this.heartbeatTimer = null;
    }

    // ------------------------------------------------------------ commands

    /**
     * send a ControlMessage
     * @param type ControlMessageType name, e.g. 'TWIST_COMMAND'
     * @param payload the command message as object (stringified into ControlMessage.json)
     * @returns the reply text (ControlMessageReply, telemetry or plain text for version requests)
     */
    async command(type, payload, timeout = 5000) {
        const body = {type};
        if (payload !== undefined) {
            body.json = JSON.stringify(payload);
        }
        const abort = new AbortController();
        const timer = setTimeout(() => abort.abort(), timeout);
        try {
            const response = await fetch(this.commandUrl, {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify(body),
                signal: abort.signal,
            });
            const text = await response.text();
            if (!response.ok) {
                throw new Error(`${type} failed: ${response.status} ${text}`);
            }
            this.setStatus({commands: 'online'});
            return text;
        } catch (error) {
            this.setStatus({commands: 'offline'});
            throw error;
        } finally {
            clearTimeout(timer);
        }
    }

    async commandJson(type, payload, timeout) {
        return parseJson(await this.command(type, payload, timeout));
    }

    /**
     * @returns the latest telemetry of this type stored by the robot, null if there is none
     */
    requestTelemetry(type, channel = 0, timeout = 5000) {
        return this.commandJson('TELEMETRY_REQUEST', {type, channel}, timeout);
    }

    // ------------------------------------------------------------ telemetry

    connectTelemetry() {
        clearTimeout(this.reconnectTimer);
        const socket = new WebSocket(this.telemetryUrl);
        this.socket = socket;
        socket.onopen = () => this.setStatus({telemetry: 'connected'});
        socket.onclose = () => {
            if (this.socket !== socket) {
                return;
            }
            this.setStatus({telemetry: 'disconnected'});
            this.reconnectTimer = setTimeout(() => this.connectTelemetry(), 2000);
        };
        socket.onmessage = (event) => this.handleTelemetry(event.data);
    }

    handleTelemetry(text) {
        const message = parseJson(text);
        if (!message) {
            return;
        }
        const type = telemetryTypeName(message.type);
        const channel = message.channel || 0;
        const data = parseJson(message.json);
        if (data === null) {
            return;
        }
        const callbacks = this.listeners.get(type);
        if (callbacks) {
            callbacks.forEach((callback) => callback(data, channel));
        }
        this.anyListeners.forEach((callback) => callback(type, data, channel));
    }

    /**
     * @param type TelemetryMessageType name, e.g. 'CURRENT_POSE'
     * @param callback called with (data, channel)
     * @returns function to remove the callback
     */
    on(type, callback) {
        if (!this.listeners.has(type)) {
            this.listeners.set(type, new Set());
        }
        this.listeners.get(type).add(callback);
        return () => this.listeners.get(type).delete(callback);
    }

    /**
     * @param callback called with (type, data, channel) for all telemetry
     */
    onAny(callback) {
        this.anyListeners.add(callback);
    }

    // ------------------------------------------------------------ connection state

    onStatus(callback) {
        this.statusListeners.add(callback);
        callback(this.status);
    }

    setStatus(changes) {
        const changed = Object.keys(changes).some((key) => this.status[key] !== changes[key]);
        if (changed) {
            Object.assign(this.status, changes);
            this.statusListeners.forEach((callback) => callback(this.status));
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
                await this.command('HEARTBEAT', {heartBeatDuration: duration}, 2000);
                this.setStatus({latency: Math.round(performance.now() - start)});
            } catch (error) {
                this.setStatus({latency: null});
            } finally {
                busy = false;
            }
        }, interval * 1000);
    }
}

/**
 * Sends values one request at a time and in order, values set while a request is running are
 * replaced by newer ones (e.g. twist or joint commands from continuous user input).
 */
export class LatestValueSender {
    constructor(send) {
        this.send = send;
        this.pending = null;
        this.hasPending = false;
        this.busy = false;
    }

    set(value) {
        this.pending = value;
        this.hasPending = true;
        if (!this.busy) {
            this.flush();
        }
    }

    async flush() {
        this.busy = true;
        while (this.hasPending) {
            const value = this.pending;
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
