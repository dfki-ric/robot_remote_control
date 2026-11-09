/**
 * Reactive state of the continuous telemetry of the robot. Frequent messages are applied at most once per
 * animation frame, large messages (scans, clouds) are kept as raw state and replaced, not deeply proxied.
 */
import type { RobotRemoteControl } from '../rrc/client';
import {
	LOG_LEVELS, TELEMETRY_TYPES,
	type ChannelDefinition, type IMUMsg, type ImageLayersMsg, type ImageMsg, type JointStateMsg, type LaserScanMsg,
	type LogMessageMsg, type PermissionRequestMsg, type PointCloudMsg, type PoseMsg, type PosesMsg,
	type SimpleSensorMsg, type TwistMsg, type WrenchStateMsg
} from '../rrc/protocol';
import { format, RateMeter, throttle, timeOf, xyz } from '../rrc/util';
import { ImageSources } from './images.svelte';

const MAX_LOG_ENTRIES = 300;

export interface LogEntry {
	id: number;
	time: string;
	level: string;
	message: string;
}

export interface SensorReading {
	key: string;
	title: string;
	rows: [name: string, value: string][];
}

export interface TelemetryStats {
	type: string;
	channel: number;
	meter: RateMeter;
	data: unknown;
}

/**
 * calls apply once per animation frame with the latest value
 */
function perFrame<T>(apply: (value: T) => void): (value: T) => void {
	let latest: T;
	let scheduled = false;
	return (value: T) => {
		latest = value;
		if (!scheduled) {
			scheduled = true;
			requestAnimationFrame(() => {
				scheduled = false;
				apply(latest);
			});
		}
	};
}

export class TelemetryState {
	pose = $state.raw<PoseMsg | null>(null);
	twist = $state.raw<TwistMsg | null>(null);
	path = $state.raw<PosesMsg | null>(null);
	laserScan = $state.raw<LaserScanMsg | null>(null);
	pointCloud = $state.raw<PointCloudMsg | null>(null);
	/** joint name -> position */
	jointPositions = $state.raw<ReadonlyMap<string, number>>(new Map());
	log = $state.raw<LogEntry[]>([]);
	sensors = $state.raw<SensorReading[]>([]);
	permissionRequests = $state.raw<PermissionRequestMsg[]>([]);
	readonly images = new ImageSources();
	/** "<type>:<channel>" -> statistics, not reactive: the inspector polls it */
	readonly stats = new Map<string, TelemetryStats>();

	private logId = 0;
	private readonly sensorUpdates = new Map<string, (reading: SensorReading) => void>();

	/**
	 * @param channelName name of a channel from the channel definitions of the robot
	 */
	constructor(
		rrc: RobotRemoteControl,
		private readonly channelName: (type: string, channel: number) => string
	) {
		rrc.on<PoseMsg>('CURRENT_POSE', perFrame((pose) => (this.pose = pose)));
		rrc.on<TwistMsg>('CURRENT_TWIST', perFrame((twist) => (this.twist = twist)));
		rrc.on<PosesMsg>('POSES', perFrame((path) => (this.path = path)));
		rrc.on<LaserScanMsg>('LASER_SCAN', perFrame((scan) => (this.laserScan = scan)));
		rrc.on<PointCloudMsg>('POINTCLOUD', perFrame((cloud) => (this.pointCloud = cloud)));
		rrc.on<JointStateMsg>('JOINT_STATE', perFrame((state) => this.setJointState(state)));
		rrc.on<LogMessageMsg>('LOG_MESSAGE', (log) => this.addLog(log));
		rrc.on<PermissionRequestMsg>('PERMISSION_REQUEST', (request) => {
			if (!this.permissionRequests.some((pending) => pending.requestuid === request.requestuid)) {
				this.permissionRequests = [...this.permissionRequests, request];
			}
		});
		rrc.on<ImageMsg>('IMAGE', (image, channel) => {
			this.images.update(`image:${channel}`, this.channelName('IMAGE', channel) || `image ${channel}`, image);
		});
		rrc.on<ImageLayersMsg>('IMAGE_LAYERS', (layers, channel) => {
			const name = this.channelName('IMAGE_LAYERS', channel) || `layers ${channel}`;
			(layers.layers ?? []).forEach((image, i) => {
				this.images.update(`layers:${channel}:${i}`, `${name}, ${i}: ${image.encoding ?? ''}`, image);
			});
		});
		rrc.on<SimpleSensorMsg>('SIMPLE_SENSOR', (sensor, channel) => this.setSimpleSensor(sensor, channel));
		rrc.on<IMUMsg>('IMU_VALUES', (imu, channel) => {
			this.setSensor({
				key: `imu:${channel}`,
				title: `IMU ${imu.header?.frame ?? ''}`,
				rows: [
					['acceleration (m/s²)', xyz(imu.acceleration)],
					['gyro (rad/s)', xyz(imu.gyro, 3)],
					['magnetometer', xyz(imu.mag)]
				]
			});
		});
		rrc.on<WrenchStateMsg>('WRENCH_STATE', (wrenches, channel) => {
			(wrenches.wrenches ?? []).forEach((wrench, i) => {
				this.setSensor({
					key: `wrench:${channel}:${i}`,
					title: `wrench ${wrench.header?.frame || i}`,
					rows: [
						['force (N)', xyz(wrench.force)],
						['torque (Nm)', xyz(wrench.torque, 3)]
					]
				});
			});
		});
		rrc.onAny((type, data, channel) => this.count(type, data, channel));
	}

	private setJointState(state: JointStateMsg) {
		const names = state.name ?? [];
		const positions = state.position ?? [];
		const merged = new Map(this.jointPositions);
		names.forEach((name, i) => {
			if (i < positions.length) {
				merged.set(name, positions[i]);
			}
		});
		this.jointPositions = merged;
	}

	private addLog(log: LogMessageMsg) {
		const value = log.level ?? 0;
		const level = LOG_LEVELS[value] ?? (value >= 20 ? 'CUSTOM' : String(value));
		const entry = { id: ++this.logId, time: timeOf(log.header), level, message: (log.message ?? '').trim() };
		this.log = [entry, ...this.log.slice(0, MAX_LOG_ENTRIES - 1)];
	}

	clearLog() {
		this.log = [];
	}

	private setSimpleSensor(sensor: SimpleSensorMsg, channel: number) {
		const values = sensor.value ?? [];
		const names = sensor.valueNames ?? [];
		let rows: SensorReading['rows'];
		if (values.length <= 12) {
			rows = values.map((value, i) => [names[i] ?? `value ${i}`, format(value, 3)]);
		} else {
			const finite = values.filter(Number.isFinite);
			const size = sensor.size ?? {};
			rows = [[`grid ${size.x ?? values.length}×${size.y ?? 1}`, `${format(Math.min(...finite))} … ${format(Math.max(...finite))}`]];
		}
		const title = sensor.name || this.channelName('SIMPLE_SENSOR', channel) || `sensor ${channel}`;
		this.setSensor({ key: `simple:${channel}`, title, rows });
	}

	/**
	 * sensors are shown in the order they were first received, each updated at most 4 times per second
	 */
	private setSensor(reading: SensorReading) {
		let update = this.sensorUpdates.get(reading.key);
		if (!update) {
			update = throttle((latest: SensorReading) => {
				const index = this.sensors.findIndex((sensor) => sensor.key === latest.key);
				this.sensors = index < 0
					? [...this.sensors, latest]
					: this.sensors.map((sensor, i) => (i === index ? latest : sensor));
			}, 250);
			this.sensorUpdates.set(reading.key, update);
		}
		update(reading);
	}

	resolvePermissionRequest(requestuid: string | undefined) {
		this.permissionRequests = this.permissionRequests.filter((request) => request.requestuid !== requestuid);
	}

	private count(type: string, data: unknown, channel: number) {
		const key = `${type}:${channel}`;
		let stats = this.stats.get(key);
		if (!stats) {
			stats = { type, channel, meter: new RateMeter(), data };
			this.stats.set(key, stats);
		}
		stats.meter.tick();
		stats.data = data;
	}
}

/**
 * @returns the channel name lookup for the channel definitions of the robot
 */
export function channelNameLookup(channels: () => ChannelDefinition[]) {
	return (type: string, channel: number): string => {
		const typeId = (TELEMETRY_TYPES as readonly string[]).indexOf(type);
		const definition = channels().find(
			(c) => (c.messagetype ?? 0) === typeId && (c.channelno ?? 0) === channel
		);
		return definition?.name ?? '';
	};
}
