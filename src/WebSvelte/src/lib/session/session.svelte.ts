/**
 * Connection to one robot: interface description (name, actions, joints, frames...), robot model, commands
 * and the telemetry state. Components get it with getSession().
 */
import { createContext } from 'svelte';
import type { IUrdfRobot } from 'urdf-viewer';
import { LatestValueSender, RobotRemoteControl, type ConnectionStatus } from '../rrc/client';
import { RobotKinematics } from '../rrc/kinematics';
import { loadRobotModel, type ModelFiles } from '../rrc/model-loader';
import {
	TELEMETRY_TYPES,
	type ChannelDefinition, type ChannelsDefinitionMsg, type ComplexAction, type ComplexActionsMsg,
	type ControllableFrame, type ControllableFramesMsg, type FileDefinitionMsg, type FolderMsg,
	type InterfaceOptionsMsg, type JointStateMsg, type MapMsg, type PoseMsg, type RobotStateMsg, type SimpleAction,
	type SimpleActionsMsg, type StringMsg, type TwistMsg, type VideoStreamsMsg
} from '../rrc/protocol';
import { base64ToBytes } from '../rrc/util';
import { MapLayers } from './maps.svelte';
import { channelNameLookup, TelemetryState } from './telemetry.svelte';

export interface RobotModel {
	robot: IUrdfRobot;
	kinematics: RobotKinematics;
	files: ModelFiles;
}

export interface Toast {
	id: number;
	message: string;
}

export interface MapChannel {
	channel: number;
	name: string;
}

export interface DownloadedFile {
	path: string;
	url: string;
}

const INFO_LABELS: Record<string, string> = {
	name: 'name', model: 'model', library: 'library version', protocol: 'protocol', git: 'git version',
	options: 'options', frames: 'controllable', commands: 'commands', telemetry: 'telemetry'
};

export class RobotSession {
	readonly rrc: RobotRemoteControl;
	readonly telemetry: TelemetryState;
	readonly maps: MapLayers;

	status = $state<ConnectionStatus>({ telemetry: 'connecting', commands: 'unknown', latency: null });
	/** null until the interface was loaded */
	robotName = $state<string | null>(null);
	unreachable = $state(false);
	robotState = $state.raw<RobotStateMsg | null>(null);
	channels = $state.raw<ChannelDefinition[]>([]);
	controllableFrames = $state.raw<ControllableFrame[]>([]);
	controllableJoints = $state.raw<string[]>([]);
	simpleActions = $state.raw<SimpleAction[]>([]);
	complexActions = $state.raw<ComplexAction[]>([]);
	videoStreams = $state.raw<string[]>([]);
	files = $state.raw<FileDefinitionMsg | null>(null);
	mapChannels = $state.raw<MapChannel[]>([]);
	info = $state<Record<string, string>>({});
	model = $state.raw<RobotModel | null>(null);
	/** progress or error of loading the model, empty when done */
	modelStatus = $state('');
	toasts = $state<Toast[]>([]);

	private dynamicSimpleActions = false;
	private loadingInterface = false;
	private toastId = 0;
	private readonly jointSender: LatestValueSender<JointStateMsg>;
	readonly channelName: (type: string, channel: number) => string;

	constructor(commandUrl: string, telemetryUrl: string) {
		this.rrc = new RobotRemoteControl(commandUrl, telemetryUrl);
		this.channelName = channelNameLookup(() => this.channels);
		this.telemetry = new TelemetryState(this.rrc, this.channelName);
		this.maps = new MapLayers((channel) => this.request<MapMsg>('MAP', channel, 60000));
		this.jointSender = new LatestValueSender((command) => this.rrc.command('JOINTS_COMMAND', command));

		this.rrc.on<RobotStateMsg>('ROBOT_STATE', (state) => (this.robotState = state));
		this.rrc.on<SimpleActionsMsg>('SIMPLE_ACTIONS', (actions) => this.setSimpleActions(actions));
		// maps are usually requested, but may also be sent as telemetry
		this.rrc.on<MapMsg>('MAP', (map, channel) => {
			if (map.map) {
				this.maps.receive(channel, map.map);
			}
		});

		let telemetryWasConnected: boolean | null = false;
		this.rrc.onStatus((status) => {
			this.status = status;
			if (status.telemetry === 'connected') {
				// reload after a reconnect, the robot may have been restarted
				if (telemetryWasConnected === null) {
					void this.loadInterface();
				}
				telemetryWasConnected = true;
			} else if (telemetryWasConnected) {
				telemetryWasConnected = null;
			}
		});
	}

	start() {
		this.setInfo('commands', this.rrc.commandUrl);
		this.setInfo('telemetry', this.rrc.telemetryUrl);
		this.rrc.connectTelemetry();
		this.rrc.startHeartbeat();
		void this.loadInterface();
	}

	stop() {
		this.rrc.close();
		this.model?.files.dispose();
	}

	// ------------------------------------------------------------ messages to the user

	toast(message: string) {
		this.toasts.push({ id: ++this.toastId, message });
	}

	dismissToast(id: number) {
		const index = this.toasts.findIndex((toast) => toast.id === id);
		if (index >= 0) {
			this.toasts.splice(index, 1);
		}
	}

	// ------------------------------------------------------------ commands

	/**
	 * send a command, errors are shown to the user and rethrown
	 */
	async send(type: string, payload?: unknown): Promise<string> {
		try {
			return await this.rrc.command(type, payload);
		} catch (error) {
			const name = error instanceof Error ? error.name : '';
			this.toast(name === 'AbortError' ? `${type}: no reply from the robot` : String((error as Error).message ?? error));
			throw error;
		}
	}

	/**
	 * @returns the telemetry stored by the robot, null if there is none or the request failed
	 */
	request<T>(type: string, channel = 0, timeout = 5000): Promise<T | null> {
		return this.rrc.requestTelemetry<T>(type, channel, timeout).catch(() => null);
	}

	supportsControl(type: string): boolean {
		return this.controllableFrames.some((frame) => (frame.type ?? []).includes(type));
	}

	get twistFrames(): string[] {
		return this.controllableFrames
			.filter((frame) => (frame.type ?? []).includes('TWIST_CONTROL') && frame.frame)
			.map((frame) => frame.frame as string);
	}

	/**
	 * a pose in the frame of the current pose (goals and task points are picked in it)
	 */
	poseMessage(x: number, y: number, yaw: number | null): PoseMsg {
		const pose: PoseMsg = { header: { frame: this.telemetry.pose?.header?.frame ?? '' }, position: { x, y, z: 0 } };
		if (yaw !== null) {
			pose.orientation = { x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) };
		}
		return pose;
	}

	async sendGoal(x: number, y: number, yaw: number | null, maxForwardSpeed: number) {
		const pose = this.poseMessage(x, y, yaw);
		// robots without information on controllable frames get a target pose
		const gotoOnly = this.controllableFrames.length > 0 && !this.supportsControl('POSE_CONTROL') && this.supportsControl('GOTO_CONTROL');
		if (gotoOnly) {
			await this.send('GOTO_COMMAND', { waypointPose: pose, maxForwardSpeed }).catch(() => {});
		} else {
			await this.send('TARGET_POSE_COMMAND', pose).catch(() => {});
		}
	}

	sendJoint(name: string, position: number) {
		this.jointSender.set({ name: [name], position: [position] });
	}

	async sendSimpleAction(name: string, state: number) {
		try {
			await this.send('SIMPLE_ACTIONS_COMMAND', { name, state });
		} catch {
			return;
		}
		if (!this.dynamicSimpleActions) {
			// the robot does not send updates, the state may still change
			const actions = await this.request<SimpleActionsMsg>('SIMPLE_ACTIONS');
			if (actions) {
				this.setSimpleActions(actions);
			}
		}
	}

	/**
	 * @returns true if the robot accepted the command
	 */
	async sendComplexAction(command: { name?: string; type?: string; poses?: PoseMsg[]; twists?: TwistMsg[] }): Promise<boolean> {
		try {
			await this.send('COMPLEX_ACTION_COMMAND', command);
			return true;
		} catch {
			return false;
		}
	}

	async answerPermissionRequest(requestuid: string | undefined, granted: boolean) {
		this.telemetry.resolvePermissionRequest(requestuid);
		await this.send('PERMISSION', { requestuid, granted }).catch(() => {});
	}

	async setLogLevel(level: string) {
		await this.send('LOG_LEVEL_SELECT', { level }).catch(() => {});
	}

	/**
	 * @returns object urls of the transferred files, the caller revokes them
	 */
	async downloadFiles(identifier: string): Promise<DownloadedFile[]> {
		const folder = JSON.parse(await this.send('FILE_REQUEST', { identifier, compressed: false })) as FolderMsg;
		return (folder.file ?? [])
			.filter((file) => file.data && file.path)
			.map((file) => ({ path: file.path as string, url: URL.createObjectURL(new Blob([base64ToBytes(file.data)])) }));
	}

	// ------------------------------------------------------------ interface description

	private setInfo(key: string, value: string) {
		this.info[key] = value;
	}

	get infoRows(): [label: string, value: string][] {
		return Object.keys(INFO_LABELS)
			.filter((key) => this.info[key] !== undefined)
			.map((key) => [INFO_LABELS[key], this.info[key]]);
	}

	async loadInterface() {
		if (this.loadingInterface) {
			return;
		}
		this.loadingInterface = true;
		try {
			const [name, options, channels, frames, joints, actions, complex, streams, files, robotState] = await Promise.all([
				this.request<StringMsg>('ROBOT_NAME'),
				this.request<InterfaceOptionsMsg>('INTERFACE_OPTIONS'),
				this.request<ChannelsDefinitionMsg>('CHANNELS_DEFINITION'),
				this.request<ControllableFramesMsg>('CONTROLLABLE_FRAMES'),
				this.request<JointStateMsg>('CONTROLLABLE_JOINTS'),
				this.request<SimpleActionsMsg>('SIMPLE_ACTIONS'),
				this.request<ComplexActionsMsg>('COMPLEX_ACTIONS'),
				this.request<VideoStreamsMsg>('VIDEO_STREAMS'),
				this.request<FileDefinitionMsg>('FILE_DEFINITION'),
				this.request<RobotStateMsg>('ROBOT_STATE')
			]);
			this.unreachable = this.rrc.status.commands !== 'online';
			if (this.unreachable) {
				return;
			}
			const robotName = name?.value ?? '';
			const otherRobot = this.robotName !== null && this.robotName !== robotName;
			this.robotName = robotName;
			document.title = `${robotName || 'Robot'} · Robot Remote Control`;
			this.dynamicSimpleActions = Boolean(options?.dynamicSimpleActions);
			this.channels = channels?.channel ?? [];
			this.controllableFrames = frames?.controllableFrames ?? [];
			this.controllableJoints = joints?.name ?? [];
			if (actions) {
				this.setSimpleActions(actions);
			}
			this.complexActions = complex?.actions ?? [];
			this.videoStreams = (streams?.stream ?? []).map((stream) => stream.url ?? '').filter(Boolean);
			this.files = files;
			this.mapChannels = this.listMapChannels();
			if (robotState) {
				this.robotState = robotState;
			}
			this.setInfo('name', robotName || '–');
			this.setInfo('options', options ? Object.keys(options).filter((key) => options[key]).join(', ') || 'none' : '–');
			this.setInfo('frames', this.controllableFrames
				.map((frame) => `${frame.frame} (${(frame.type ?? []).map((t) => t.replace('_CONTROL', '').toLowerCase()).join(', ')})`)
				.join('; ') || 'none');
			void this.loadVersions();
			if (!this.model || otherRobot) {
				void this.loadModel();
			}
		} finally {
			this.loadingInterface = false;
		}
	}

	private setSimpleActions(actions: SimpleActionsMsg) {
		this.simpleActions = actions.actions ?? [];
	}

	private listMapChannels(): MapChannel[] {
		const mapType = TELEMETRY_TYPES.indexOf('MAP');
		const channels = this.channels
			.filter((channel) => (channel.messagetype ?? 0) === mapType)
			.map((channel) => ({ name: channel.name ?? '', channel: channel.channelno ?? 0 }));
		if (!channels.some((channel) => channel.channel === 0)) {
			channels.unshift({ name: 'default', channel: 0 });
		}
		return channels;
	}

	private async loadVersions() {
		for (const [key, type] of [['library', 'LIBRARY_VERSION'], ['protocol', 'PROTOCOL_VERSION'], ['git', 'GIT_VERSION']]) {
			try {
				const version = await this.rrc.command(type);
				this.setInfo(key, type === 'LIBRARY_VERSION' ? version : version.slice(0, 12));
			} catch {
				this.setInfo(key, '–');
			}
		}
	}

	async loadModel() {
		try {
			const loaded = await loadRobotModel(this.rrc, this.robotName ?? '', (text) => (this.modelStatus = `${text}…`));
			this.modelStatus = '';
			if (!loaded) {
				this.setInfo('model', 'none');
				return;
			}
			this.model?.files.dispose();
			const kinematics = new RobotKinematics(loaded.robot);
			this.model = { robot: loaded.robot, kinematics, files: loaded.files };
			this.setInfo('model', `${loaded.robot.name || 'unnamed'}, ${kinematics.movableJointCount} movable joints (from the ${loaded.source})`);
			if (loaded.missing.length) {
				this.toast(`${loaded.missing.length} model file(s) are missing: ${loaded.missing.slice(0, 3).join(', ')}`);
			}
		} catch (error) {
			this.modelStatus = `model: ${(error as Error).message}`;
			this.setInfo('model', 'failed');
		}
	}
}

export const [getSession, setSession] = createContext<RobotSession>();
