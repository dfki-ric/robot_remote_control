/**
 * Message types of robot_remote_control in the protobuf json format: lowerCamelCase field names, enums as
 * names and fields with default values omitted (a pose at the origin is `"position": {}`, channel 0 is
 * missing), so every field is optional and numbers must be read with `?? 0`.
 */

// TelemetryMessageType enum of RobotRemoteControl.proto, the index is the value
export const TELEMETRY_TYPES = [
	'NO_TELEMETRY_DATA', 'CURRENT_POSE', 'JOINT_STATE', 'CONTROLLABLE_JOINTS', 'SIMPLE_ACTIONS', 'COMPLEX_ACTIONS',
	'ROBOT_NAME', 'ROBOT_STATE', 'LOG_MESSAGE', 'VIDEO_STREAMS', 'SIMPLE_SENSOR', 'WRENCH_STATE', 'MAP', 'POSES',
	'TRANSFORMS', 'PERMISSION_REQUEST', 'POINTCLOUD', 'IMU_VALUES', 'CONTACT_POINTS', 'CURRENT_TWIST',
	'CURRENT_ACCELERATION', 'CAMERA_INFORMATION', 'IMAGE', 'IMAGE_LAYERS', 'ODOMETRY', 'CONTROLLABLE_FRAMES',
	'FILE_DEFINITION', 'ROBOT_MODEL_INFORMATION', 'INTERFACE_OPTIONS', 'CHANNELS_DEFINITION', 'LASER_SCAN'
] as const;

export type TelemetryType = (typeof TELEMETRY_TYPES)[number];

export const LOG_LEVELS: Record<number, string> = {
	0: 'NONE', 1: 'FATAL', 2: 'ERROR', 3: 'WARN', 4: 'INFO', 5: 'DEBUG', 20: 'CUSTOM'
};

export const SELECTABLE_LOG_LEVELS = ['FATAL', 'ERROR', 'WARN', 'INFO', 'DEBUG'] as const;

export function telemetryTypeName(type: number | string): string {
	return typeof type === 'number' ? (TELEMETRY_TYPES[type] ?? String(type)) : type;
}

export interface Vector3Msg { x?: number; y?: number; z?: number }
export interface QuaternionMsg { x?: number; y?: number; z?: number; w?: number }
export interface TimeStamp { secs?: number; nsecs?: number }
export interface Header { frame?: string; timestamp?: TimeStamp }

export interface PoseMsg { header?: Header; position?: Vector3Msg; orientation?: QuaternionMsg }
export interface PosesMsg { header?: Header; poses?: PoseMsg[] }
export interface TwistMsg { header?: Header; linear?: Vector3Msg; angular?: Vector3Msg }
export interface JointStateMsg { header?: Header; name?: string[]; position?: number[] }

export interface LaserScanMsg {
	header?: Header;
	ranges?: number[];
	startAngle?: number;
	horizontalAngleResolution?: number;
	rangeMin?: number;
	rangeMax?: number;
}

export interface PointCloudMsg {
	'@type'?: string;
	header?: Header;
	origin?: PoseMsg;
	points?: Vector3Msg[];
}

export interface GridMapLayer { name?: string; size?: Vector3Msg; scale?: Vector3Msg; value?: number[] }
export interface GridMapMsg {
	'@type'?: string;
	header?: Header;
	frame?: string;
	origin?: PoseMsg;
	layers?: GridMapLayer[];
}
export type MapContent = PointCloudMsg | GridMapMsg;
export interface MapMsg { map?: MapContent }

export interface RobotStateMsg { state?: string[]; stateName?: string[] }
export interface LogMessageMsg { header?: Header; level?: number; message?: string }

export interface ChannelDefinition { messagetype?: number; channelno?: number; name?: string }
export interface ChannelsDefinitionMsg { channel?: ChannelDefinition[] }

export interface ControllableFrame { frame?: string; type?: string[] }
export interface ControllableFramesMsg { controllableFrames?: ControllableFrame[] }

export interface NamedValue { name?: string; value?: number }
export interface ActionDependency { dependsOnAction?: string; dependsOnActionInState?: number }
export interface SimpleActionDef {
	type?: string;
	minState?: number;
	maxState?: number;
	stepSize?: number;
	documentation?: string;
	valueNames?: NamedValue[];
	actionDependency?: ActionDependency[];
}
export interface SimpleAction { name?: string; type?: SimpleActionDef; state?: number }
export interface SimpleActionsMsg { actions?: SimpleAction[] }

export interface ComplexAction { name?: string; type?: string }
export interface ComplexActionsMsg { actions?: ComplexAction[] }

export interface VideoStream { url?: string }
export interface VideoStreamsMsg { stream?: VideoStream[] }

export interface FileDefinitionEntry { identifier?: string; path?: string }
export interface FileDefinitionMsg { file?: FileDefinitionEntry[]; isFolder?: boolean[] }
export interface TransferredFile { path?: string; data?: string }
export interface FolderMsg { file?: TransferredFile[] }

export interface RobotModelInformationMsg {
	filedef?: FileDefinitionMsg;
	modelfilename?: string;
}

export interface ImageMsg {
	header?: Header;
	encoding?: string;
	width?: number;
	height?: number;
	step?: number;
	isBigendian?: boolean;
	data?: string;
}
export interface ImageLayersMsg { layers?: ImageMsg[] }

export interface SimpleSensorMsg { name?: string; value?: number[]; valueNames?: string[]; size?: Vector3Msg }
export interface IMUMsg { header?: Header; acceleration?: Vector3Msg; gyro?: Vector3Msg; mag?: Vector3Msg }
export interface WrenchMsg { header?: Header; force?: Vector3Msg; torque?: Vector3Msg }
export interface WrenchStateMsg { wrenches?: WrenchMsg[] }

export interface PermissionRequestMsg { requestuid?: string; description?: string }
export type InterfaceOptionsMsg = Record<string, boolean | undefined>;
export interface StringMsg { value?: string }
