/**
 * Forward kinematics of a urdf model, to place data given in link frames (laser scans, point clouds) and to
 * show joint values in the urdf-viewer model. Uses the original joint origins: the viewer pose of prismatic
 * joints is applied by moving their origin.
 */
import { Euler, Matrix4, Quaternion, Vector3 } from 'three';
import type { IUrdfJoint, IUrdfRobot } from 'urdf-viewer';

export interface JointLimits {
	type: string;
	lower: number;
	upper: number;
}

interface JointInfo {
	name: string;
	type: string;
	parent: string;
	child: string;
	originPosition: Vector3;
	originQuaternion: Quaternion;
	axis: Vector3;
	limits: JointLimits;
}

/** frames given in the world or map frame of the robot */
const WORLD_FRAMES = new Set(['', 'world', 'map', 'odom']);

function originQuaternion(rpy: [number, number, number]): Quaternion {
	// urdf rpy: fixed axes x, y, z = euler ZYX, the same order urdf-viewer uses
	return new Quaternion().setFromEuler(new Euler(rpy[0], rpy[1], rpy[2], 'ZYX'));
}

/**
 * the limits are read from the xml, UrdfParser rounds them to integers
 */
function readLimits(joint: IUrdfJoint): JointLimits {
	const type = joint.type ?? 'fixed';
	const element = joint.elem.getElementsByTagName('limit')[0];
	const lower = Number.parseFloat(element?.getAttribute('lower') ?? '');
	const upper = Number.parseFloat(element?.getAttribute('upper') ?? '');
	if (type === 'continuous' || !(upper > lower)) {
		return { type, lower: -Math.PI, upper: Math.PI };
	}
	return { type, lower, upper };
}

export class RobotKinematics {
	private readonly joints = new Map<string, JointInfo>();
	private readonly childJoints = new Map<string, JointInfo[]>();
	private readonly roots: string[];
	readonly linkNames: string[];

	constructor(robot: IUrdfRobot) {
		this.linkNames = Object.keys(robot.links);
		const children = new Set<string>();
		for (const joint of robot.joints) {
			if (!joint.name) {
				continue;
			}
			const info: JointInfo = {
				name: joint.name,
				type: joint.type ?? 'fixed',
				parent: joint.parent.name,
				child: joint.child.name,
				originPosition: new Vector3(...joint.origin_xyz),
				originQuaternion: originQuaternion(joint.origin_rpy),
				axis: new Vector3(...(joint.axis_xyz ?? [0, 0, 1])).normalize(),
				limits: readLimits(joint)
			};
			this.joints.set(info.name, info);
			const siblings = this.childJoints.get(info.parent) ?? [];
			siblings.push(info);
			this.childJoints.set(info.parent, siblings);
			children.add(info.child);
		}
		this.roots = this.linkNames.filter((name) => !children.has(name));
	}

	get movableJointCount(): number {
		return [...this.joints.values()].filter((joint) => joint.type !== 'fixed').length;
	}

	limits(joint: string): JointLimits | null {
		return this.joints.get(joint)?.limits ?? null;
	}

	/**
	 * @returns the link transforms relative to the robot frame (the root links)
	 */
	linkTransforms(values: ReadonlyMap<string, number>): Map<string, Matrix4> {
		const transforms = new Map<string, Matrix4>();
		const local = new Matrix4();
		const motion = new Matrix4();
		const visit = (link: string, transform: Matrix4) => {
			transforms.set(link, transform);
			for (const joint of this.childJoints.get(link) ?? []) {
				local.compose(joint.originPosition, joint.originQuaternion, new Vector3(1, 1, 1));
				const value = values.get(joint.name) ?? 0;
				if (joint.type === 'prismatic') {
					motion.makeTranslation(joint.axis.clone().multiplyScalar(value));
				} else if (joint.type === 'revolute' || joint.type === 'continuous') {
					motion.makeRotationAxis(joint.axis, value);
				} else {
					motion.identity();
				}
				visit(joint.child, transform.clone().multiply(local).multiply(motion));
			}
		};
		this.roots.forEach((root) => visit(root, new Matrix4()));
		return transforms;
	}

	/**
	 * The link of a frame name used in telemetry headers. Robots may prefix the link names
	 * (e.g. "r1_laser_link" for the link "laser_link"), they are matched by the longest suffix.
	 * @returns null for world frames and unknown frames
	 */
	findLink(frame: string | undefined): string | null {
		if (!frame || WORLD_FRAMES.has(frame)) {
			return null;
		}
		if (this.linkNames.includes(frame)) {
			return frame;
		}
		const matches = this.linkNames.filter((name) => frame.endsWith(name)).sort((a, b) => b.length - a.length);
		return matches[0] ?? null;
	}

	/**
	 * How urdf-viewer shows a joint value: UrdfJoint rotates by `joint.rotation` (euler XYZ), translations
	 * are not supported by it, so prismatic joints move their origin.
	 */
	viewerPose(joint: string, value: number): { rotation?: [number, number, number]; origin?: [number, number, number] } | null {
		const info = this.joints.get(joint);
		if (!info) {
			return null;
		}
		if (info.type === 'revolute' || info.type === 'continuous') {
			const euler = new Euler().setFromQuaternion(new Quaternion().setFromAxisAngle(info.axis, value));
			return { rotation: [euler.x, euler.y, euler.z] };
		}
		if (info.type === 'prismatic') {
			const offset = info.axis.clone().multiplyScalar(value).applyQuaternion(info.originQuaternion);
			return { origin: info.originPosition.clone().add(offset).toArray() };
		}
		return null;
	}
}
