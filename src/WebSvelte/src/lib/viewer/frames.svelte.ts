/**
 * Transforms of the frames used in telemetry headers into the world group of the scene (robot coordinates):
 * world frames and unknown frames are the world, link frames are placed by the current pose and joint values.
 */
import { createContext } from 'svelte';
import { Matrix4, Quaternion, Vector3 } from 'three';
import type { PoseMsg, QuaternionMsg, Vector3Msg } from '../rrc/protocol';
import type { RobotSession } from '../session/session.svelte';

export function toVector3(v: Vector3Msg | undefined): Vector3 {
	return new Vector3(v?.x ?? 0, v?.y ?? 0, v?.z ?? 0);
}

export function toQuaternion(q: QuaternionMsg | undefined): Quaternion {
	const result = new Quaternion(q?.x ?? 0, q?.y ?? 0, q?.z ?? 0, q?.w ?? 0);
	// an unset orientation is all zero
	return result.lengthSq() < 1e-9 ? new Quaternion() : result.normalize();
}

export function poseMatrix(pose: PoseMsg | null | undefined): Matrix4 {
	return new Matrix4().compose(toVector3(pose?.position), toQuaternion(pose?.orientation), new Vector3(1, 1, 1));
}

export interface Placement {
	position: Vector3;
	quaternion: Quaternion;
}

export class RobotFrames {
	readonly robotPose: Matrix4;
	private readonly linkTransforms: Map<string, Matrix4>;

	constructor(private readonly session: RobotSession) {
		this.robotPose = $derived(poseMatrix(session.telemetry.pose));
		this.linkTransforms = $derived(
			session.model?.kinematics.linkTransforms(session.telemetry.jointPositions) ?? new Map<string, Matrix4>()
		);
	}

	/**
	 * @returns the placement of a frame in the world group, reactive to pose and joint changes for link frames
	 */
	placement(frame: string | undefined): Placement {
		const placement = { position: new Vector3(), quaternion: new Quaternion() };
		const link = this.session.model?.kinematics.findLink(frame) ?? null;
		if (link === null) {
			return placement;
		}
		const matrix = this.robotPose.clone().multiply(this.linkTransforms.get(link) ?? new Matrix4());
		matrix.decompose(placement.position, placement.quaternion, new Vector3());
		return placement;
	}
}

export const [getRobotFrames, setRobotFrames] = createContext<RobotFrames>();
