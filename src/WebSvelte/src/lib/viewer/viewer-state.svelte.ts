/**
 * State of the 3D view shared with the panels that use it: visible layers, camera following and picking
 * points on the ground (goals, task points).
 */
import { createContext } from 'svelte';

export type PickMode = 'goal' | 'waypoints';
export type Layer = 'robot' | 'frames' | 'trail' | 'path' | 'laser' | 'cloud';

/** robot coordinates on the ground, yaw only when dragged (goals) */
export type PickCallback = (x: number, y: number, yaw: number | null) => void;

export interface GroundPoint {
	x: number;
	y: number;
}

export const LAYER_LABELS: Record<Layer, string> = {
	robot: 'Robot', frames: 'Frames', trail: 'Trail', path: 'Path', laser: 'Laser', cloud: 'Point cloud'
};

export class ViewerState {
	layers = $state<Record<Layer, boolean>>({
		robot: true, frames: false, trail: true, path: true, laser: true, cloud: true
	});
	follow = $state(true);
	pickMode = $state<PickMode | null>(null);
	/** task points shown in the view */
	waypoints = $state.raw<GroundPoint[]>([]);
	/** incremented to request a camera reset / clearing the trail */
	resetRequests = $state(0);
	trailClears = $state(0);
	private pickCallback: PickCallback | null = null;

	get pickHint(): string {
		switch (this.pickMode) {
			case 'goal':
				return 'Click on the ground to set a goal, drag to set its orientation. Esc cancels.';
			case 'waypoints':
				return 'Click on the ground to add points. Esc finishes.';
			default:
				return '';
		}
	}

	setPickMode(mode: PickMode | null, callback: PickCallback | null = null) {
		if (mode !== null && callback === null) {
			throw new Error(`pick mode ${mode} needs a callback`);
		}
		this.pickMode = mode;
		this.pickCallback = callback;
	}

	picked(x: number, y: number, yaw: number | null) {
		this.pickCallback?.(x, y, yaw);
	}
}

export const [getViewerState, setViewerState] = createContext<ViewerState>();
