<script lang="ts">
	// picks goals (click, drag for the orientation) and task points on the ground, shows them
	import { T, useThrelte } from '@threlte/core';
	import { ArrowHelper, Group, Plane, Raycaster, Vector2, Vector3 } from 'three';
	import type { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
	import { getViewerState } from '#lib/viewer/viewer-state.svelte.ts';
	import LineLayer from './LineLayer.svelte';

	interface Props {
		/** disabled while dragging a goal */
		controls: OrbitControls | undefined;
	}

	let { controls }: Props = $props();

	const MIN_DRAG = 0.15;
	const WAYPOINT_COLOR = 0xba68c8;

	const { dom, camera } = useThrelte();
	const viewer = getViewerState();

	// the ground in scene coordinates (y up), this group is in robot coordinates
	const ground = new Plane(new Vector3(0, 1, 0), 0);
	const raycaster = new Raycaster();
	const arrow = new ArrowHelper(new Vector3(1, 0, 0), new Vector3(), 0.8, 0x66bb6a, 0.25, 0.15);
	let frame = $state<Group>();
	let arrowVisible = $state(false);
	let pick: { start: Vector3; yaw: number | null } | null = null;

	const waypointLine = $derived(new Float32Array(viewer.waypoints.flatMap((p) => [p.x, p.y, 0.05])));

	function groundPoint(event: PointerEvent): Vector3 | null {
		const rect = dom.getBoundingClientRect();
		const pointer = new Vector2(
			((event.clientX - rect.left) / rect.width) * 2 - 1,
			-((event.clientY - rect.top) / rect.height) * 2 + 1
		);
		raycaster.setFromCamera(pointer, camera.current);
		const hit = raycaster.ray.intersectPlane(ground, new Vector3());
		return hit && frame ? frame.worldToLocal(hit) : null;
	}

	$effect(() => {
		const down = (event: PointerEvent) => {
			if (!viewer.pickMode || event.button !== 0) {
				return;
			}
			const point = groundPoint(event);
			if (!point) {
				return;
			}
			// runs before the orbit controls (capture), so they ignore this drag
			if (controls) {
				controls.enabled = false;
			}
			dom.setPointerCapture(event.pointerId);
			pick = { start: point, yaw: null };
			if (viewer.pickMode === 'goal') {
				arrow.position.set(point.x, point.y, 0.05);
				arrow.setDirection(new Vector3(1, 0, 0));
				arrowVisible = true;
			}
		};
		const move = (event: PointerEvent) => {
			if (!pick || viewer.pickMode !== 'goal') {
				return;
			}
			const point = groundPoint(event);
			if (point && point.distanceTo(pick.start) > MIN_DRAG) {
				pick.yaw = Math.atan2(point.y - pick.start.y, point.x - pick.start.x);
				arrow.setDirection(new Vector3(Math.cos(pick.yaw), Math.sin(pick.yaw), 0));
			}
		};
		const up = () => {
			if (!pick) {
				return;
			}
			const { start, yaw } = pick;
			pick = null;
			if (controls) {
				controls.enabled = true;
			}
			viewer.picked(start.x, start.y, yaw);
		};
		dom.addEventListener('pointerdown', down, { capture: true });
		dom.addEventListener('pointermove', move);
		dom.addEventListener('pointerup', up);
		dom.addEventListener('pointercancel', up);
		return () => {
			dom.removeEventListener('pointerdown', down, { capture: true });
			dom.removeEventListener('pointermove', move);
			dom.removeEventListener('pointerup', up);
			dom.removeEventListener('pointercancel', up);
		};
	});

	$effect(() => {
		dom.style.cursor = viewer.pickMode ? 'crosshair' : '';
		if (viewer.pickMode !== 'goal') {
			arrowVisible = false;
		}
	});
</script>

<T.Group bind:ref={frame}>
	<T is={arrow} visible={arrowVisible} />
	{#each viewer.waypoints as point, i (i)}
		<T.Mesh position={[point.x, point.y, 0.05]}>
			<T.SphereGeometry args={[0.08, 12, 8]} />
			<T.MeshBasicMaterial color={WAYPOINT_COLOR} />
		</T.Mesh>
	{/each}
	<LineLayer positions={waypointLine} color={WAYPOINT_COLOR} />
</T.Group>
