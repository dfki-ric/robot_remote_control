<script lang="ts">
	// the driven path of the robot
	import { untrack } from 'svelte';
	import { getSession } from '#lib/session/session.svelte.ts';
	import { getViewerState } from '#lib/viewer/viewer-state.svelte.ts';
	import LineLayer from './LineLayer.svelte';

	const MAX_POINTS = 5000;
	const MIN_DISTANCE = 0.05;

	const session = getSession();
	const viewer = getViewerState();

	let points: number[] = [];
	let positions = $state.raw(new Float32Array());

	$effect(() => {
		void viewer.trailClears;
		points = [];
		positions = new Float32Array();
	});

	$effect(() => {
		const p = session.telemetry.pose?.position;
		if (!p) {
			return;
		}
		untrack(() => {
			const [x, y, z] = [p.x ?? 0, p.y ?? 0, (p.z ?? 0) + 0.01];
			const last = points.length - 3;
			if (last >= 0 && Math.hypot(x - points[last], y - points[last + 1], z - points[last + 2]) < MIN_DISTANCE) {
				return;
			}
			points.push(x, y, z);
			if (points.length > MAX_POINTS * 3) {
				points.splice(0, 3);
			}
			positions = new Float32Array(points);
		});
	});
</script>

<LineLayer {positions} color={0x7d8794} visible={viewer.layers.trail} />
