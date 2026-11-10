<script lang="ts">
	// scene of the 3D view, everything robot related is in the world group in robot coordinates (z up)
	import { T, useTask } from '@threlte/core';
	import { Grid, OrbitControls } from '@threlte/extras';
	import { untrack } from 'svelte';
	import { Group, PerspectiveCamera, Vector3 } from 'three';
	import type { OrbitControls as OrbitControlsImpl } from 'three/examples/jsm/controls/OrbitControls.js';
	import { getSession } from '#lib/session/session.svelte.ts';
	import { RobotFrames, setRobotFrames } from '#lib/viewer/frames.svelte.ts';
	import { cloudPositions, laserPositions, pathPositions } from '#lib/viewer/geometry.ts';
	import { toQuaternion, toVector3 } from '#lib/viewer/frames.svelte.ts';
	import { getViewerState } from '#lib/viewer/viewer-state.svelte.ts';
	import FrameGroup from './FrameGroup.svelte';
	import GroundPicker from './GroundPicker.svelte';
	import LineLayer from './LineLayer.svelte';
	import MapLayerView from './MapLayerView.svelte';
	import PointsLayer from './PointsLayer.svelte';
	import RobotModel from './RobotModel.svelte';
	import Trail from './Trail.svelte';

	const CAMERA_OFFSET = new Vector3(-3.5, 3, 3.5);
	const TARGET_OFFSET = new Vector3(0, 0.4, 0);

	const session = getSession();
	const viewer = getViewerState();
	setRobotFrames(new RobotFrames(session));

	let camera = $state<PerspectiveCamera>();
	let controls = $state<OrbitControlsImpl>();
	let robotRoot = $state<Group>();

	const telemetry = session.telemetry;
	const path = $derived(pathPositions(telemetry.path));
	const laser = $derived(laserPositions(telemetry.laserScan));
	const cloud = $derived(cloudPositions(telemetry.pointCloud));
	const maps = $derived(Object.entries(session.maps.status).filter(([, status]) => status.layer));

	// keep the robot in view: move the camera with the robot
	let lastRobotPosition: Vector3 | null = null;
	useTask(() => {
		if (!viewer.follow || !robotRoot || !camera || !controls) {
			lastRobotPosition = null;
			return;
		}
		const position = robotRoot.getWorldPosition(new Vector3());
		if (lastRobotPosition) {
			const delta = position.clone().sub(lastRobotPosition);
			camera.position.add(delta);
			controls.target.add(delta);
		}
		lastRobotPosition = position;
	});

	function resetView() {
		if (!robotRoot || !camera || !controls) {
			return;
		}
		const position = robotRoot.getWorldPosition(new Vector3());
		controls.target.copy(position).add(TARGET_OFFSET);
		camera.position.copy(position).add(CAMERA_OFFSET);
		lastRobotPosition = null;
	}

	$effect(() => {
		if (viewer.resetRequests > 0) {
			untrack(resetView);
		}
	});
</script>

<!-- the renderer is transparent: the background is the theme color of the canvas container -->
<T.PerspectiveCamera makeDefault bind:ref={camera} position={CAMERA_OFFSET.toArray()} fov={50} near={0.05} far={1000}>
	<OrbitControls bind:ref={controls} enableDamping target={TARGET_OFFSET.toArray()} />
</T.PerspectiveCamera>

<T.HemisphereLight args={[0xffffff, 0x404850, 2.2]} />
<T.DirectionalLight position={[5, 10, 6]} intensity={2} />

<!-- ground in scene coordinates: 1 m cells, 5 m sections -->
<Grid
	plane="xz"
	cellSize={1}
	sectionSize={5}
	cellColor="#3a424c"
	sectionColor="#56606c"
	cellThickness={1}
	sectionThickness={1.2}
	infiniteGrid
	fadeDistance={60}
	position.y={-0.001}
/>

<T.Group rotation.x={-Math.PI / 2}>
	<T.AxesHelper args={[1]} />

	<RobotModel bind:root={robotRoot} />
	<Trail />

	<FrameGroup frame={telemetry.path?.header?.frame}>
		<LineLayer positions={path} color={0x4fc3f7} visible={viewer.layers.path} />
	</FrameGroup>
	<FrameGroup frame={telemetry.laserScan?.header?.frame}>
		<PointsLayer positions={laser} color={0xff5a5a} size={0.05} visible={viewer.layers.laser} />
	</FrameGroup>
	<FrameGroup frame={telemetry.pointCloud?.header?.frame}>
		<T.Group
			position={toVector3(telemetry.pointCloud?.origin?.position).toArray()}
			quaternion={toQuaternion(telemetry.pointCloud?.origin?.orientation).toArray()}
		>
			<PointsLayer positions={cloud} color={0xffb74d} size={0.04} visible={viewer.layers.cloud} />
		</T.Group>
	</FrameGroup>

	{#each maps as [channel, status] (channel)}
		{#if status.layer}
			<MapLayerView layer={status.layer} />
		{/if}
	{/each}

	<GroundPicker {controls} />
</T.Group>
