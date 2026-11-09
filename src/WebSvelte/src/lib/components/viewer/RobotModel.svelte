<script lang="ts">
	// the urdf model (urdf-viewer) at the current pose with the current joint values
	import { T } from '@threlte/core';
	import { untrack } from 'svelte';
	import type { Group } from 'three';
	import { UrdfThree, urdf_viewer_state } from 'urdf-viewer';
	import { getSession } from '#lib/session/session.svelte.ts';
	import { toQuaternion, toVector3 } from '#lib/viewer/frames.svelte.ts';
	import { getViewerState } from '#lib/viewer/viewer-state.svelte.ts';

	interface Props {
		root?: Group;
	}

	let { root = $bindable() }: Props = $props();

	const session = getSession();
	const viewer = getViewerState();

	const pose = $derived(session.telemetry.pose);
	const position = $derived(toVector3(pose?.position).toArray());
	const quaternion = $derived(toQuaternion(pose?.orientation).toArray());

	// urdf-viewer is an editor, this is a view: no selection highlight, names and joint axes only on request
	urdf_viewer_state.edit = false;

	$effect(() => {
		const shown = viewer.layers.frames;
		urdf_viewer_state.joints = shown;
		urdf_viewer_state.jointNames = shown;
		urdf_viewer_state.linkNames = shown;
	});

	/** joint name -> index in the joints of the viewer robot */
	let jointIndex = new Map<string, number>();

	$effect(() => {
		const robot = session.model?.robot;
		urdf_viewer_state.selectedLink = undefined;
		urdf_viewer_state.selectedJoint = undefined;
		urdf_viewer_state.robot = robot;
		jointIndex = new Map((robot?.joints ?? []).map((joint, i) => [joint.name ?? '', i]));
	});

	$effect(() => {
		const positions = session.telemetry.jointPositions;
		const kinematics = session.model?.kinematics;
		untrack(() => {
			// write through the reactive viewer state, the viewer components update from it
			const joints = urdf_viewer_state.robot?.joints;
			if (!kinematics || !joints) {
				return;
			}
			for (const [name, value] of positions) {
				const index = jointIndex.get(name);
				const viewerPose = kinematics.viewerPose(name, value);
				if (index === undefined || !viewerPose) {
					continue;
				}
				if (viewerPose.rotation) {
					joints[index].rotation = viewerPose.rotation;
				}
				if (viewerPose.origin) {
					joints[index].origin_xyz = viewerPose.origin;
				}
			}
		});
	});
</script>

<T.Group bind:ref={root} {position} {quaternion}>
	<T.AxesHelper args={[0.5]} />
	{#if viewer.layers.robot}
		<UrdfThree />
	{/if}
</T.Group>
