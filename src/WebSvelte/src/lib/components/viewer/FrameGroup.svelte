<script lang="ts">
	// places its children in a frame of a telemetry header (world or a link of the robot)
	import { T } from '@threlte/core';
	import type { Snippet } from 'svelte';
	import { getRobotFrames } from '#lib/viewer/frames.svelte.ts';

	interface Props {
		frame: string | undefined;
		children: Snippet;
	}

	let { frame, children }: Props = $props();

	const frames = getRobotFrames();
	const placement = $derived(frames.placement(frame));
</script>

<T.Group position={placement.position.toArray()} quaternion={placement.quaternion.toArray()}>
	{@render children()}
</T.Group>
