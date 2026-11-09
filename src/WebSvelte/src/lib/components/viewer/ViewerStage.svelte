<script lang="ts">
	// the 3D view with its toolbar and overlays
	import { Canvas } from '@threlte/core';
	import { Button, Checkbox } from 'noph-ui';
	import { getDrive } from '#lib/drive.svelte.ts';
	import { getLayout } from '#lib/layout.svelte.ts';
	import { degrees, format, yawOf } from '#lib/rrc/util.ts';
	import { getSession } from '#lib/session/session.svelte.ts';
	import { getViewerState, LAYER_LABELS, type Layer } from '#lib/viewer/viewer-state.svelte.ts';
	import Scene from './Scene.svelte';

	const session = getSession();
	const viewer = getViewerState();
	const drive = getDrive();
	const layout = getLayout();

	const layers = Object.keys(LAYER_LABELS) as Layer[];

	const poseText = $derived.by(() => {
		const pose = session.telemetry.pose;
		if (!pose) {
			return 'no pose';
		}
		const p = pose.position ?? {};
		const yaw = format(degrees(yawOf(pose.orientation)), 1);
		return `x ${format(p.x ?? 0)}  y ${format(p.y ?? 0)}  z ${format(p.z ?? 0)}  yaw ${yaw}°`;
	});

	const message = $derived(
		session.unreachable ? `The robot does not answer at ${session.rrc.commandUrl}.` : session.modelStatus
	);

	function toggleGoalMode() {
		if (viewer.pickMode === 'goal') {
			viewer.setPickMode(null);
			return;
		}
		viewer.setPickMode('goal', (x, y, yaw) => {
			viewer.setPickMode(null);
			void session.sendGoal(x, y, yaw, drive.linearSpeed);
		});
	}
</script>

<div class="stage">
	<div class="canvas">
		<Canvas renderMode="always">
			<Scene />
		</Canvas>
	</div>

	<div class="toolbar" role="toolbar" aria-label="3D view">
		<Button
			size="xs"
			variant={viewer.pickMode === 'goal' ? 'filled' : 'tonal'}
			title="Drag on the ground to set a goal and its orientation"
			onclick={toggleGoalMode}>Set goal</Button
		>
		<Button size="xs" variant="tonal" toggle bind:selected={viewer.follow} title="Keep the robot in view">Follow</Button>
		<Button size="xs" variant="tonal" onclick={() => viewer.resetRequests++}>Reset view</Button>
		<span class="separator"></span>
		{#each layers as layer (layer)}
			<label class="layer"><Checkbox bind:checked={viewer.layers[layer]} />{LAYER_LABELS[layer]}</label>
		{/each}
		<Button size="xs" variant="text" title="Clear the driven path" onclick={() => viewer.trailClears++}>Clear trail</Button>
		<span class="separator"></span>
		<Button size="xs" variant="tonal" toggle selected={layout.images.visible} onclick={() => layout.images.setVisible(!layout.images.visible)}>Images</Button>
		<Button size="xs" variant="tonal" toggle selected={layout.log.visible} onclick={() => layout.log.setVisible(!layout.log.visible)}>Log</Button>
	</div>

	<div class={['pose', viewer.pickHint && 'raised']}>{poseText}</div>
	{#if message}
		<div class="message">{message}</div>
	{/if}
	{#if viewer.pickHint}
		<div class="pick-hint" role="status">{viewer.pickHint}</div>
	{/if}
</div>

<style>
	.stage {
		position: relative;
		flex: 1;
		min-height: 260px;
		overflow: hidden;
	}

	.canvas {
		position: absolute;
		inset: 0;
		background: var(--np-color-surface-dim);
	}

	.toolbar {
		position: absolute;
		top: 8px;
		left: 8px;
		right: 8px;
		display: flex;
		flex-wrap: wrap;
		align-items: center;
		gap: 6px;
		pointer-events: none;
	}

	.toolbar > :global(*) {
		pointer-events: auto;
	}

	.layer {
		display: inline-flex;
		align-items: center;
		padding-right: 6px;
		border-radius: 16px;
		font-size: 0.8125rem;
		background: color-mix(in srgb, var(--np-color-surface-container) 85%, transparent);
	}

	.separator {
		width: 1px;
		height: 24px;
		margin: 0 4px;
		background: var(--np-color-outline-variant);
	}

	.pose,
	.message {
		position: absolute;
		padding: 4px 10px;
		border-radius: 8px;
		font-size: 0.8125rem;
		background: color-mix(in srgb, var(--np-color-surface-container) 85%, transparent);
		pointer-events: none;
	}

	.pose {
		left: 8px;
		bottom: 8px;
		transition: bottom 0.15s;
		font-variant-numeric: tabular-nums;
		white-space: pre;
	}

	.message {
		left: 50%;
		top: 50%;
		transform: translate(-50%, -50%);
		max-width: 80%;
		color: var(--np-color-on-surface);
	}

	.pose.raised {
		bottom: 36px;
	}

	@media (max-width: 900px) {
		.toolbar {
			flex-wrap: nowrap;
			overflow-x: auto;
			pointer-events: auto;
			scrollbar-width: none;
		}

		.toolbar > :global(*) {
			flex-shrink: 0;
		}
	}

	/* transparent, centered at the bottom of the 3D view, readable on any background */
	.pick-hint {
		position: absolute;
		left: 50%;
		bottom: 8px;
		transform: translateX(-50%);
		max-width: calc(100% - 16px);
		padding: 4px 10px;
		font-size: 0.8125rem;
		text-align: center;
		color: var(--np-color-on-surface);
		text-shadow: 0 0 3px #000, 0 0 6px #000;
		background: transparent;
		pointer-events: none;
	}
</style>
