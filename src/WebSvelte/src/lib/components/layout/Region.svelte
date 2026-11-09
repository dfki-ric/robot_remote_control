<script lang="ts">
	// a region above or below the 3D view, resized with its splitter
	import type { Snippet } from 'svelte';
	import { MIN_REGION_HEIGHT, MIN_STAGE_HEIGHT, type RegionLayout } from '#lib/layout.svelte.ts';

	interface Props {
		layout: RegionLayout;
		/** above the 3D view: the splitter is below the region */
		above: boolean;
		/** the area containing the 3D view and the regions */
		area: HTMLElement | undefined;
		/** height of the other visible regions */
		reserved: () => number;
		header: Snippet;
		children: Snippet;
		element?: HTMLElement;
	}

	let { layout, above, area, reserved, header, children, element = $bindable() }: Props = $props();

	let dragging = $state(false);

	function resize(pointerY: number, remember: boolean) {
		if (!area) {
			return;
		}
		const rect = area.getBoundingClientRect();
		const wanted = above ? pointerY - rect.top : rect.bottom - pointerY;
		const height = Math.min(Math.max(wanted, MIN_REGION_HEIGHT), rect.height - reserved() - MIN_STAGE_HEIGHT);
		layout.setHeight(height, remember);
	}
</script>

{#snippet splitter()}
	<div
		class={['splitter', dragging && 'dragging']}
		role="separator"
		aria-orientation="horizontal"
		title="Drag to resize"
		onpointerdown={(event) => {
			event.currentTarget.setPointerCapture(event.pointerId);
			dragging = true;
		}}
		onpointermove={(event) => dragging && resize(event.clientY, false)}
		onpointerup={(event) => {
			if (dragging) {
				dragging = false;
				resize(event.clientY, true);
			}
		}}
		onpointercancel={() => (dragging = false)}
	></div>
{/snippet}

{#if layout.visible}
	{#if !above}{@render splitter()}{/if}
	<section class="region" bind:this={element} style:height={layout.height ? `${layout.height}px` : null}>
		<header class="region-header">{@render header()}</header>
		<div class="region-content">{@render children()}</div>
	</section>
	{#if above}{@render splitter()}{/if}
{/if}

<style>
	.region {
		display: flex;
		flex-direction: column;
		flex-shrink: 0;
		height: 220px;
		min-height: 0;
		background: var(--np-color-surface-container-low);
	}

	.region-header {
		display: flex;
		align-items: center;
		gap: 12px;
		min-height: 44px;
		padding: 4px 12px;
		border-bottom: 1px solid var(--np-color-outline-variant);
	}

	.region-content {
		flex: 1;
		min-height: 0;
		overflow: auto;
	}

	.splitter {
		flex-shrink: 0;
		height: 6px;
		cursor: row-resize;
		touch-action: none;
		background: var(--np-color-outline-variant);
	}

	.splitter:hover,
	.splitter.dragging {
		background: var(--np-color-primary);
	}
</style>
