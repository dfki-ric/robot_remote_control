<script lang="ts">
	import { Button } from 'noph-ui';
	import { getLayout } from '#lib/layout.svelte.ts';
	import { getSession } from '#lib/session/session.svelte.ts';
	import ImageView from './ImageView.svelte';

	const session = getSession();
	const layout = getLayout();
	const sources = session.telemetry.images;


	export function addView() {
		// prefer a source that is not shown yet
		const shown = new Set(layout.imageViews.map((view) => view.key));
		const keys = sources.list.map((source) => source.key);
		layout.addImageView(keys.find((key) => !shown.has(key)) ?? keys[0] ?? '');
	}
</script>

<div class="image-views">
	{#each layout.imageViews as view (view.id)}
		<ImageView
			{sources}
			key={view.key}
			shown={layout.images.visible}
			onselect={(key) => layout.setImageView(view.id, key)}
			onremove={() => layout.removeImageView(view.id)}
		/>
	{:else}
		<p class="empty">No image views yet.</p>
		<Button variant="text" onclick={addView}>Add a view</Button>
	{/each}
</div>

<style>
	.image-views {
		display: flex;
		gap: 12px;
		height: 100%;
		padding: 8px 12px;
		overflow-x: auto;
		align-items: flex-start;
	}

	.empty {
		margin: 8px 0;
		color: var(--np-color-on-surface-variant);
	}
</style>
