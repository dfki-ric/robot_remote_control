<script lang="ts">
	// images above, the 3D view, log below
	import { Button } from 'noph-ui';
	import { getLayout } from '#lib/layout.svelte.ts';
	import { SELECTABLE_LOG_LEVELS } from '#lib/rrc/protocol.ts';
	import { getSession } from '#lib/session/session.svelte.ts';
	import ViewerStage from '../viewer/ViewerStage.svelte';
	import ImageViews from './ImageViews.svelte';
	import LogPanel from './LogPanel.svelte';
	import Region from './Region.svelte';

	const session = getSession();
	const layout = getLayout();

	let area = $state<HTMLElement>();
	let imageRegion = $state<HTMLElement>();
	let logRegion = $state<HTMLElement>();
	let imageViews = $state<ImageViews>();
	let logLevel = $state('');

	const sources = session.telemetry.images;

	// first images: show them, unless the user arranged the views or hid the region before
	let autoAdded = false;
	$effect(() => {
		const first = sources.list[0];
		if (!first || autoAdded) {
			return;
		}
		autoAdded = true;
		if (!layout.imageViewsRemembered && layout.imageViews.length === 0) {
			layout.addImageView(first.key);
		}
		if (!layout.images.visibilityRemembered) {
			layout.images.setVisible(true, false);
		}
	});
</script>

<section class="viewer-area" bind:this={area}>
	<Region layout={layout.images} above {area} reserved={() => logRegion?.offsetHeight ?? 0} bind:element={imageRegion}>
		{#snippet header()}
			<strong>Images</strong>
			<Button size="xs" variant="tonal" title="Each view shows one image channel or layer" onclick={() => imageViews?.addView()}>Add view</Button>
			{#if session.videoStreams.length}
				<span class="streams">
					Video streams:
					{#each session.videoStreams as url (url)}
						<a href={url} target="_blank" rel="noreferrer">{url}</a>
					{/each}
				</span>
			{/if}
		{/snippet}
		<ImageViews bind:this={imageViews} />
	</Region>

	<ViewerStage />

	<Region layout={layout.log} above={false} {area} reserved={() => imageRegion?.offsetHeight ?? 0} bind:element={logRegion}>
		{#snippet header()}
			<strong>Log</strong>
			<select
				class="level"
				bind:value={logLevel}
				title="Log level of the robot"
				onchange={() => session.setLogLevel(logLevel)}
			>
				<option value="" disabled>Robot log level</option>
				{#each SELECTABLE_LOG_LEVELS as level (level)}
					<option>{level}</option>
				{/each}
			</select>
			<Button size="xs" variant="text" onclick={() => session.telemetry.clearLog()}>Clear</Button>
		{/snippet}
		<LogPanel />
	</Region>
</section>

<style>
	.viewer-area {
		display: flex;
		flex-direction: column;
		flex: 1;
		min-width: 0;
		min-height: 0;
	}

	.streams {
		display: flex;
		gap: 8px;
		font-size: 0.8125rem;
		color: var(--np-color-on-surface-variant);
		overflow: hidden;
		white-space: nowrap;
	}

	.streams a {
		color: var(--np-color-primary);
	}

	.level {
		font: inherit;
		padding: 2px 6px;
		color: var(--np-color-on-surface);
		background: var(--np-color-surface-container-high);
		border: 1px solid var(--np-color-outline-variant);
		border-radius: 6px;
	}
</style>
