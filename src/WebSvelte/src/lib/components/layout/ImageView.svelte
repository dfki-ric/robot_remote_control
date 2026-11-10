<script lang="ts">
	import { NativeSelect } from 'noph-ui';
	import type { ImageSources } from '#lib/session/images.svelte.ts';
	import { format, throttle } from '#lib/rrc/util.ts';
	import { drawImage } from '#lib/viewer/draw-image.ts';

	interface Props {
		sources: ImageSources;
		key: string;
		/** images are only drawn while shown */
		shown: boolean;
		onselect: (key: string) => void;
		onremove: () => void;
	}

	let { sources, key, shown, onselect, onremove }: Props = $props();

	let canvas = $state<HTMLCanvasElement>();
	let info = $state('');

	const draw = throttle(() => {
		const source = sources.get(key);
		if (!source || !canvas || !shown) {
			return;
		}
		drawImage(canvas, source.image)
			.then((details) => (info = `${details}, ${format(source.rate, 1)} Hz`))
			.catch((error: Error) => (info = error.message));
	}, 100);

	$effect(() => {
		// redraw when the source or the visibility changes
		void key;
		if (shown) {
			draw();
		}
	});

	$effect(() => sources.subscribe((updated) => updated === key && draw()));

	// views restored from the last session may show sources that were not received yet
	const options = $derived.by(() => {
		const list = sources.list.map((source) => ({ value: source.key, label: source.label }));
		if (key && !list.some((option) => option.value === key)) {
			list.push({ value: key, label: `${key} (waiting)` });
		}
		return list;
	});
</script>

<figure class="image-view">
	<div class="image-view-header">
		<NativeSelect
			value={key}
			onchange={(event) => onselect((event.currentTarget as HTMLSelectElement).value)}
			aria-label="Image channel or layer"
			style="--np-select-min-width: 0; flex: 1"
		>
			{#each options as option (option.value)}
				<option value={option.value}>{option.label}</option>
			{:else}
				<option value="">no images</option>
			{/each}
		</NativeSelect>
		<button class="close" type="button" title="Remove this view" aria-label="Remove this view" onclick={onremove}>×</button>
	</div>
	<canvas bind:this={canvas} width="320" height="240"></canvas>
	<figcaption>{info}</figcaption>
</figure>

<style>
	.image-view {
		display: flex;
		flex-direction: column;
		gap: 4px;
		min-width: 220px;
		max-width: 640px;
		flex: 1;
		height: 100%;
		margin: 0;
	}

	.image-view-header {
		display: flex;
		align-items: center;
		gap: 4px;
	}

	.close {
		border: none;
		background: none;
		color: var(--np-color-on-surface-variant);
		font-size: 1.4rem;
		cursor: pointer;
		padding: 0 8px;
	}

	canvas {
		flex: 1;
		min-height: 0;
		width: 100%;
		object-fit: contain;
		background: #000;
		border-radius: 4px;
	}

	figcaption {
		font-size: 0.75rem;
		color: var(--np-color-on-surface-variant);
	}
</style>
