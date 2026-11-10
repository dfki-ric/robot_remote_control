<script lang="ts">
	// noph-ui theme of the settings dialog, edited in the draft of the dialog (previewed live)
	import { Button, NativeSelect } from 'noph-ui';
	import {
		CONTRAST_LEVELS, SOURCE_PRESETS, THEME_MODES, THEME_VARIANTS,
		type ThemeConfig, type ThemeMode, type ThemeVariant
	} from '#lib/theme.svelte.ts';

	let { draft = $bindable() }: { draft: ThemeConfig } = $props();

	const modes = Object.entries(THEME_MODES) as [ThemeMode, string][];
	const variants = Object.entries(THEME_VARIANTS) as [ThemeVariant, string][];
	const generated = $derived(draft.variant !== 'default');

	/** roles shown in the preview */
	const SAMPLES = [
		['primary', 'on-primary', 'Primary'],
		['primary-container', 'on-primary-container', 'Primary container'],
		['secondary-container', 'on-secondary-container', 'Secondary'],
		['tertiary', 'on-tertiary', 'Tertiary'],
		['error', 'on-error', 'Error'],
		['surface-container-highest', 'on-surface', 'Surface']
	];
</script>

<div class="content">
	<section>
		<h3>Color scheme</h3>
		<div class="row">
			{#each modes as [mode, label] (mode)}
				<Button size="s" variant={draft.mode === mode ? 'filled' : 'outlined'} onclick={() => (draft.mode = mode)}>{label}</Button>
			{/each}
		</div>
	</section>

	<section>
		<h3>Palette</h3>
		<div class="row">
			<NativeSelect label="Palette" bind:value={draft.variant}>
				{#each variants as [variant, label] (variant)}
					<option value={variant}>{label}</option>
				{/each}
			</NativeSelect>
			<NativeSelect
				label="Contrast"
				value={String(draft.contrast)}
				disabled={!generated}
				onchange={(event) => (draft.contrast = Number((event.currentTarget as HTMLSelectElement).value))}
			>
				{#each Object.entries(CONTRAST_LEVELS) as [level, label] (level)}
					<option value={level}>{label}</option>
				{/each}
			</NativeSelect>
		</div>
		<div class="row sources" aria-disabled={!generated}>
			<span class="small">Source color</span>
			{#each SOURCE_PRESETS as color (color)}
				<button
					type="button"
					class={['swatch', draft.source === color && 'selected']}
					style:background={color}
					title={color}
					aria-label="Source color {color}"
					aria-pressed={draft.source === color}
					disabled={!generated}
					onclick={() => (draft.source = color)}
				></button>
			{/each}
			<label class="custom small">
				<input type="color" bind:value={draft.source} disabled={!generated} />
				{draft.source}
			</label>
		</div>
		{#if !generated}
			<p class="muted small">Choose a palette other than the noph-ui default to generate colors from a source color.</p>
		{/if}
	</section>

	<section>
		<h3>Preview</h3>
		<div class="preview">
			{#each SAMPLES as [background, text, label] (label)}
				<span class="sample" style:background="var(--np-color-{background})" style:color="var(--np-color-{text})">{label}</span>
			{/each}
		</div>
	</section>
</div>

<style>
	.content {
		display: flex;
		flex-direction: column;
		gap: 16px;
	}

	h3 {
		margin: 0 0 8px;
		font-size: 0.9375rem;
		font-weight: 500;
	}

	.row {
		display: flex;
		flex-wrap: wrap;
		align-items: center;
		gap: 12px;
	}

	.sources {
		margin-top: 12px;
		gap: 8px;
	}

	.sources[aria-disabled='true'] {
		opacity: 0.5;
	}

	.swatch {
		width: 32px;
		height: 32px;
		border-radius: 50%;
		border: 2px solid transparent;
		cursor: pointer;
	}

	.swatch.selected {
		outline: 2px solid var(--np-color-on-surface);
		outline-offset: 2px;
	}

	.swatch:focus-visible {
		outline: 2px solid var(--np-color-primary);
		outline-offset: 2px;
	}

	.custom {
		display: inline-flex;
		align-items: center;
		gap: 6px;
		font-variant-numeric: tabular-nums;
	}

	input[type='color'] {
		width: 40px;
		height: 32px;
		padding: 0;
		border: 1px solid var(--np-color-outline-variant);
		border-radius: 6px;
		background: none;
	}

	.preview {
		display: flex;
		flex-wrap: wrap;
		gap: 8px;
	}

	.sample {
		padding: 10px 14px;
		border-radius: 12px;
		font-size: 0.875rem;
	}

	.muted {
		color: var(--np-color-on-surface-variant);
	}

	.small {
		font-size: 0.8125rem;
	}
</style>
