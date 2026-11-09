<script lang="ts">
	// a key or gamepad button as icon (Kenney input prompts of svelte-gamepad-virtual-joystick), text if there is none
	import { Icon } from 'svelte-gamepad-virtual-joystick';
	import { keyLabel } from '#lib/input-config.svelte.ts';

	interface Props {
		key?: string;
		/** gamepad button index or a named input of the icon set (e.g. 'dpad_vertical') */
		button?: number | string;
		size?: number;
	}

	let { key, button, size = 26 }: Props = $props();

	// the icons replace white in the svg with this color, the style gives it the color of the theme
	const ICON_COLOR = '#d6dadf';
	// the arrow key icons show all four arrows and look like a d-pad, text is clearer
	const TEXT_ONLY = new Set(['arrowup', 'arrowdown', 'arrowleft', 'arrowright']);

	const label = $derived(key !== undefined ? keyLabel(key) : typeof button === 'string' ? button.replace('_', ' ') : `Button ${button}`);
</script>

<span class="input-icon" title={key !== undefined ? `Key ${label}` : `Gamepad ${label.toLowerCase()}`}>
	<span class="glyph"
		>{#if key !== undefined && !TEXT_ONLY.has(key)}<Icon type="keyboard_mouse" input={key} width={size} height={size} color={ICON_COLOR} />{:else if button !== undefined}<Icon
				type="generic"
				input={button}
				width={size}
				height={size}
				color={ICON_COLOR}
			/>{/if}</span
	><kbd class="fallback">{label}</kbd>
</span>

<style>
	.input-icon {
		display: inline-flex;
		align-items: center;
		line-height: 1;
	}

	/* the svg keeps its 64 px size attributes, only its content is scaled */
	.glyph {
		display: inline-flex;
		overflow: hidden;
	}

	.glyph :global(svg) {
		display: block;
		width: 100%;
		height: 100%;
	}

	.glyph :global(path[fill='#d6dadf']) {
		fill: var(--np-color-on-surface);
	}

	.glyph:empty {
		display: none;
	}

	/* the text is only shown when there is no icon for the input */
	.glyph:not(:empty) + .fallback {
		display: none;
	}

	.fallback {
		min-width: 1.6em;
		padding: 2px 6px;
		text-align: center;
		border-radius: 4px;
		font: inherit;
		font-size: 0.75rem;
		border: 1px solid var(--np-color-outline);
	}
</style>
