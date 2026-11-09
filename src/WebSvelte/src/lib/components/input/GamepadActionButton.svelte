<script lang="ts">
	// a noph-ui button that is also pressed by its key or gamepad button, which are shown next to the label
	import { Button } from 'noph-ui';
	import type { ButtonProps } from 'noph-ui/types';
	import { untrack } from 'svelte';
	import { ButtonInputComponent, registerComponent, unregisterComponent } from 'svelte-gamepad-virtual-joystick';
	import { getInputSettings } from '#lib/input-config.svelte.ts';
	import InputIcons from './InputIcons.svelte';

	interface Props extends Omit<ButtonProps, 'onclick' | 'end'> {
		/** the binding in the input settings */
		action: 'stop' | 'hints';
		onpress: () => void;
	}

	let { action, onpress, children, ...rest }: Props = $props();

	const input = getInputSettings();

	// registered once: the mapping object is updated in place when the bindings change
	$effect(() => {
		const component = untrack(() => {
			const created = new ButtonInputComponent(input.buttonMappings[action], undefined, false, () => {
				if (!input.suspended && !rest.disabled) {
					onpress();
				}
			});
			registerComponent(['default'], created);
			return created;
		});
		return () => untrack(() => unregisterComponent(['default'], component));
	});
</script>

{#snippet icons()}
	<span class="keycaps"><InputIcons binding={input.config[action]} size={20} /></span>
{/snippet}

<Button {...rest} end={icons} onclick={onpress}>
	{@render children?.()}
</Button>

<style>
	/* readable on every button color */
	.keycaps {
		display: inline-flex;
		margin-inline-start: 4px;
		padding: 1px 3px;
		border-radius: 6px;
		color: #e3e6ea;
		background: rgb(0 0 0 / 0.45);
	}

	/* light icons on the dark keycap in every theme */
	.keycaps :global(.glyph path[fill='#d6dadf']) {
		fill: #e3e6ea;
	}
</style>
