<script lang="ts">
	import type { KeyButtonBinding } from '#lib/input-config.svelte.ts';
	import InputIcon from './InputIcon.svelte';

	let { binding, size }: { binding: KeyButtonBinding; size?: number } = $props();
</script>

<span class="input-icons">
	{#each binding.keys as key (key)}
		<InputIcon {key} {size} />
	{/each}
	{#each binding.buttons as button (button)}
		<InputIcon {button} {size} />
	{/each}
</span>

<style>
	.input-icons {
		display: inline-flex;
		align-items: center;
		gap: 2px;
	}
</style>
