<script lang="ts">
	// a collapsible card of the sidebar, the open state is remembered
	import { untrack, type Snippet } from 'svelte';
	import { getLayout } from '#lib/layout.svelte.ts';

	interface Props {
		/** storage key */
		id: string;
		title: string;
		defaultOpen?: boolean;
		open?: boolean;
		children: Snippet;
	}

	let { id, title, defaultOpen = false, open = $bindable(), children }: Props = $props();

	const layout = getLayout();
	// restored once, the card remembers its state from then on
	open = untrack(() => layout.cardOpen(id, defaultOpen));
</script>

<details class="card" bind:open ontoggle={(event) => layout.setCardOpen(id, event.currentTarget.open)}>
	<summary>{title}</summary>
	<div class="card-content">
		{@render children()}
	</div>
</details>

<style>
	.card {
		border-radius: 12px;
		background: var(--np-color-surface-container);
	}

	summary {
		padding: 12px 16px;
		font-weight: 500;
		cursor: pointer;
		user-select: none;
		border-radius: 12px;
	}

	summary:focus-visible {
		outline: 2px solid var(--np-color-primary);
		outline-offset: -2px;
	}

	.card-content {
		display: flex;
		flex-direction: column;
		gap: 10px;
		padding: 0 16px 16px;
	}

	.card-content :global(.muted) {
		color: var(--np-color-on-surface-variant);
	}

	.card-content :global(.small) {
		font-size: 0.8125rem;
	}

	.card-content :global(.row) {
		display: flex;
		flex-wrap: wrap;
		align-items: center;
		gap: 8px;
	}
</style>
