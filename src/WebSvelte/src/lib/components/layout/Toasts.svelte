<script lang="ts">
	// messages to the user, one at a time
	import { Snackbar } from 'noph-ui';
	import { getSession } from '#lib/session/session.svelte.ts';

	const session = getSession();
	const current = $derived(session.toasts[0]);
</script>

{#if current}
	{#key current.id}
		<Snackbar
			open
			label={current.message}
			timeout={6000}
			ontoggle={(event: ToggleEvent) => event.newState === 'closed' && session.dismissToast(current.id)}
		/>
	{/key}
{/if}
