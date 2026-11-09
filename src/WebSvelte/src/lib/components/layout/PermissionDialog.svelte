<script lang="ts">
	import { Button, Dialog } from 'noph-ui';
	import { getSession } from '#lib/session/session.svelte.ts';

	const session = getSession();
	const pending = $derived(session.telemetry.permissionRequests[0]);
</script>

<Dialog open={pending !== undefined} headline="Permission requested" supportingText={pending?.description || pending?.requestuid}>
	{#snippet actions()}
		<Button variant="text" onclick={() => session.answerPermissionRequest(pending?.requestuid, false)}>Deny</Button>
		<Button variant="filled" onclick={() => session.answerPermissionRequest(pending?.requestuid, true)}>Grant</Button>
	{/snippet}
</Dialog>
