<script lang="ts">
	import { Button, Checkbox } from 'noph-ui';
	import { getSession } from '#lib/session/session.svelte.ts';
	import SidebarCard from './SidebarCard.svelte';

	const session = getSession();
	const maps = session.maps;
</script>

<SidebarCard id="maps" title="Maps">
	{#each session.mapChannels as { channel, name } (channel)}
		{@const status = maps.status[channel]}
		<div class="map">
			<strong>{name || `channel ${channel}`}</strong>
			<span class="muted small description">{status?.text ?? 'not loaded'}</span>
			{#if status?.layer}
				<label class="small">
					<Checkbox checked={status.layer.visible} onchange={(event) => maps.setVisible(channel, event.currentTarget.checked)} />
					shown
				</label>
			{/if}
			<Button size="xs" variant="tonal" onclick={() => maps.load(channel)}>Load</Button>
		</div>
	{/each}
</SidebarCard>

<style>
	.map {
		display: flex;
		align-items: center;
		gap: 8px;
		flex-wrap: wrap;
	}

	.description {
		flex: 1;
	}

	label {
		display: inline-flex;
		align-items: center;
	}
</style>
