<script lang="ts">
	import { getSession } from '#lib/session/session.svelte.ts';
	import SidebarCard from './SidebarCard.svelte';

	const session = getSession();
</script>

<SidebarCard id="sensors" title="Sensors">
	{#each session.telemetry.sensors as sensor (sensor.key)}
		<div class="sensor">
			<div class="title">{sensor.title}</div>
			<table>
				<tbody>
					{#each sensor.rows as [name, value], i (i)}
						<tr><td>{name}</td><td>{value}</td></tr>
					{/each}
				</tbody>
			</table>
		</div>
	{:else}
		<span class="muted">No sensor data received yet.</span>
	{/each}
</SidebarCard>

<style>
	.title {
		font-weight: 500;
		margin-bottom: 2px;
	}

	table {
		width: 100%;
		font-size: 0.8125rem;
		border-collapse: collapse;
	}

	td:last-child {
		text-align: right;
		font-variant-numeric: tabular-nums;
	}
</style>
