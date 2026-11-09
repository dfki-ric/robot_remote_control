<script lang="ts">
	// telemetry inspector: rates and the latest message of every type and channel
	import { format, summarize } from '#lib/rrc/util.ts';
	import { getSession } from '#lib/session/session.svelte.ts';
	import SidebarCard from './SidebarCard.svelte';

	const session = getSession();

	interface Row {
		key: string;
		type: string;
		channel: number;
		rate: string;
		age: string;
	}

	let open = $state(false);
	let rows = $state.raw<Row[]>([]);
	let selected = $state<string | null>(null);
	let detail = $state('');

	function refresh() {
		const now = performance.now();
		rows = [...session.telemetry.stats.entries()]
			.sort(([a], [b]) => a.localeCompare(b))
			.map(([key, stats]) => {
				const age = (now - stats.meter.last) / 1000;
				return {
					key,
					type: stats.type.toLowerCase(),
					channel: stats.channel,
					rate: age < 3 ? format(stats.meter.rate, 1) : '0',
					age: age < 60 ? `${format(age, 1)}s` : '>1m'
				};
			});
		const data = selected ? session.telemetry.stats.get(selected)?.data : undefined;
		detail = data === undefined ? '' : JSON.stringify(summarize(data), null, 2);
	}

	// polled while open, telemetry may arrive at hundreds of messages per second
	$effect(() => {
		if (!open) {
			return;
		}
		refresh();
		const timer = setInterval(refresh, 1000);
		return () => clearInterval(timer);
	});
</script>

<SidebarCard id="telemetry" title="Telemetry" bind:open>
	<table>
		<thead><tr><th>type</th><th>ch</th><th>Hz</th><th>age</th></tr></thead>
		<tbody>
			{#each rows as row (row.key)}
				<tr
					class={{ selected: row.key === selected }}
					onclick={() => {
						selected = selected === row.key ? null : row.key;
						refresh();
					}}
				>
					<td>{row.type}</td><td>{row.channel}</td><td>{row.rate}</td><td>{row.age}</td>
				</tr>
			{/each}
		</tbody>
	</table>
	{#if detail}
		<pre>{detail}</pre>
	{:else}
		<span class="muted small">Select a row to see its latest message.</span>
	{/if}
</SidebarCard>

<style>
	table {
		width: 100%;
		font-size: 0.8125rem;
		border-collapse: collapse;
	}

	th {
		text-align: left;
		font-weight: 500;
		color: var(--np-color-on-surface-variant);
	}

	td,
	th {
		padding: 2px 4px;
	}

	td:nth-child(n + 2) {
		font-variant-numeric: tabular-nums;
	}

	tbody tr {
		cursor: pointer;
	}

	tbody tr:hover,
	tr.selected {
		background: var(--np-color-surface-container-highest);
	}

	pre {
		max-height: 300px;
		margin: 0;
		padding: 8px;
		overflow: auto;
		font-size: 0.75rem;
		border-radius: 8px;
		background: var(--np-color-surface-container-lowest);
	}
</style>
