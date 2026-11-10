<script lang="ts">
	import { getSession } from '#lib/session/session.svelte.ts';

	const session = getSession();
</script>

<div class="log" role="log">
	{#each session.telemetry.log as entry (entry.id)}
		<div class="entry">
			<span class="time">{entry.time}</span>
			<span class="level level-{entry.level}">{entry.level}</span>
			<span class="message">{entry.message}</span>
		</div>
	{:else}
		<p class="empty">Log messages of the robot appear here.</p>
	{/each}
</div>

<style>
	.log {
		padding: 4px 12px;
		font-size: 0.8125rem;
		line-height: 1.5;
	}

	.entry {
		display: grid;
		grid-template-columns: auto 4.5em 1fr;
		gap: 10px;
	}

	.time {
		color: var(--np-color-on-surface-variant);
		font-variant-numeric: tabular-nums;
	}

	.message {
		white-space: pre-wrap;
		overflow-wrap: anywhere;
	}

	.level {
		font-weight: 600;
	}

	.level-FATAL,
	.level-ERROR {
		color: var(--np-color-error);
	}

	.level-WARN {
		color: var(--rrc-warn);
	}

	.level-DEBUG {
		color: var(--np-color-on-surface-variant);
	}

	.empty {
		color: var(--np-color-on-surface-variant);
	}
</style>
