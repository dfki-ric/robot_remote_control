<script lang="ts">
	import { IconButton } from 'noph-ui';
	import { getConnectedGamepads } from '#lib/gamepads.svelte.ts';
	import { getInputSettings } from '#lib/input-config.svelte.ts';
	import MaterialSymbol from '../MaterialSymbol.svelte';
	import { getSession } from '#lib/session/session.svelte.ts';

	const session = getSession();
	const input = getInputSettings();
	const gamepads = getConnectedGamepads();

	const gamepadTitle = $derived(
		gamepads.list.map((pad) => `Gamepad ${pad.index}: ${pad.id}`).join('\n') +
			(input.config.gamepad >= 0 ? `\nused: gamepad ${input.config.gamepad}` : '')
	);

	const robotName = $derived(
		session.unreachable ? 'robot not reachable' : session.robotName === null ? 'connecting…' : session.robotName || 'unnamed robot'
	);
	const chips = $derived.by(() => {
		const names = session.robotState?.stateName ?? [];
		return (session.robotState?.state ?? []).map((value, i) => ({ name: names[i] ?? '', value }));
	});
	const commands = $derived(session.status.commands);
	const telemetry = $derived(session.status.telemetry);
</script>

<header class="topbar">
	<span class="brand">RRC</span>
	<span class="robot-name">{robotName}</span>
	<span class="chips">
		{#each chips as chip, i (i)}
			<span class="chip" title={chip.name}>
				{#if chip.name}<span class="chip-label">{chip.name}</span>{/if}{chip.value}
			</span>
		{/each}
	</span>
	<span class="spacer"></span>
	<span class="connection" title="http commands and heartbeat round trip time">
		<span class={['dot', commands === 'online' && 'ok', commands === 'offline' && 'error']}></span>
		commands
		{#if session.status.latency !== null}<span class="muted">{session.status.latency} ms</span>{/if}
	</span>
	<span class="connection" title="websocket telemetry">
		<span class={['dot', telemetry === 'connected' && 'ok', telemetry === 'disconnected' && 'error']}></span>
		telemetry
	</span>
	{#if gamepads.list.length}
		<span class="gamepad" role="img" title={gamepadTitle} aria-label="Gamepad connected">
			<MaterialSymbol name="sports_esports" />
			{#if gamepads.list.length > 1}<span class="count">{gamepads.list.length}</span>{/if}
		</span>
	{/if}
	<IconButton variant="text" title="Settings" aria-label="Settings" onclick={() => (input.settingsOpen = true)}>
		<MaterialSymbol name="settings" />
	</IconButton>
</header>

<style>
	.topbar {
		display: flex;
		align-items: center;
		gap: 16px;
		height: 52px;
		padding: 0 8px 0 16px;
		flex-shrink: 0;
		background: var(--np-color-surface-container);
		border-bottom: 1px solid var(--np-color-outline-variant);
	}

	.brand {
		font-weight: 700;
		color: var(--np-color-primary);
	}

	.robot-name {
		font-size: 1.0625rem;
		font-weight: 500;
	}

	.chips {
		display: flex;
		gap: 6px;
		overflow: hidden;
	}

	.chip {
		display: inline-flex;
		gap: 6px;
		padding: 2px 10px;
		border-radius: 8px;
		font-size: 0.8125rem;
		white-space: nowrap;
		background: var(--np-color-secondary-container);
		color: var(--np-color-on-secondary-container);
	}

	.chip-label {
		opacity: 0.7;
	}

	.spacer {
		flex: 1;
	}

	.connection {
		display: inline-flex;
		align-items: center;
		gap: 6px;
		font-size: 0.8125rem;
		white-space: nowrap;
	}

	.gamepad {
		display: inline-flex;
		align-items: center;
		gap: 2px;
		color: var(--rrc-ok);
	}

	.count {
		font-size: 0.75rem;
		font-weight: 600;
	}

	.muted {
		color: var(--np-color-on-surface-variant);
	}

	.dot {
		width: 10px;
		height: 10px;
		border-radius: 50%;
		background: var(--np-color-outline);
	}

	.dot.ok {
		background: var(--rrc-ok);
	}

	.dot.error {
		background: var(--np-color-error);
	}

	@media (max-width: 700px) {
		.chips {
			display: none;
		}
	}
</style>
