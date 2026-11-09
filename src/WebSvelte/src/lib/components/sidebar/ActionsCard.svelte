<script lang="ts">
	// simple actions: triggers, named values, numeric values; actions are shown while their dependencies hold
	import { Button, Slider } from 'noph-ui';
	import type { SimpleAction } from '#lib/rrc/protocol.ts';
	import { getSession } from '#lib/session/session.svelte.ts';
	import SidebarCard from './SidebarCard.svelte';

	const session = getSession();

	const states = $derived(new Map(session.simpleActions.map((action) => [action.name ?? '', action.state ?? 0])));

	function available(action: SimpleAction): boolean {
		return (action.type?.actionDependency ?? []).every(
			(dependency) => states.get(dependency.dependsOnAction ?? '') === (dependency.dependsOnActionInState ?? 0)
		);
	}

	const visible = $derived(session.simpleActions.filter((action) => action.name && available(action)));
	const triggers = $derived(visible.filter((action) => action.type?.type === 'TRIGGER'));
	const values = $derived(visible.filter((action) => action.type?.type !== 'TRIGGER'));

	/** values being edited, not overwritten by the robot state */
	let editing = $state<Record<string, number>>({});

	function send(name: string, value: number) {
		delete editing[name];
		void session.sendSimpleAction(name, value);
	}
</script>

<SidebarCard id="actions" title="Actions" defaultOpen>
	{#if !visible.length}
		<span class="muted">The robot has no simple actions.</span>
	{/if}
	{#if triggers.length}
		<div class="row">
			{#each triggers as action (action.name)}
				<Button variant="tonal" title={action.type?.documentation} onclick={() => send(action.name ?? '', 1)}>{action.name}</Button>
			{/each}
		</div>
	{/if}
	{#each values as action (action.name)}
		{@const name = action.name ?? ''}
		{@const type = action.type ?? {}}
		{@const min = type.minState ?? 0}
		{@const max = type.maxState ?? 0}
		<div class="action">
			<span class="label" title={type.documentation || name}>{name}</span>
			{#if type.valueNames?.length}
				<div class="row">
					{#each type.valueNames as named (named.name)}
						<Button
							size="xs"
							variant={(named.value ?? 0) === (action.state ?? 0) ? 'filled' : 'outlined'}
							onclick={() => send(name, named.value ?? 0)}>{named.name}</Button
						>
					{/each}
				</div>
			{:else}
				<div class="value">
					{#if max > min}
						<div
							class="slider"
							onchange={(event) => send(name, Number((event.target as HTMLInputElement).value))}
							oninput={(event) => (editing[name] = Number((event.target as HTMLInputElement).value))}
						>
							<Slider value={editing[name] ?? action.state ?? 0} {min} {max} step={type.stepSize || (type.type === 'VALUE_INT' ? 1 : 0)} aria-label={name} />
						</div>
					{/if}
					<input
						type="number"
						aria-label={name}
						{min}
						max={max > min ? max : undefined}
						step={type.stepSize || (type.type === 'VALUE_INT' ? 1 : 'any')}
						value={editing[name] ?? action.state ?? 0}
						oninput={(event) => (editing[name] = Number(event.currentTarget.value))}
						onchange={(event) => send(name, Number(event.currentTarget.value))}
					/>
				</div>
			{/if}
		</div>
	{/each}
</SidebarCard>

<style>
	.action {
		display: flex;
		flex-direction: column;
		gap: 4px;
	}

	.label {
		font-size: 0.875rem;
	}

	.value {
		display: flex;
		align-items: center;
		gap: 12px;
	}

	.slider {
		flex: 1;
	}

	input[type='number'] {
		width: 6em;
		font: inherit;
		padding: 4px 6px;
		color: var(--np-color-on-surface);
		background: var(--np-color-surface-container-high);
		border: 1px solid var(--np-color-outline-variant);
		border-radius: 6px;
	}
</style>
