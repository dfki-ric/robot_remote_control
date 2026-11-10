<script lang="ts">
	// keyboard and gamepad bindings of the settings dialog, edited in the draft of the dialog
	import { Button, NativeSelect, Slider, Switch } from 'noph-ui';
	import {
		AXIS_LABELS, BINDING_LABELS, validateInputConfig, type AxisName, type BindingName, type InputConfig
	} from '#lib/input-config.svelte.ts';
	import { getConnectedGamepads } from '#lib/gamepads.svelte.ts';
	import InputIcon from '../input/InputIcon.svelte';

	interface Props {
		draft: InputConfig;
		/** shown: a capture is cancelled when the tab is left or the settings close */
		active: boolean;
		/** a key, button or axis is being captured */
		capturing?: boolean;
	}

	let { draft = $bindable(), active, capturing = $bindable(false) }: Props = $props();

	type Capture = { kind: 'key'; name: BindingName } | { kind: 'button'; name: BindingName } | { kind: 'axis'; name: AxisName };

	const AXIS_COUNT = 8;
	const AXIS_THRESHOLD = 0.6;

	const bindings = Object.keys(BINDING_LABELS) as BindingName[];
	const axes = Object.keys(AXIS_LABELS) as AxisName[];

	let capture = $state<Capture | null>(null);
	const connected = getConnectedGamepads();
	const gamepads = $derived(connected.list);

	const issues = $derived(validateInputConfig(draft));

	const captureHint = $derived.by(() => {
		const current = capture;
		if (!current) {
			return '';
		}
		if (current.kind === 'key') {
			return `Press a key for ${BINDING_LABELS[current.name]}. Esc cancels.`;
		}
		if (current.kind === 'button') {
			return `Press a gamepad button for ${BINDING_LABELS[current.name]}. Esc cancels.`;
		}
		return `${AXIS_LABELS[current.name].push} for ${AXIS_LABELS[current.name].label}. Esc cancels.`;
	});

	$effect(() => {
		capturing = capture !== null;
	});

	$effect(() => {
		if (!active) {
			capture = null;
		}
	});

	// keys: the next key press, Esc cancels every capture
	$effect(() => {
		const current = capture;
		if (!current) {
			return;
		}
		const keydown = (event: KeyboardEvent) => {
			event.preventDefault();
			event.stopImmediatePropagation();
			const key = event.key.toLowerCase();
			if (key === 'escape') {
				capture = null;
				return;
			}
			if (current.kind === 'key') {
				const binding = draft[current.name];
				if (!binding.keys.includes(key)) {
					binding.keys.push(key);
				}
				capture = null;
			}
		};
		document.addEventListener('keydown', keydown, { capture: true });
		return () => document.removeEventListener('keydown', keydown, { capture: true });
	});

	// gamepad buttons and axes: the first change compared to the state when the capture started
	$effect(() => {
		const current = capture;
		if (!current || current.kind === 'key') {
			return;
		}
		const pads = () =>
			[...navigator.getGamepads()].filter(
				(pad): pad is Gamepad => pad !== null && (draft.gamepad < 0 || pad.index === draft.gamepad)
			);
		const baseline = new Map(pads().map((pad) => [pad.index, { buttons: pad.buttons.map((b) => b.pressed), axes: [...pad.axes] }]));
		let frame = requestAnimationFrame(function poll() {
			for (const pad of pads()) {
				const start = baseline.get(pad.index) ?? { buttons: [], axes: [] };
				if (current.kind === 'button') {
					const button = pad.buttons.findIndex((b, i) => b.pressed && !start.buttons[i]);
					if (button >= 0) {
						draft[current.name].buttons = [button];
						capture = null;
						return;
					}
				} else {
					const axis = pad.axes.findIndex((value, i) => Math.abs(value - (start.axes[i] ?? 0)) > AXIS_THRESHOLD);
					if (axis >= 0) {
						const value = pad.axes[axis];
						draft[current.name] = { axis, invert: AXIS_LABELS[current.name].expectsNegative ? value > 0 : value < 0 };
						capture = null;
						return;
					}
				}
			}
			frame = requestAnimationFrame(poll);
		});
		return () => cancelAnimationFrame(frame);
	});

	function toggleCapture(next: Capture) {
		const same = capture?.kind === next.kind && capture.name === next.name;
		capture = same ? null : next;
	}

	function isCapturing(kind: Capture['kind'], name: string): boolean {
		return capture?.kind === kind && capture.name === name;
	}
</script>

<div class="content">
	<section>
		<h3>Gamepad</h3>
		<div class="row">
			<NativeSelect label="Use" value={String(draft.gamepad)} onchange={(event) => (draft.gamepad = Number((event.currentTarget as HTMLSelectElement).value))}>
				<option value="-1">any gamepad</option>
				{#each gamepads as pad (pad.index)}
					<option value={String(pad.index)}>{pad.index}: {pad.id}</option>
				{/each}
				{#if draft.gamepad >= 0 && !gamepads.some((pad) => pad.index === draft.gamepad)}
					<option value={String(draft.gamepad)}>{draft.gamepad}: not connected</option>
				{/if}
			</NativeSelect>
			<div class="deadzone">
				<span class="small">Dead zone {draft.deadzone.toFixed(2)}</span>
				<Slider bind:value={draft.deadzone} min={0} max={0.5} step={0.01} aria-label="Dead zone" />
			</div>
		</div>
		{#if !gamepads.length}
			<p class="muted small">No gamepad found. Press a button on it to make it visible to the browser.</p>
		{/if}
	</section>

	<section>
		<h3>Sticks</h3>
		<table>
			<thead><tr><th>Direction</th><th>Axis</th><th>Inverted</th><th></th></tr></thead>
			<tbody>
				{#each axes as name (name)}
					<tr>
						<td>{AXIS_LABELS[name].label}</td>
						<td>
							<select
								class="axis"
								aria-label="{AXIS_LABELS[name].label} axis"
								value={draft[name].axis}
								onchange={(event) => (draft[name].axis = Number(event.currentTarget.value))}
							>
								{#each { length: AXIS_COUNT }, axis (axis)}
									<option value={axis}>{axis}</option>
								{/each}
							</select>
						</td>
						<td><Switch bind:selected={draft[name].invert} aria-label="{AXIS_LABELS[name].label} inverted" /></td>
						<td>
							<Button size="xs" variant={isCapturing('axis', name) ? 'filled' : 'tonal'} onclick={() => toggleCapture({ kind: 'axis', name })}>
								{isCapturing('axis', name) ? 'Cancel' : 'Detect'}
							</Button>
						</td>
					</tr>
				{/each}
			</tbody>
		</table>
	</section>

	<section>
		<h3>Keys and buttons</h3>
		<table>
			<thead><tr><th>Action</th><th>Keys</th><th>Gamepad</th></tr></thead>
			<tbody>
				{#each bindings as name (name)}
					{@const binding = draft[name]}
					<tr>
						<td>{BINDING_LABELS[name]}</td>
						<td>
							<div class="inputs">
								{#each binding.keys as key (key)}
									<button
										type="button"
										class="assigned"
										title="Remove"
										onclick={() => (binding.keys = binding.keys.filter((k) => k !== key))}
									>
										<InputIcon {key} /><span aria-hidden="true">×</span>
									</button>
								{/each}
								<Button size="xs" variant={isCapturing('key', name) ? 'filled' : 'text'} onclick={() => toggleCapture({ kind: 'key', name })}>
									{isCapturing('key', name) ? 'Cancel' : 'Add key'}
								</Button>
							</div>
						</td>
						<td>
							<div class="inputs">
								{#each binding.buttons as button (button)}
									<button
										type="button"
										class="assigned"
										title="Remove"
										onclick={() => (binding.buttons = binding.buttons.filter((b) => b !== button))}
									>
										<InputIcon {button} /><span aria-hidden="true">×</span>
									</button>
								{/each}
								<Button size="xs" variant={isCapturing('button', name) ? 'filled' : 'text'} onclick={() => toggleCapture({ kind: 'button', name })}>
									{isCapturing('button', name) ? 'Cancel' : 'Set'}
								</Button>
							</div>
						</td>
					</tr>
				{/each}
			</tbody>
		</table>
	</section>

	{#if captureHint}
		<p class="capture" role="status">{captureHint}</p>
	{/if}
	{#each issues as issue (issue)}
		<p class="issue" role="alert">{issue}</p>
	{/each}
</div>

<style>
	.content {
		display: flex;
		flex-direction: column;
		gap: 16px;
	}

	h3 {
		margin: 0 0 8px;
		font-size: 0.9375rem;
		font-weight: 500;
	}

	.row {
		display: flex;
		flex-wrap: wrap;
		align-items: center;
		gap: 16px;
	}

	.deadzone {
		display: flex;
		flex-direction: column;
		flex: 1;
		min-width: 180px;
	}

	table {
		width: 100%;
		border-collapse: collapse;
		font-size: 0.875rem;
	}

	th {
		text-align: left;
		font-weight: 500;
		color: var(--np-color-on-surface-variant);
		padding: 4px;
	}

	td {
		padding: 4px;
		border-top: 1px solid var(--np-color-outline-variant);
		vertical-align: middle;
	}

	.inputs {
		display: flex;
		flex-wrap: wrap;
		align-items: center;
		gap: 4px;
	}

	.assigned {
		display: inline-flex;
		align-items: center;
		gap: 2px;
		padding: 2px 6px 2px 2px;
		font: inherit;
		color: var(--np-color-on-surface-variant);
		background: var(--np-color-surface-container-highest);
		border: none;
		border-radius: 8px;
		cursor: pointer;
	}

	.assigned:hover {
		color: var(--np-color-error);
	}

	.axis {
		font: inherit;
		padding: 2px 6px;
		color: var(--np-color-on-surface);
		background: var(--np-color-surface-container-high);
		border: 1px solid var(--np-color-outline-variant);
		border-radius: 6px;
	}

	.capture {
		margin: 0;
		padding: 8px 12px;
		border-radius: 8px;
		background: var(--np-color-primary-container);
		color: var(--np-color-on-primary-container);
	}

	.issue {
		margin: 0;
		color: var(--np-color-error);
	}

	.muted {
		color: var(--np-color-on-surface-variant);
	}

	.small {
		font-size: 0.8125rem;
	}

</style>
