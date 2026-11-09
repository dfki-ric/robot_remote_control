<script lang="ts">
	// complex actions: points picked in the 3D view or a list of twists
	import { Button, NativeSelect } from 'noph-ui';
	import type { TwistMsg } from '#lib/rrc/protocol.ts';
	import { getSession } from '#lib/session/session.svelte.ts';
	import { getViewerState, type GroundPoint } from '#lib/viewer/viewer-state.svelte.ts';
	import SidebarCard from './SidebarCard.svelte';

	const POSE_TASKS = ['POSE', 'POSE_LIST', 'AREA', 'VOLUME'];
	const TWIST_TASKS = ['TWIST', 'TWIST_LIST'];

	const session = getSession();
	const viewer = getViewerState();

	let selected = $state('0');
	let points = $state.raw<GroundPoint[]>([]);
	let twists = $state.raw<TwistMsg[]>([]);
	let twistInput = $state({ vx: 0.2, vy: 0, wz: 0 });

	const task = $derived(session.complexActions[Number(selected)]);
	const usesPoses = $derived(POSE_TASKS.includes(task?.type ?? ''));
	const usesTwists = $derived(TWIST_TASKS.includes(task?.type ?? ''));
	const picking = $derived(viewer.pickMode === 'waypoints');

	$effect(() => {
		viewer.waypoints = usesPoses ? points : [];
	});

	function addPoint(x: number, y: number) {
		// a single pose task has one point
		points = task?.type === 'POSE' ? [{ x, y }] : [...points, { x, y }];
	}

	function togglePicking() {
		viewer.setPickMode(picking ? null : 'waypoints', picking ? null : addPoint);
	}

	function clear() {
		points = [];
		twists = [];
	}

	function addTwist() {
		twists = [
			...twists,
			{ linear: { x: twistInput.vx, y: twistInput.vy, z: 0 }, angular: { x: 0, y: 0, z: twistInput.wz } }
		];
	}

	async function sendTask() {
		if (!task) {
			return;
		}
		const command: Parameters<typeof session.sendComplexAction>[0] = { name: task.name, type: task.type };
		if (usesPoses) {
			if (!points.length) {
				session.toast('Pick points in the 3D view first.');
				return;
			}
			command.poses = points.map((p) => session.poseMessage(p.x, p.y, null));
		} else if (usesTwists) {
			if (!twists.length) {
				session.toast('Add twists first.');
				return;
			}
			command.twists = twists;
		}
		if (await session.sendComplexAction(command)) {
			if (picking) {
				viewer.setPickMode(null);
			}
			clear();
		}
	}
</script>

<SidebarCard id="tasks" title="Tasks">
	{#if session.complexActions.length}
		<NativeSelect label="Action" bind:value={selected}>
			{#each session.complexActions as action, i (i)}
				<option value={String(i)}>{action.name} ({(action.type ?? 'undefined').toLowerCase()})</option>
			{/each}
		</NativeSelect>

		{#if usesPoses}
			<div class="row">
				<Button size="xs" variant={picking ? 'filled' : 'tonal'} onclick={togglePicking}>Pick points</Button>
				<Button size="xs" variant="text" onclick={clear}>Clear</Button>
				<span class="muted small">{points.length} point(s)</span>
			</div>
		{/if}

		{#if usesTwists}
			<div class="row twist">
				<label>vx <input type="number" step="0.1" bind:value={twistInput.vx} title="linear x (m/s)" /></label>
				<label>vy <input type="number" step="0.1" bind:value={twistInput.vy} title="linear y (m/s)" /></label>
				<label>wz <input type="number" step="0.1" bind:value={twistInput.wz} title="angular z (rad/s)" /></label>
				<Button size="xs" variant="tonal" onclick={addTwist}>Add</Button>
			</div>
			<span class="muted small">
				{twists.map((t) => `(${t.linear?.x}, ${t.linear?.y}, ${t.angular?.z})`).join(' ')}
			</span>
		{/if}

		<Button variant="filled" onclick={sendTask}>Send task</Button>
	{:else}
		<span class="muted">The robot has no complex actions.</span>
	{/if}
</SidebarCard>

<style>
	.twist label {
		display: inline-flex;
		align-items: center;
		gap: 4px;
		font-size: 0.8125rem;
	}

	input[type='number'] {
		width: 4.5em;
		font: inherit;
		padding: 4px 6px;
		color: var(--np-color-on-surface);
		background: var(--np-color-surface-container-high);
		border: 1px solid var(--np-color-outline-variant);
		border-radius: 6px;
	}
</style>
