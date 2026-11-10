<script lang="ts">
	// controllable joints, the slider range comes from the urdf limits
	import { Slider } from 'noph-ui';
	import { degrees, format } from '#lib/rrc/util.ts';
	import { getSession } from '#lib/session/session.svelte.ts';
	import SidebarCard from './SidebarCard.svelte';

	const session = getSession();

	/** joints being dragged: their slider is not moved by the joint state */
	let dragging = $state<Record<string, number>>({});

	const joints = $derived(
		session.controllableJoints.map((name) => {
			const limits = session.model?.kinematics.limits(name) ?? { type: 'revolute', lower: -Math.PI, upper: Math.PI };
			return { name, ...limits, position: session.telemetry.jointPositions.get(name) };
		})
	);

	function positionText(type: string, position: number | undefined): string {
		if (position === undefined) {
			return '–';
		}
		return type === 'prismatic' ? `${format(position, 3)} m` : `${format(position, 2)} rad  ${format(degrees(position), 0)}°`;
	}
</script>

<SidebarCard id="joints" title="Joints">
	{#each joints as joint (joint.name)}
		<div class="joint">
			<div class="name"><span>{joint.name}</span><span class="muted small">{positionText(joint.type, joint.position)}</span></div>
			<div
				role="group"
				aria-label={joint.name}
				oninput={(event) => {
					const value = Number((event.target as HTMLInputElement).value);
					dragging[joint.name] = value;
					session.sendJoint(joint.name, value);
				}}
				onpointerup={() => delete dragging[joint.name]}
				onfocusout={() => delete dragging[joint.name]}
			>
				<Slider
					value={dragging[joint.name] ?? joint.position ?? 0}
					min={joint.lower}
					max={joint.upper}
					step={(joint.upper - joint.lower) / 1000}
					aria-label={joint.name}
				/>
			</div>
		</div>
	{:else}
		<span class="muted">The robot has no controllable joints.</span>
	{/each}
</SidebarCard>

<style>
	.joint .name {
		display: flex;
		justify-content: space-between;
		gap: 8px;
		font-size: 0.875rem;
	}
</style>
