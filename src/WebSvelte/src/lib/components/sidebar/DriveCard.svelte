<script lang="ts">
	// driving: on screen joysticks that also follow keyboard and gamepad input (svelte-gamepad-virtual-joystick)
	import { NativeSelect } from 'noph-ui';
	import { Slider as GamepadSlider } from 'noph-gamepad-components';
	import { component_state, Joystick } from 'svelte-gamepad-virtual-joystick';
	import { getDrive } from '#lib/drive.svelte.ts';
	import { getInputSettings } from '#lib/input-config.svelte.ts';
	import { format } from '#lib/rrc/util.ts';
	import { getSession } from '#lib/session/session.svelte.ts';
	import GamepadActionButton from '../input/GamepadActionButton.svelte';
	import MaterialSymbol from '../MaterialSymbol.svelte';
	import InputIcons from '../input/InputIcons.svelte';
	import SidebarCard from './SidebarCard.svelte';

	const session = getSession();
	const drive = getDrive();
	const input = getInputSettings();

	const JOYSTICK_SIZE = 132;
	const SPEED_MIN = 0.05;
	const SPEED_MAX = 2;
	const SPEED_STEP = 0.05;
	const joystickStyle = 'background-color: var(--np-color-surface-container-highest); border-radius: 50%;';

	const velocity = $derived.by(() => {
		const twist = session.telemetry.twist;
		if (!twist) {
			return '';
		}
		return `velocity x ${format(twist.linear?.x ?? 0)}  y ${format(twist.linear?.y ?? 0)} m/s  turn ${format(twist.angular?.z ?? 0)} rad/s`;
	});
</script>

<SidebarCard id="drive" title="Drive" defaultOpen>
	<!-- while the settings are open the controls get no input: they stay mounted (and registered in the input
	     library) but invisible, a notice takes their place -->
	<div class="drive-area">
		<div class="drive-controls" inert={input.suspended} aria-hidden={input.suspended}>
		<div class="joysticks">
			<div class="joystick">
				<!-- the mapping objects are updated in place when the bindings change -->
				<Joystick
					bind:position={drive.move}
					inputMapping={input.driveMapping}
					size={JOYSTICK_SIZE / 1.8}
					backgroundWidth={JOYSTICK_SIZE}
					backgroundHeight={JOYSTICK_SIZE}
					color="var(--np-color-primary)"
					borderColor="var(--np-color-outline)"
					style={joystickStyle}
				/>
				<span class="muted small">forward / sideways</span>
			</div>
			<div class="joystick">
				<Joystick
					bind:position={drive.turn}
					inputMapping={input.turnMapping}
					size={JOYSTICK_SIZE / 1.8}
					backgroundWidth={JOYSTICK_SIZE}
					backgroundHeight={JOYSTICK_SIZE}
					color="var(--np-color-tertiary)"
					borderColor="var(--np-color-outline)"
					style={joystickStyle}
				/>
				<span class="muted small">turn</span>
			</div>
		</div>

		<!-- noph-gamepad-components sliders, stepped by their buttons and keys without focus; the mapping objects are
		     updated in place when the bindings change, the hints are shown next to the label instead -->
		<div class="speed">
			<span class="small">Linear speed <span class="muted">{drive.linearSpeed.toFixed(2)} m/s</span></span>
			{#if component_state.showHints}
				<span class="steps small" aria-label="slower and faster">
					<InputIcons binding={input.config.linearSlower} size={20} /> −&nbsp;/&nbsp;+ <InputIcons binding={input.config.linearFaster} size={20} />
				</span>
			{/if}
		</div>
		<GamepadSlider
			bind:value={drive.linearSpeed}
			min={SPEED_MIN}
			max={SPEED_MAX}
			step={SPEED_STEP}
			inputMapping={input.speedMappings.linear}
			requiresFocus={false}
			addHints={false}
			tabindex={0}
			aria-label="Linear speed"
		/>
		<div class="speed">
			<span class="small">Angular speed <span class="muted">{drive.angularSpeed.toFixed(2)} rad/s</span></span>
			{#if component_state.showHints}
				<span class="steps small" aria-label="slower and faster">
					<InputIcons binding={input.config.angularSlower} size={20} /> −&nbsp;/&nbsp;+ <InputIcons binding={input.config.angularFaster} size={20} />
				</span>
			{/if}
		</div>
		<GamepadSlider
			bind:value={drive.angularSpeed}
			min={SPEED_MIN}
			max={SPEED_MAX}
			step={SPEED_STEP}
			inputMapping={input.speedMappings.angular}
			requiresFocus={false}
			addHints={false}
			tabindex={0}
			aria-label="Angular speed"
		/>
		</div>
		{#if input.suspended}
			<div class="paused" role="status">
				<MaterialSymbol name="sports_esports" size={40} />
				<strong>Driving is paused</strong>
				<span>Your gamepad and keyboard are working the settings right now. Close the settings to take the wheel again.</span>
			</div>
		{/if}
	</div>

	<span class="small">Twist frame</span>
	<NativeSelect bind:value={drive.frame} aria-label="Twist frame">
		<option value="">default</option>
		{#each session.twistFrames as frame (frame)}
			<option value={frame}>{frame}</option>
		{/each}
	</NativeSelect>

	<div class="row">
		<GamepadActionButton variant="filled" class="stop" action="stop" onpress={() => drive.halt()}>Stop</GamepadActionButton>
		<GamepadActionButton
			variant="tonal"
			toggle
			selected={component_state.showHints}
			action="hints"
			onpress={() => (component_state.showHints = !component_state.showHints)}
		>
			{component_state.showHints ? 'Hide controls' : 'Show controls'}
		</GamepadActionButton>
	</div>
	{#if velocity}
		<span class="readout small">{velocity}</span>
	{/if}
</SidebarCard>

<style>
	/* controls and notice share one grid cell: the card keeps its height */
	.drive-area {
		display: grid;
	}

	.drive-controls,
	.paused {
		grid-area: 1 / 1;
	}

	.drive-controls {
		display: flex;
		flex-direction: column;
		gap: 10px;
	}

	.drive-controls[inert] {
		visibility: hidden;
	}

	.paused {
		display: flex;
		flex-direction: column;
		align-items: center;
		justify-content: center;
		gap: 8px;
		padding: 16px;
		text-align: center;
		border: 2px dashed var(--np-color-outline-variant);
		border-radius: 16px;
		color: var(--np-color-on-surface-variant);
	}

	.paused strong {
		font-size: 1.0625rem;
		color: var(--np-color-on-surface);
	}

	.joysticks {
		display: flex;
		justify-content: space-around;
		gap: 12px;
	}

	.joystick {
		display: flex;
		flex-direction: column;
		align-items: center;
		gap: 6px;
	}

	/* the hints of the joystick are positioned absolutely in its area */
	.joystick :global(.joystick_area) {
		position: relative;
	}

	.joystick :global(.hints_container) {
		top: 0;
		left: 0;
		z-index: 1;
	}

	/* the icons are 40 px, too large for the joystick area */
	.joystick :global(.hints_container .hint) {
		zoom: 0.55;
	}

	.joystick :global(.hints_container path[fill='#666666']) {
		fill: var(--np-color-on-surface);
	}

	/* noph-gamepad-components adds a global .hint { top: 20px } that would stretch the bottom hint */
	.joystick :global(.hints_container .hint-down) {
		top: auto;
	}

	.speed {
		display: flex;
		align-items: center;
		justify-content: space-between;
		gap: 8px;
		margin-bottom: -6px;
	}

	.steps {
		display: inline-flex;
		align-items: center;
		gap: 4px;
		color: var(--np-color-on-surface-variant);
	}

	:global(.stop) {
		--np-filled-button-container-color: var(--np-color-error);
		--np-filled-button-label-text-color: var(--np-color-on-error);
	}

	.readout {
		font-variant-numeric: tabular-nums;
	}
</style>
