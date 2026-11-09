<script lang="ts">
	// the operator console of one robot: owns the session and the shared ui state
	import { untrack } from 'svelte';
	import { InputManager } from 'svelte-gamepad-virtual-joystick';
	import type { ConnectionConfig } from '#lib/config.ts';
	import { DriveController, setDrive } from '#lib/drive.svelte.ts';
	import { ConnectedGamepads, setConnectedGamepads } from '#lib/gamepads.svelte.ts';
	import { InputSettings, SETTINGS_CONTEXT, setInputSettings } from '#lib/input-config.svelte.ts';
	import { LayoutState, setLayout } from '#lib/layout.svelte.ts';
	import { RobotSession, setSession } from '#lib/session/session.svelte.ts';
	import { setViewerState, ViewerState } from '#lib/viewer/viewer-state.svelte.ts';
	import { setThemeSettings, ThemeSettings } from '#lib/theme.svelte.ts';
	import SettingsDialog from './settings/SettingsDialog.svelte';
	import PermissionDialog from './layout/PermissionDialog.svelte';
	import Toasts from './layout/Toasts.svelte';
	import TopBar from './layout/TopBar.svelte';
	import ViewerArea from './layout/ViewerArea.svelte';
	import ActionsCard from './sidebar/ActionsCard.svelte';
	import DriveCard from './sidebar/DriveCard.svelte';
	import JointsCard from './sidebar/JointsCard.svelte';
	import MapsCard from './sidebar/MapsCard.svelte';
	import RobotCard from './sidebar/RobotCard.svelte';
	import SensorsCard from './sidebar/SensorsCard.svelte';
	import TasksCard from './sidebar/TasksCard.svelte';
	import TelemetryCard from './sidebar/TelemetryCard.svelte';

	let { config }: { config: ConnectionConfig } = $props();

	// the connection is fixed for the lifetime of the page
	const { commandUrl, telemetryUrl } = untrack(() => config);
	const session = setSession(new RobotSession(commandUrl, telemetryUrl));
	const input = setInputSettings(new InputSettings());
	setThemeSettings(new ThemeSettings());
	const gamepads = setConnectedGamepads(new ConnectedGamepads());
	const drive = setDrive(new DriveController(session.rrc, () => input.suspended));
	const viewer = setViewerState(new ViewerState());
	setLayout(new LayoutState());


	function isTyping(event: KeyboardEvent): boolean {
		const target = event.target as HTMLElement | null;
		return Boolean(target && (target.isContentEditable || ['INPUT', 'SELECT', 'TEXTAREA'].includes(target.tagName)));
	}

	$effect(() => gamepads.watch());

	// connection and drive loop live as long as the console; untracked: starting reads and writes session state
	$effect(() => {
		untrack(() => {
			session.start();
			drive.start();
		});
		return () => {
			drive.stop();
			session.stop();
		};
	});

	function releaseDriveKeys() {
		input.driveKeys.forEach((key) => window.dispatchEvent(new KeyboardEvent('keyup', { key })));
	}

	// InputManager listens on window: keys typed into fields or pressed while the bindings are edited must not
	// drive the robot. Key releases always pass, so no key stays pressed.
	$effect(() => {
		const guard = (event: KeyboardEvent) => {
			if (isTyping(event) || input.suspended) {
				event.stopPropagation();
			}
		};
		const releaseKeys = () => {
			releaseDriveKeys();
			drive.halt();
		};
		const escape = (event: KeyboardEvent) => {
			if (event.key === 'Escape' && !isTyping(event)) {
				viewer.setPickMode(null);
			}
		};
		document.addEventListener('keydown', guard);
		window.addEventListener('blur', releaseKeys);
		window.addEventListener('keydown', escape);
		return () => {
			document.removeEventListener('keydown', guard);
			window.removeEventListener('blur', releaseKeys);
			window.removeEventListener('keydown', escape);
		};
	});

	/*
	 * While the settings are open, gamepad and keyboard input goes to the 'settings' context of
	 * svelte-gamepad-virtual-joystick (only the tab switching), so the joysticks, sliders and buttons of the
	 * robot ignore it. Keys held when the settings open are released, their keyup would not reach them.
	 */
	let inputManager = $state<{ changeContext(context: string): void }>();
	let inputContext = 'default';

	$effect(() => {
		const next = input.suspended ? SETTINGS_CONTEXT : 'default';
		const manager = inputManager;
		if (!manager || next === inputContext) {
			return;
		}
		inputContext = next;
		untrack(() => {
			if (next === SETTINGS_CONTEXT) {
				releaseDriveKeys();
				drive.move = [0, 0];
				drive.turn = [0, 0];
			}
			manager.changeContext(next);
		});
	});
</script>

<InputManager bind:this={inputManager} />

<div class="console">
	<TopBar />
	<main>
		<ViewerArea />
		<aside class="sidebar">
			<DriveCard />
			<ActionsCard />
			<JointsCard />
			<SensorsCard />
			<TasksCard />
			<MapsCard />
			<TelemetryCard />
			<RobotCard />
		</aside>
	</main>
</div>

<SettingsDialog />
<PermissionDialog />
<Toasts />

<style>
	.console {
		display: flex;
		flex-direction: column;
		height: 100dvh;
	}

	main {
		display: flex;
		flex: 1;
		min-height: 0;
	}

	.sidebar {
		display: flex;
		flex-direction: column;
		gap: 8px;
		width: var(--rrc-sidebar-width);
		flex-shrink: 0;
		padding: 8px;
		overflow-y: auto;
		border-left: 1px solid var(--np-color-outline-variant);
		background: var(--np-color-surface);
	}

	@media (max-width: 900px) {
		main {
			flex-direction: column;
			overflow-y: auto;
		}

		main > :global(.viewer-area) {
			flex: none;
			height: 70dvh;
		}

		.sidebar {
			width: auto;
			overflow: visible;
			border-left: none;
		}
	}
</style>
