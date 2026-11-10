<script lang="ts">
	// settings: controls and theme as tabs. Edited as drafts, applied with Save; the theme is previewed while
	// editing. While open, the inputs are in the 'settings' context (see RobotConsole): the gamepad navigates the
	// dialog (SettingsNavigation), L1/R1 switch the tabs.
	import { Button, Dialog, Tab, Tabs } from 'noph-ui';
	import { untrack } from 'svelte';
	import { ButtonInputComponent, registerComponent, unregisterComponent, type ButtonInput } from 'svelte-gamepad-virtual-joystick';
	import {
		defaultInputConfig, getInputSettings, SETTINGS_BUTTONS, SETTINGS_CONTEXT, validateInputConfig, type InputConfig
	} from '#lib/input-config.svelte.ts';
	import { SettingsNavigation } from '#lib/settings-navigation.ts';
	import { defaultThemeConfig, getThemeSettings, type ThemeConfig } from '#lib/theme.svelte.ts';
	import InputIcon from '../input/InputIcon.svelte';
	import ControlsSettings from './ControlsSettings.svelte';
	import ThemeSettings from './ThemeSettings.svelte';

	const TABS = ['controls', 'theme'] as const;
	type SettingsTab = (typeof TABS)[number];

	const input = getInputSettings();
	const theme = getThemeSettings();

	let tab = $state<SettingsTab>('controls');
	let controlsDraft = $state<InputConfig>(defaultInputConfig());
	let themeDraft = $state<ThemeConfig>(defaultThemeConfig());
	let capturing = $state(false);
	let error = $state('');
	let dialog = $state<HTMLDialogElement>();

	/** the gamepad press that ends a capture must not also act in the dialog */
	const CAPTURE_COOLDOWN_MS = 400;
	let navigationBlockedUntil = 0;

	$effect(() => {
		if (!capturing) {
			navigationBlockedUntil = performance.now() + CAPTURE_COOLDOWN_MS;
		}
	});

	const controlIssues = $derived(validateInputConfig(controlsDraft).length > 0);

	// fresh drafts every time the settings open
	$effect(() => {
		if (input.settingsOpen) {
			untrack(() => {
				controlsDraft = $state.snapshot(input.config);
				themeDraft = $state.snapshot(theme.config);
				error = '';
			});
		}
	});

	// theme preview while open, the saved theme again when closed (also after Esc or a click outside)
	$effect(() => {
		if (input.settingsOpen) {
			const draft = $state.snapshot(themeDraft);
			untrack(() => theme.preview(draft));
		} else {
			untrack(() => theme.restore());
		}
	});

	function navigationEnabled(): boolean {
		return input.settingsOpen && !capturing && performance.now() > navigationBlockedUntil;
	}

	function switchTab(step: number) {
		if (navigationEnabled()) {
			tab = TABS[(TABS.indexOf(tab) + step + TABS.length) % TABS.length];
		}
	}

	/** visible, enabled controls of the dialog in document order (the hidden tab is skipped) */
	function focusables(): HTMLElement[] {
		if (!dialog) {
			return [];
		}
		const selector = 'button:not([disabled]), input:not([disabled]):not([type="hidden"]), select:not([disabled]), [tabindex]:not([tabindex="-1"])';
		return [...dialog.querySelectorAll<HTMLElement>(selector)].filter((element) => element.checkVisibility());
	}

	function focused(): HTMLElement | null {
		const active = document.activeElement;
		return active instanceof HTMLElement && dialog?.contains(active) ? active : null;
	}

	function focus(element: HTMLElement | undefined) {
		// focusVisible: the focus ring is shown although the focus was not moved with the keyboard
		element?.focus({ focusVisible: true } as FocusOptions);
		element?.scrollIntoView({ block: 'nearest' });
	}

	function move(step: -1 | 1) {
		const elements = focusables();
		const current = focused();
		const index = current ? elements.indexOf(current) : -1;
		focus(index < 0 ? elements[0] : elements[(index + step + elements.length) % elements.length]);
	}

	function emitChange(element: HTMLElement) {
		element.dispatchEvent(new Event('input', { bubbles: true }));
		element.dispatchEvent(new Event('change', { bubbles: true }));
	}

	/** left/right: steps sliders, cycles selects, moves the focus elsewhere */
	function adjust(step: -1 | 1) {
		const element = focused();
		if (element instanceof HTMLInputElement && element.type === 'range') {
			if (step > 0) {
				element.stepUp();
			} else {
				element.stepDown();
			}
			emitChange(element);
		} else if (element instanceof HTMLSelectElement) {
			const count = element.options.length;
			let index = element.selectedIndex;
			// skip disabled options (e.g. the placeholder of a select)
			for (let i = 0; i < count; ++i) {
				index = (index + step + count) % count;
				if (!element.options[index].disabled) {
					break;
				}
			}
			element.selectedIndex = index;
			emitChange(element);
		} else {
			move(step);
		}
	}

	function select() {
		const element = focused();
		if (!element) {
			move(1);
		} else if (element instanceof HTMLSelectElement) {
			adjust(1);
		} else {
			element.click();
		}
	}

	// navigation and L1/R1; registered once in the settings context, which is only active while open
	$effect(() => {
		const components = untrack(() => {
			const previous: ButtonInput = { name: 'Previous tab', gamepad: -1, buttons: [SETTINGS_BUTTONS.previousTab], keys: [] };
			const next: ButtonInput = { name: 'Next tab', gamepad: -1, buttons: [SETTINGS_BUTTONS.nextTab], keys: [] };
			const created = [
				new ButtonInputComponent(previous, undefined, false, () => switchTab(-1)),
				new ButtonInputComponent(next, undefined, false, () => switchTab(1)),
				new SettingsNavigation({
					enabled: navigationEnabled,
					move,
					adjust,
					select,
					close: () => (input.settingsOpen = false),
					save: () => {
						if (!controlIssues) {
							save();
						}
					}
				})
			];
			created.forEach((component) => registerComponent([SETTINGS_CONTEXT], component));
			return created;
		});
		return () => untrack(() => components.forEach((component) => unregisterComponent([SETTINGS_CONTEXT], component)));
	});

	function resetTab() {
		if (tab === 'controls') {
			controlsDraft = defaultInputConfig();
		} else {
			themeDraft = defaultThemeConfig();
		}
	}

	function save() {
		try {
			input.apply($state.snapshot(controlsDraft));
			theme.apply($state.snapshot(themeDraft));
			input.settingsOpen = false;
		} catch (e) {
			error = (e as Error).message;
		}
	}
</script>

<Dialog bind:open={input.settingsOpen} bind:element={dialog} headline="Settings" style="--np-dialog-container-width: min(54rem, 100vw)">
	<div class="tabs">
		<InputIcon button={SETTINGS_BUTTONS.previousTab} size={28} />
		<Tabs bind:value={tab} class="tab-list">
			<Tab value="controls">Controls</Tab>
			<Tab value="theme">Theme</Tab>
		</Tabs>
		<InputIcon button={SETTINGS_BUTTONS.nextTab} size={28} />
	</div>

	<div class="panel">
		<!-- both stay mounted: the controls keep their capture state, the drafts are owned here -->
		<div hidden={tab !== 'controls'}>
			<ControlsSettings bind:draft={controlsDraft} bind:capturing active={input.settingsOpen && tab === 'controls'} />
		</div>
		<div hidden={tab !== 'theme'}>
			<ThemeSettings bind:draft={themeDraft} />
		</div>
	</div>

	{#if tab === 'theme' && controlIssues}
		<p class="issue" role="alert">The controls have conflicting bindings, fix them before saving.</p>
	{/if}
	{#if error}
		<p class="issue" role="alert">{error}</p>
	{/if}

	<p class="legend" aria-label="Gamepad controls of the settings">
		<span><InputIcon button="dpad_vertical" size={22} /> move</span>
		<span><InputIcon button="dpad_horizontal" size={22} /> change</span>
		<span><InputIcon button={SETTINGS_BUTTONS.select} size={22} /> select</span>
		<span><InputIcon button={SETTINGS_BUTTONS.close} size={22} /> close</span>
		<span><InputIcon button={SETTINGS_BUTTONS.save} size={22} /> save</span>
	</p>

	{#snippet actions()}
		<Button variant="text" onclick={resetTab}>Reset {tab === 'controls' ? 'controls' : 'theme'}</Button>
		<span class="spacer"></span>
		<Button variant="text" onclick={() => (input.settingsOpen = false)}>Cancel</Button>
		<Button variant="filled" disabled={controlIssues} onclick={save}>Save</Button>
	{/snippet}
</Dialog>

<style>
	.tabs {
		display: flex;
		align-items: center;
		gap: 8px;
		margin-bottom: 16px;
	}

	.tabs :global(.tab-list) {
		flex: 1;
	}

	.issue {
		margin: 12px 0 0;
		color: var(--np-color-error);
	}

	.legend {
		display: flex;
		flex-wrap: wrap;
		gap: 4px 16px;
		margin: 16px 0 0;
		font-size: 0.8125rem;
		color: var(--np-color-on-surface-variant);
	}

	.legend span {
		display: inline-flex;
		align-items: center;
		gap: 4px;
	}

	.spacer {
		flex: 1;
	}
</style>
