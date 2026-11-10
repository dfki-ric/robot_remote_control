/**
 * Keyboard and gamepad bindings of the drive controls, configurable in the controls dialog and remembered
 * by the browser. Converted into the input mappings of svelte-gamepad-virtual-joystick.
 * Joysticks use screen coordinates: x right, y down.
 */
import { createContext } from 'svelte';
import { GamepadButtons, type ButtonInput, type JoystickInput, type SliderInput } from 'svelte-gamepad-virtual-joystick';
import { storageGetJson, storageSetJson } from './storage';

/** keys as KeyboardEvent.key in lower case, buttons as indices of the standard gamepad mapping */
export interface KeyButtonBinding {
	keys: string[];
	buttons: number[];
}

export interface AxisBinding {
	axis: number;
	invert: boolean;
}

export interface InputConfig {
	/** index of the gamepad to use, -1 for any */
	gamepad: number;
	deadzone: number;
	forward: KeyButtonBinding;
	backward: KeyButtonBinding;
	left: KeyButtonBinding;
	right: KeyButtonBinding;
	/** stick axis for forward/backward, pushed forward it reads negative unless inverted */
	driveAxis: AxisBinding;
	/** stick axis for sideways, pushed right it reads positive unless inverted */
	sidewaysAxis: AxisBinding;
	turnLeft: KeyButtonBinding;
	turnRight: KeyButtonBinding;
	/** stick axis for turning, pushed right it reads positive unless inverted */
	turnAxis: AxisBinding;
	linearSlower: KeyButtonBinding;
	linearFaster: KeyButtonBinding;
	angularSlower: KeyButtonBinding;
	angularFaster: KeyButtonBinding;
	stop: KeyButtonBinding;
	hints: KeyButtonBinding;
}

export type BindingName = {
	[K in keyof InputConfig]: InputConfig[K] extends KeyButtonBinding ? K : never;
}[keyof InputConfig];

export type AxisName = {
	[K in keyof InputConfig]: InputConfig[K] extends AxisBinding ? K : never;
}[keyof InputConfig];

export const BINDING_LABELS: Record<BindingName, string> = {
	forward: 'Forward',
	backward: 'Backward',
	left: 'Sideways left',
	right: 'Sideways right',
	turnLeft: 'Turn left',
	turnRight: 'Turn right',
	linearSlower: 'Linear speed down',
	linearFaster: 'Linear speed up',
	angularSlower: 'Angular speed down',
	angularFaster: 'Angular speed up',
	stop: 'Stop',
	hints: 'Show controls'
};

export const AXIS_LABELS: Record<AxisName, { label: string; push: string; expectsNegative: boolean }> = {
	driveAxis: { label: 'Forward / backward', push: 'Push the stick forward', expectsNegative: true },
	sidewaysAxis: { label: 'Sideways', push: 'Push the stick right', expectsNegative: false },
	turnAxis: { label: 'Turn', push: 'Push the stick right', expectsNegative: false }
};

/** input context of svelte-gamepad-virtual-joystick while the settings are open */
export const SETTINGS_CONTEXT = 'settings';

/** buttons of the settings dialog, only used while it is open (any gamepad) */
export const SETTINGS_BUTTONS = {
	previousTab: GamepadButtons.BUMPER_LEFT,
	nextTab: GamepadButtons.BUMPER_RIGHT,
	select: GamepadButtons.DOWN,
	close: GamepadButtons.RIGHT,
	save: GamepadButtons.MENU,
	up: GamepadButtons.DPAD_UP,
	down: GamepadButtons.DPAD_DOWN,
	left: GamepadButtons.DPAD_LEFT,
	right: GamepadButtons.DPAD_RIGHT
};

export function defaultInputConfig(): InputConfig {
	return {
		gamepad: -1,
		deadzone: 0.15,
		forward: { keys: ['w', 'arrowup'], buttons: [GamepadButtons.DPAD_UP] },
		backward: { keys: ['s', 'arrowdown'], buttons: [GamepadButtons.DPAD_DOWN] },
		left: { keys: ['a'], buttons: [GamepadButtons.DPAD_LEFT] },
		right: { keys: ['d'], buttons: [GamepadButtons.DPAD_RIGHT] },
		driveAxis: { axis: 1, invert: false },
		sidewaysAxis: { axis: 0, invert: false },
		turnLeft: { keys: ['q', 'arrowleft'], buttons: [GamepadButtons.LEFT] },
		turnRight: { keys: ['e', 'arrowright'], buttons: [GamepadButtons.RIGHT] },
		turnAxis: { axis: 2, invert: false },
		// bumpers: linear speed, triggers: angular speed; left: slower, right: faster
		linearSlower: { keys: ['f'], buttons: [GamepadButtons.BUMPER_LEFT] },
		linearFaster: { keys: ['g'], buttons: [GamepadButtons.BUMPER_RIGHT] },
		angularSlower: { keys: ['r'], buttons: [GamepadButtons.TRIGGER_LEFT] },
		angularFaster: { keys: ['t'], buttons: [GamepadButtons.TRIGGER_RIGHT] },
		stop: { keys: [' '], buttons: [GamepadButtons.HOME] },
		hints: { keys: ['h'], buttons: [GamepadButtons.VIEW] }
	};
}

function isBinding(value: unknown): value is KeyButtonBinding {
	const binding = value as KeyButtonBinding;
	return (
		Array.isArray(binding?.keys) && binding.keys.every((key) => typeof key === 'string') &&
		Array.isArray(binding?.buttons) && binding.buttons.every((button) => Number.isInteger(button) && button >= 0)
	);
}

function isAxis(value: unknown): value is AxisBinding {
	const axis = value as AxisBinding;
	return Number.isInteger(axis?.axis) && axis.axis >= 0 && typeof axis.invert === 'boolean';
}

/**
 * @returns the stored configuration if it is complete and valid, null otherwise
 */
export function parseInputConfig(value: unknown): InputConfig | null {
	const config = value as InputConfig;
	if (!config || !Number.isInteger(config.gamepad) || config.gamepad < -1) {
		return null;
	}
	if (typeof config.deadzone !== 'number' || config.deadzone < 0 || config.deadzone >= 1) {
		return null;
	}
	const valid =
		(Object.keys(BINDING_LABELS) as BindingName[]).every((name) => isBinding(config[name])) &&
		(Object.keys(AXIS_LABELS) as AxisName[]).every((name) => isAxis(config[name]));
	return valid ? config : null;
}

/**
 * @returns problems that make the configuration ambiguous, empty if it is usable
 */
export function validateInputConfig(config: InputConfig): string[] {
	const issues: string[] = [];
	const keyOwners = new Map<string, BindingName>();
	const buttonOwners = new Map<number, BindingName>();
	for (const name of Object.keys(BINDING_LABELS) as BindingName[]) {
		for (const key of config[name].keys) {
			const owner = keyOwners.get(key);
			if (owner) {
				issues.push(`Key ${keyLabel(key)} is used for ${BINDING_LABELS[owner]} and ${BINDING_LABELS[name]}.`);
			}
			keyOwners.set(key, name);
		}
		for (const button of config[name].buttons) {
			const owner = buttonOwners.get(button);
			if (owner) {
				issues.push(`Gamepad button ${button} is used for ${BINDING_LABELS[owner]} and ${BINDING_LABELS[name]}.`);
			}
			buttonOwners.set(button, name);
		}
	}
	const axes = (Object.keys(AXIS_LABELS) as AxisName[]).map((name) => config[name].axis);
	if (new Set(axes).size !== axes.length) {
		issues.push('Every stick direction needs its own axis.');
	}
	return issues;
}

const KEY_LABELS: Record<string, string> = {
	' ': 'Space', arrowup: '↑', arrowdown: '↓', arrowleft: '←', arrowright: '→', escape: 'Esc'
};

export function keyLabel(key: string): string {
	return KEY_LABELS[key] ?? (key.length === 1 ? key.toUpperCase() : key[0].toUpperCase() + key.slice(1));
}

const STORAGE_KEY = 'input-config';

function driveMapping(config: InputConfig): JoystickInput {
	return {
		name: 'Drive',
		gamepad: config.gamepad,
		axes_x: config.sidewaysAxis.axis,
		axes_y: config.driveAxis.axis,
		invert_x: config.sidewaysAxis.invert,
		invert_y: config.driveAxis.invert,
		deadzoneX: config.deadzone,
		deadzoneY: config.deadzone,
		key_x_pos: [...config.right.keys],
		key_x_neg: [...config.left.keys],
		key_y_pos: [...config.backward.keys],
		key_y_neg: [...config.forward.keys],
		button_x_pos: [...config.right.buttons],
		button_x_neg: [...config.left.buttons],
		button_y_pos: [...config.backward.buttons],
		button_y_neg: [...config.forward.buttons]
	};
}

function turnMapping(config: InputConfig): JoystickInput {
	return {
		name: 'Turn',
		gamepad: config.gamepad,
		axes_x: config.turnAxis.axis,
		axes_y: config.turnAxis.axis,
		invert_x: config.turnAxis.invert,
		invert_y: false,
		deadzoneX: config.deadzone,
		deadzoneY: config.deadzone,
		key_x_pos: [...config.turnRight.keys],
		key_x_neg: [...config.turnLeft.keys],
		key_y_pos: [],
		key_y_neg: [],
		button_x_pos: [...config.turnRight.buttons],
		button_x_neg: [...config.turnLeft.buttons],
		button_y_pos: [],
		button_y_neg: [],
		disable_axes: ['y']
	};
}

export type SpeedName = 'linear' | 'angular';

/** speed sliders (noph-gamepad-components): only the step buttons and keys, no axis, no focus buttons */
function speedMapping(config: InputConfig, name: SpeedName): SliderInput {
	const slower = name === 'linear' ? config.linearSlower : config.angularSlower;
	const faster = name === 'linear' ? config.linearFaster : config.angularFaster;
	return {
		name: name === 'linear' ? 'Linear speed' : 'Angular speed',
		gamepad: config.gamepad,
		axes: -1,
		sensitivity: 0,
		invert: false,
		buttons: [],
		keys: [],
		buttons_neg: [...slower.buttons],
		buttons_pos: [...faster.buttons],
		keys_neg: [...slower.keys],
		keys_pos: [...faster.keys]
	};
}

function buttonMapping(config: InputConfig, name: 'stop' | 'hints'): ButtonInput {
	return { name: BINDING_LABELS[name], gamepad: config.gamepad, keys: [...config[name].keys], buttons: [...config[name].buttons] };
}

export class InputSettings {
	config = $state<InputConfig>(defaultInputConfig());
	settingsOpen = $state(false);
	/** robot inputs are ignored while the settings are open */
	readonly suspended = $derived(this.settingsOpen);

	/*
	 * Input mappings of svelte-gamepad-virtual-joystick. Its components keep the mapping object they were
	 * created with and read it on every event, so these objects stay the same and are updated in place:
	 * registering components again on changes loses registrations in the library.
	 */
	/** drive joystick: x sideways, y forward (negative) / backward */
	readonly driveMapping: JoystickInput;
	/** turn joystick: x only */
	readonly turnMapping: JoystickInput;
	readonly buttonMappings: Record<'stop' | 'hints', ButtonInput>;
	readonly speedMappings: Record<SpeedName, SliderInput>;

	constructor() {
		const config = this.load();
		this.config = config;
		this.driveMapping = driveMapping(config);
		this.turnMapping = turnMapping(config);
		this.buttonMappings = { stop: buttonMapping(config, 'stop'), hints: buttonMapping(config, 'hints') };
		this.speedMappings = { linear: speedMapping(config, 'linear'), angular: speedMapping(config, 'angular') };
	}

	/**
	 * the stored configuration, bindings added in newer versions get their defaults;
	 * the defaults if it is invalid or the merged bindings conflict
	 */
	private load(): InputConfig {
		const stored = storageGetJson<unknown>(STORAGE_KEY);
		if (stored === null) {
			return defaultInputConfig();
		}
		const merged = typeof stored === 'object' ? parseInputConfig({ ...defaultInputConfig(), ...stored }) : null;
		if (!merged || validateInputConfig(merged).length) {
			console.warn('the stored input configuration is invalid or conflicts, using the defaults');
			return defaultInputConfig();
		}
		return merged;
	}

	/** keys used by the joysticks */
	get driveKeys(): string[] {
		const c = this.config;
		return [c.forward, c.backward, c.left, c.right, c.turnLeft, c.turnRight].flatMap((binding) => binding.keys);
	}

	/**
	 * @throws for configurations that do not validate, nothing is changed then
	 */
	apply(config: InputConfig) {
		const issues = validateInputConfig(config);
		if (issues.length) {
			throw new Error(issues.join(' '));
		}
		this.config = config;
		Object.assign(this.driveMapping, driveMapping(config));
		Object.assign(this.turnMapping, turnMapping(config));
		Object.assign(this.buttonMappings.stop, buttonMapping(config, 'stop'));
		Object.assign(this.buttonMappings.hints, buttonMapping(config, 'hints'));
		Object.assign(this.speedMappings.linear, speedMapping(config, 'linear'));
		Object.assign(this.speedMappings.angular, speedMapping(config, 'angular'));
		storageSetJson(STORAGE_KEY, config);
	}
}

export const [getInputSettings, setInputSettings] = createContext<InputSettings>();
