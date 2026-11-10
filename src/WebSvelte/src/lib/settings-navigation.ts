/**
 * Gamepad navigation of the settings dialog as an input component of svelte-gamepad-virtual-joystick, active in
 * the settings context: d-pad or left stick up/down move the focus, left/right change sliders and selects,
 * A selects, B closes, Menu saves. Directions repeat while held.
 */
import { InputComponent } from 'svelte-gamepad-virtual-joystick';
import { SETTINGS_BUTTONS } from './input-config.svelte';

export interface NavigationActions {
	/** false while e.g. a binding is captured: nothing happens and held directions must be released first */
	enabled(): boolean;
	move(step: -1 | 1): void;
	adjust(step: -1 | 1): void;
	select(): void;
	close(): void;
	save(): void;
}

type Direction = 'up' | 'down' | 'left' | 'right';

const REPEAT_DELAY_MS = 400;
const REPEAT_INTERVAL_MS = 120;
const AXIS_PRESSED = 0.6;
const AXIS_RELEASED = 0.3;
const STICK_X = 0;
const STICK_Y = 1;

const DIRECTION_BUTTONS: Record<number, Direction> = {
	[SETTINGS_BUTTONS.up]: 'up',
	[SETTINGS_BUTTONS.down]: 'down',
	[SETTINGS_BUTTONS.left]: 'left',
	[SETTINGS_BUTTONS.right]: 'right'
};

const ACTION_BUTTONS = [SETTINGS_BUTTONS.select, SETTINGS_BUTTONS.close, SETTINGS_BUTTONS.save];

export class SettingsNavigation extends InputComponent {
	/** source ("button:<i>" or "axis:<direction>") -> time of the next repeat, Infinity: wait for the release */
	private readonly held = new Map<string, number>();

	constructor(private readonly actions: NavigationActions) {
		super({ name: 'Settings navigation', gamepad: -1 });
	}

	thisGamepadButton(gamepad: Gamepad, btn: number): boolean {
		return super.thisGamepadButton(gamepad, btn) && (btn in DIRECTION_BUTTONS || ACTION_BUTTONS.includes(btn));
	}

	onbuttonpressed(_gamepad: Gamepad, btn: number): boolean {
		if (!this.actions.enabled()) {
			return false;
		}
		if (btn === SETTINGS_BUTTONS.select) {
			this.actions.select();
		} else if (btn === SETTINGS_BUTTONS.close) {
			this.actions.close();
		} else if (btn === SETTINGS_BUTTONS.save) {
			this.actions.save();
		}
		return false;
	}

	onbuttonhold(_gamepad: Gamepad, btn: number) {
		const direction = DIRECTION_BUTTONS[btn];
		if (direction) {
			this.hold(`button:${btn}`, direction);
		}
	}

	onbuttonrelease(_gamepad: Gamepad, btn: number) {
		this.held.delete(`button:${btn}`);
	}

	onupdate(gamepad: Gamepad) {
		const x = gamepad.axes[STICK_X] ?? 0;
		const y = gamepad.axes[STICK_Y] ?? 0;
		this.axis('up', -y);
		this.axis('down', y);
		this.axis('left', -x);
		this.axis('right', x);
	}

	private axis(direction: Direction, value: number) {
		const source = `axis:${direction}`;
		if (value > AXIS_PRESSED || (this.held.has(source) && value > AXIS_RELEASED)) {
			this.hold(source, direction);
		} else {
			this.held.delete(source);
		}
	}

	/**
	 * called while a direction is held: acts at once, then repeats
	 */
	private hold(source: string, direction: Direction) {
		const now = performance.now();
		if (!this.actions.enabled()) {
			this.held.set(source, Infinity);
			return;
		}
		const next = this.held.get(source);
		if (next !== undefined && now < next) {
			return;
		}
		this.held.set(source, next === undefined ? now + REPEAT_DELAY_MS : now + REPEAT_INTERVAL_MS);
		if (direction === 'up' || direction === 'down') {
			this.actions.move(direction === 'up' ? -1 : 1);
		} else {
			this.actions.adjust(direction === 'left' ? -1 : 1);
		}
	}
}
