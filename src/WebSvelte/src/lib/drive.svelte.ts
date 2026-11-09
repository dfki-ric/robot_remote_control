/**
 * Driving with twist commands. The inputs are the positions of the two joysticks of
 * svelte-gamepad-virtual-joystick (on screen, keyboard and gamepad), in screen coordinates:
 * x right, y down, both in [-1, 1].
 * While there is input, the twist is repeated (robots may stop without new commands), when the input stops
 * a zero twist is sent: twist commands may not time out on the robot.
 */
import { createContext } from 'svelte';
import { LatestValueSender, type RobotRemoteControl } from './rrc/client';
import type { TwistMsg } from './rrc/protocol';

const REPEAT_INTERVAL_MS = 100;

export class DriveController {
	/** x: sideways, y: forward/backward */
	move = $state<[number, number]>([0, 0]);
	/** x: turn */
	turn = $state<[number, number]>([0, 0]);
	#linearSpeed = $state(0.5);
	#angularSpeed = $state(0.8);
	/** controllable frame of the twist, empty for the default frame */
	frame = $state('');

	private driving = false;
	private timer: ReturnType<typeof setInterval> | undefined;
	private readonly sender: LatestValueSender<TwistMsg>;

	/**
	 * @param suspended while true the inputs are ignored (e.g. while the bindings are edited)
	 */
	/** m/s, the speed sliders step by adding: rounded to mm/s */
	get linearSpeed(): number {
		return this.#linearSpeed;
	}

	set linearSpeed(value: number) {
		this.#linearSpeed = Math.round(value * 1000) / 1000;
	}

	/** rad/s, rounded to mrad/s */
	get angularSpeed(): number {
		return this.#angularSpeed;
	}

	set angularSpeed(value: number) {
		this.#angularSpeed = Math.round(value * 1000) / 1000;
	}

	constructor(
		rrc: RobotRemoteControl,
		private readonly suspended: () => boolean = () => false
	) {
		this.sender = new LatestValueSender((twist) => rrc.command('TWIST_COMMAND', twist));
	}

	start() {
		clearInterval(this.timer);
		this.timer = setInterval(() => this.update(), REPEAT_INTERVAL_MS);
	}

	stop() {
		clearInterval(this.timer);
		this.halt();
	}

	/**
	 * sends a zero twist
	 */
	halt() {
		this.driving = false;
		this.sender.set(this.twist(0, 0, 0));
	}

	private update() {
		if (this.suspended()) {
			if (this.driving) {
				this.halt();
			}
			return;
		}
		const forward = -this.move[1];
		const left = -this.move[0];
		const turn = -this.turn[0];
		if (Math.abs(forward) + Math.abs(left) + Math.abs(turn) > 0.01) {
			this.driving = true;
			this.sender.set(this.twist(forward, left, turn));
		} else if (this.driving) {
			this.halt();
		}
	}

	private twist(forward: number, left: number, turn: number): TwistMsg {
		return {
			header: { frame: this.frame },
			linear: { x: forward * this.linearSpeed, y: left * this.linearSpeed, z: 0 },
			angular: { x: 0, y: 0, z: turn * this.angularSpeed }
		};
	}
}

export const [getDrive, setDrive] = createContext<DriveController>();
