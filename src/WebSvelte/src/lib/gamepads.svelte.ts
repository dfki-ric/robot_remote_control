/**
 * Connected gamepads. Browsers only list a gamepad after one of its buttons was pressed, the list is
 * refreshed on connect/disconnect events and polled as a fallback.
 */
import { createContext } from 'svelte';

export interface GamepadInfo {
	index: number;
	id: string;
}

const POLL_INTERVAL_MS = 1000;

export class ConnectedGamepads {
	list = $state.raw<GamepadInfo[]>([]);

	private refresh = () => {
		const list = [...navigator.getGamepads()]
			.filter((pad): pad is Gamepad => pad !== null && pad.connected)
			.map((pad) => ({ index: pad.index, id: pad.id }));
		// replaced only on changes, the list is polled
		if (JSON.stringify(list) !== JSON.stringify(this.list)) {
			this.list = list;
		}
	};

	/**
	 * @returns function to stop watching
	 */
	watch(): () => void {
		this.refresh();
		const timer = setInterval(this.refresh, POLL_INTERVAL_MS);
		window.addEventListener('gamepadconnected', this.refresh);
		window.addEventListener('gamepaddisconnected', this.refresh);
		return () => {
			clearInterval(timer);
			window.removeEventListener('gamepadconnected', this.refresh);
			window.removeEventListener('gamepaddisconnected', this.refresh);
		};
	}
}

export const [getConnectedGamepads, setConnectedGamepads] = createContext<ConnectedGamepads>();
