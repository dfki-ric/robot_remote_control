import type { Header, QuaternionMsg, Vector3Msg } from './protocol';

/**
 * @returns the parsed json, null for empty or invalid text (e.g. telemetry that was never set by the robot)
 */
export function parseJson<T = unknown>(text: string | null | undefined): T | null {
	if (!text) {
		return null;
	}
	try {
		return JSON.parse(text) as T;
	} catch {
		return null;
	}
}

export function base64ToBytes(base64: string | undefined): Uint8Array<ArrayBuffer> {
	const binary = atob(base64 ?? '');
	const bytes = new Uint8Array(binary.length);
	for (let i = 0; i < binary.length; ++i) {
		bytes[i] = binary.charCodeAt(i);
	}
	return bytes;
}

export function format(value: number | undefined, digits = 2): string {
	return value !== undefined && Number.isFinite(value) ? value.toFixed(digits) : '–';
}

export function xyz(v: Vector3Msg | undefined, digits = 2): string {
	return `${format(v?.x ?? 0, digits)} ${format(v?.y ?? 0, digits)} ${format(v?.z ?? 0, digits)}`;
}

export const degrees = (radians: number) => (radians * 180) / Math.PI;

export function yawOf(q: QuaternionMsg | undefined): number {
	const x = q?.x ?? 0, y = q?.y ?? 0, z = q?.z ?? 0, w = q?.w ?? 0;
	if (x * x + y * y + z * z + w * w < 1e-9) {
		return 0;
	}
	return Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
}

export function timeOf(header: Header | undefined): string {
	const stamp = header?.timestamp;
	const date = stamp?.secs ? new Date(stamp.secs * 1000 + (stamp.nsecs ?? 0) / 1e6) : new Date();
	return date.toLocaleTimeString();
}

/**
 * calls fn at most every interval ms with the latest arguments
 */
export function throttle<A extends unknown[]>(fn: (...args: A) => void, interval: number): (...args: A) => void {
	let last = 0;
	let timer: ReturnType<typeof setTimeout> | null = null;
	let latest: A;
	return (...args: A) => {
		latest = args;
		const wait = last + interval - performance.now();
		if (wait <= 0) {
			last = performance.now();
			fn(...latest);
		} else if (!timer) {
			timer = setTimeout(() => {
				timer = null;
				last = performance.now();
				fn(...latest);
			}, wait);
		}
	};
}

/**
 * exponentially smoothed message rate
 */
export class RateMeter {
	rate = 0;
	last = 0;

	tick(now = performance.now()) {
		if (this.last) {
			const rate = 1000 / Math.max(now - this.last, 1);
			this.rate = this.rate ? this.rate * 0.8 + rate * 0.2 : rate;
		}
		this.last = now;
	}
}

/**
 * shortens large arrays and strings (images, point clouds) for display
 */
export function summarize(value: unknown): unknown {
	if (Array.isArray(value)) {
		const items = value.slice(0, 10).map(summarize);
		return value.length > 10 ? [...items, `… ${value.length - 10} more`] : items;
	}
	if (typeof value === 'string' && value.length > 120) {
		return `${value.slice(0, 60)}… (${value.length} characters)`;
	}
	if (value && typeof value === 'object') {
		return Object.fromEntries(Object.entries(value).map(([key, item]) => [key, summarize(item)]));
	}
	return value;
}
