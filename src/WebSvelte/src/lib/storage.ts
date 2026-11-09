/**
 * The browser storage keeps the layout. It may be unavailable (private mode, blocked storage):
 * the page works without it, the layout is just not remembered.
 */

const PREFIX = 'rrc-';

export function storageGet(key: string): string | null {
	try {
		return localStorage.getItem(PREFIX + key);
	} catch {
		return null;
	}
}

export function storageSet(key: string, value: string) {
	try {
		localStorage.setItem(PREFIX + key, value);
	} catch {
		// not remembered
	}
}

export function storageGetJson<T>(key: string): T | null {
	const text = storageGet(key);
	if (text === null) {
		return null;
	}
	try {
		return JSON.parse(text) as T;
	} catch {
		return null;
	}
}

export function storageSetJson(key: string, value: unknown) {
	storageSet(key, JSON.stringify(value));
}
