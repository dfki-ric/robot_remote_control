/**
 * Inputs for driving: on screen joysticks, keyboard and gamepads.
 * All return values in [-1, 1].
 */

export class Joystick {
    /**
     * @param element container, the knob is added to it
     * @param axes 'xy' or 'x' (horizontal only)
     */
    constructor(element, axes = 'xy') {
        this.element = element;
        this.axes = axes;
        this.value = {x: 0, y: 0};
        this.knob = document.createElement('div');
        this.knob.className = 'joystick-knob';
        element.appendChild(this.knob);
        element.classList.add('joystick', `joystick-${axes}`);

        element.addEventListener('pointerdown', (event) => {
            element.setPointerCapture(event.pointerId);
            this.active = true;
            this.update(event);
        });
        element.addEventListener('pointermove', (event) => {
            if (this.active) {
                this.update(event);
            }
        });
        const release = () => {
            this.active = false;
            this.value = {x: 0, y: 0};
            this.knob.style.transform = '';
        };
        element.addEventListener('pointerup', release);
        element.addEventListener('pointercancel', release);
    }

    update(event) {
        const rect = this.element.getBoundingClientRect();
        const radius = rect.width / 2;
        let dx = (event.clientX - rect.left - radius) / radius;
        let dy = this.axes === 'x' ? 0 : -(event.clientY - rect.top - rect.height / 2) / radius;
        const length = Math.hypot(dx, dy);
        if (length > 1) {
            dx /= length;
            dy /= length;
        }
        this.value = {x: dx, y: dy};
        this.knob.style.transform = `translate(${dx * radius * 0.7}px, ${-dy * radius * 0.7}px)`;
    }
}

/**
 * W/S or arrow up/down: forward, A/D: sideways, Q/E or arrow left/right: turn
 */
export class KeyboardInput {
    constructor() {
        this.pressed = new Set();
        window.addEventListener('keydown', (event) => {
            if (!this.isTyping(event)) {
                this.pressed.add(event.code);
            }
        });
        window.addEventListener('keyup', (event) => this.pressed.delete(event.code));
        window.addEventListener('blur', () => this.pressed.clear());
    }

    isTyping(event) {
        const target = event.target;
        return target && (target.isContentEditable || ['INPUT', 'SELECT', 'TEXTAREA'].includes(target.tagName));
    }

    axis(positive, negative) {
        const p = positive.some((code) => this.pressed.has(code)) ? 1 : 0;
        const n = negative.some((code) => this.pressed.has(code)) ? 1 : 0;
        return p - n;
    }

    get value() {
        return {
            forward: this.axis(['KeyW', 'ArrowUp'], ['KeyS', 'ArrowDown']),
            left: this.axis(['KeyA'], ['KeyD']),
            turn: this.axis(['KeyQ', 'ArrowLeft'], ['KeyE', 'ArrowRight']),
        };
    }
}

/**
 * first connected gamepad (standard mapping): left stick drives, right stick turns
 */
export function readGamepad(deadzone = 0.15) {
    const pads = navigator.getGamepads ? navigator.getGamepads() : [];
    const pad = Array.from(pads).find((p) => p && p.connected);
    if (!pad) {
        return null;
    }
    const axis = (i) => {
        const v = pad.axes[i] || 0;
        return Math.abs(v) < deadzone ? 0 : v;
    };
    return {forward: -axis(1), left: -axis(0), turn: -axis(2), name: pad.id};
}
