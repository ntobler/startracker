export class ZoomHandler {
    constructor(minZoom, maxZoom, callback) {
        this.zoomLevel = 1;
        this.minZoom = minZoom;
        this.maxZoom = maxZoom;
        this.touchDistance = null;
        this.zoomLevelAnchor = null;
        this.callback = callback;
    }

    handleMouseWheel(event) {
        const delta = Math.max(-1, Math.min(1, (event.wheelDelta || -event.detail)));
        const factor = 1.04;
        const zoomChange = delta > 0 ? factor : 1 / factor;
        this.setZoomLevel(this.zoomLevel * zoomChange);
        event.preventDefault();
    }

    handleTouch(event) {

        if (event.touches.length === 2) {
            const touch1 = event.touches[0];
            const touch2 = event.touches[1];
            const distance = Math.hypot(touch1.clientX - touch2.clientX, touch1.clientY - touch2.clientY);
            if (this.touchDistance === null) {
                this.touchDistance = distance;
                this.zoomLevelAnchor = this.zoomLevel;
            } else {
                const zoomChange = distance / this.touchDistance;
                this.setZoomLevel(this.zoomLevelAnchor * zoomChange);
            }
            event.preventDefault();
        } else {
            this.touchDistance = null;
        }
    }

    setZoomLevel(level) {
        this.zoomLevel = Math.min(Math.max(level, 0.25), 20)
        this.callback(this.zoomLevel);
    }

    attachEventListeners(element) {
        element.addEventListener('wheel', this.handleMouseWheel.bind(this), { passive: false });
        element.addEventListener('touchstart', this.handleTouch.bind(this), { passive: false });
        element.addEventListener('touchmove', this.handleTouch.bind(this), { passive: false });
        element.addEventListener('touchend', this.handleTouchEnd.bind(this), { passive: false });
    }

    handleTouchEnd(event) {
        if (event.touches.length !== 2) {
            this.touchDistance = null;
        }
    }
}


export function api(url, payload = null, onSuccess = null, onFailure = null) {
    let context = { method: 'POST' }
    if (payload !== null) {
        context.headers = { 'Content-Type': 'application/json' }
        context.body = JSON.stringify(payload)
    }
    fetch(url, context).then(response => response.json()).then((data) => {
        if (onSuccess !== null) onSuccess(data);
    }).catch(error => {
        if (onFailure !== null) onFailure(error);
        console.error('Error:', error);
    });
}

export class HelpDisplay {
    constructor(container) {
        this.container = container;
        this.helpDivs = Array.from(container.querySelectorAll('.help'));
        this.helpActive = false;
        this.currentIndex = 0;

        this.helpDivs.forEach((div, index) => {
            div.onclick = () => this.handleDivClick(index);
        });
    }

    handleDivClick(index) {
        const currentDiv = this.helpDivs[index];
        currentDiv.style.display = 'none';
        currentDiv.parentNode.classList.remove("sine-blinking");

        if (index < this.helpDivs.length - 1) {
            const nextDiv = this.helpDivs[index + 1];
            nextDiv.style.display = 'block';
            nextDiv.parentNode.classList.add("sine-blinking");
            this.currentIndex = index + 1;
        } else {
            this.currentIndex = 0;
            this.helpActive = false;
        }
    }

    toggleHelp() {
        if (this.helpDivs.length === 0) return;

        const currentDiv = this.helpDivs[this.currentIndex];
        if (!this.helpActive) {
            this.helpActive = true;
            currentDiv.style.display = 'block';
            currentDiv.parentNode.classList.add("sine-blinking");
        } else {
            this.helpActive = false;
            currentDiv.style.display = 'none';
            currentDiv.parentNode.classList.remove("sine-blinking");
            this.currentIndex = 0;
        }
    }
}

export function matToLaTeX(m, digits = 2) {
    let str = String.raw`\begin{bmatrix}`;
    for (let line of m) {
        let line_latex = [];
        for (let num of line) {
            line_latex.push(parseFloat(Number(num).toFixed(digits)));
        }
        str += line_latex.join("&") + "\\\\";
    }
    str += String.raw`\end{bmatrix}`;
    return str;
}

export function vecToLaTeX(v, digits = 2) {
    let str = String.raw`\begin{bmatrix}`;
    let line_latex = [];
    for (let num of v) {
        line_latex.push(parseFloat(Number(num).toFixed(digits)));
    }
    str += line_latex.join("&");
    str += String.raw`\end{bmatrix}`;
    return str;
}

export function parseSize(size) {
    let unit
    for (unit of ["B", "kB", "MB", "GB"]) {
        if (size < 10) return size.toFixed(2) + unit
        if (size < 100) return size.toFixed(1) + unit
        size /= 1024
    }
}

export function toF32Array(u8_array) {
    if (u8_array.byteLength % 4 != 0) {
        throw Error("Array must be divisible by 4")
    }
    if (u8_array.byteOffset % 4 == 0) {
        // bytes are aligned, we can take a view
        return new Float32Array(u8_array.buffer, u8_array.byteOffset, u8_array.byteLength / 4)
    } else {
        // bytes ar not aligned, we need to copy first
        const alignedBuffer = new Uint8Array(u8_array.byteLength)
        alignedBuffer.set(u8_array, 0)
        return new Float32Array(alignedBuffer.buffer, 0, alignedBuffer.byteLength / 4)
    }
}
