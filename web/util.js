export class ZoomHandler {
    constructor(minZoom, maxZoom, callback) {
        this.zoomLevel = 1;
        this.minZoom = minZoom;
        this.maxZoom = maxZoom;
        this.touchDistance = null;
        this.callback = callback;
    }

    handleMouseWheel(event) {
        const delta = Math.max(-1, Math.min(1, (event.wheelDelta || -event.detail)));
        const zoomChange = delta > 0 ? 0.1 : -0.1;
        this.changeZoom(zoomChange);
        event.preventDefault();
    }

    handleTouch(event) {
        const touchCount = event.touches.length;
        if (touchCount === 2) {
            const touch1 = event.touches[0];
            const touch2 = event.touches[1];
            const distance = Math.hypot(touch1.clientX - touch2.clientX, touch1.clientY - touch2.clientY);
            const previousDistance = this.touchDistance || distance;
            const zoomChange = (distance - previousDistance) / 1000; // Adjust the division factor for sensitivity
            this.changeZoom(zoomChange);
            this.touchDistance = distance;
            event.preventDefault();
        }
    }

    changeZoom(zoomChange) {
        this.zoomLevel = Math.min(this.maxZoom, Math.max(this.minZoom, this.zoomLevel + zoomChange));
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


export function api(url, payload=null, onSuccess=null, onFailure=null) {
    let context = { method: 'POST' }
    if (payload !== null) {
        context.headers = { 'Content-Type': 'application/json' }
        context.body = JSON.stringify(payload)
    }
    fetch(url, context ).then(response => response.json()).then((data) => {
        if (onSuccess !== null) onSuccess(data);
        console.log(data)
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
