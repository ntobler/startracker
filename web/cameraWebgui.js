
import { api } from './util.js';
import { ref } from './vue.esm-browser.prod.min.js';

export default {
    setup() {
        return {
            camera_settings: ref({
                exposure_ms: 1,
                analog_gain: 2,
                digital_gain: 3,
                binning: 1,
            }),
            intrinsic_calibrator: ref({
                index: 1,
                pattern_width: 1,
                pattern_height: 1,
                pattern_size: 1,
            }),
            attitude: ref({
                quat: [],
                n_matches: 1,
                overlay: 1,
            }),
            brightness: ref(1),
            calibrating: false,
        }
    },
    methods: {
        connectWebSocket() {
            let url = 'ws://' + window.location.host + "/api/image"
            var ws = new WebSocket(url);
            var img = document.getElementById('image');
            ws.onmessage = function (event) {
                var blob = new Blob([event.data], { type: 'image/png' });
                img.src = URL.createObjectURL(blob);
                img.style.display = "block"
            };
        },
        setSettings() {
            let payload = {
                exposure_ms: this.camera_settings.exposure_ms,
                analog_gain: this.camera_settings.analog_gain,
                digital_gain: this.camera_settings.digital_gain,
                binning: this.camera_settings.binning,
                pattern_width: this.intrinsic_calibrator.pattern_width,
                pattern_height: this.intrinsic_calibrator.pattern_height,
                pattern_size: this.intrinsic_calibrator.pattern_size,
                overlay: this.attitude.overlay,
            }
            console.log(payload)
            api('/api/set_settings', payload, this.updateState);
        },
        updateState(data) {
            this.camera_settings = data.camera_settings
            this.intrinsic_calibrator = data.intrinsic_calibrator
            this.attitude = data.attitude
        },
        toggleOverlay() {
            let el = document.getElementById("overlay")
            this.attitude.overlay = this.attitude.overlay ? false : true
            el.classList = this.attitude.overlay ? ["active"] : []
            this.setSettings()
        },
        toggleBrightness() {
            let brightness = {
                1: 2, 2: 4, 4: 1,
            }[Number(this.brightness)];
            console.log(brightness)
            let img = document.getElementById('image');
            img.style.filter = `brightness(${brightness})`
            this.brightness = brightness
        },
        capture(mode) {
            let payload = { mode: mode };
            api('/api/capture', payload, this.updateState);
        },
        putCalibrationImage() {
            api('/api/put_calibration_image', null, this.updateState);
        },
        resetCalibration() {
            api('/api/reset_calibration', null, this.updateState);
        },
        calibrate() {
            if (this.intrinsic_calibrator.index < 1) return;
            this.calibrating = true;
            api(
                '/api/calibrate', null,
                data => { this.calibrating = false; },
                error => { this.calibrating = true; },
            );
        },
    },
    mounted() {
        this.connectWebSocket();
        api('/api/get_state', null, this.updateState);
    }
}
