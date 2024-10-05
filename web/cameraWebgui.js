let url = 'ws://' + window.location.host + "/image"
var ws = new WebSocket(url);

var img = document.getElementById('image');

ws.onmessage = function (event) {
    var blob = new Blob([event.data], { type: 'image/png' });
    img.src = URL.createObjectURL(blob);
    img.style.display = "block"
};

function updateState(state) {
    console.log(state)

    if (state.intrinsic_calibrator.index > 0) {
        document.getElementById('put_calibration_image').innerHTML = `Put (${state.intrinsic_calibrator.index})`
    } else {
        document.getElementById('put_calibration_image').innerHTML = `Put`
    }
}

function setSettings() {
    let payload = {
        exposure_ms: document.getElementById('exposure').value,
        analog_gain: document.getElementById('analog_gain').value,
        digital_gain: document.getElementById('digital_gain').value,
        binning: document.getElementById('binning').value,
        pattern_width: document.getElementById('pattern_w').value,
        pattern_height: document.getElementById('pattern_h').value,
        pattern_size: document.getElementById('pattern_s').value,
        overlay: document.getElementById('overlay').buttonValue == true,
    }
    fetch('/set_settings', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload)
    }).then(response => response.json()).then((state) => {
        updateState(state)
    }).catch(error => {
        console.error('Error:', error);
    });
}

document.getElementById("exposure").onchange = setSettings
document.getElementById("analog_gain").onchange = setSettings
document.getElementById("digital_gain").onchange = setSettings
document.getElementById("binning").onchange = () => {
    let el = document.getElementById("binning")
    if (["1", "2", "4", "8"].includes(el.value)) {
        el.classList.remove("error")
    } else {
        el.classList.add("error")
    }
    setSettings()
}
document.getElementById("pattern_w").onchange = setSettings
document.getElementById("pattern_h").onchange = setSettings
document.getElementById("pattern_s").onchange = setSettings
document.getElementById("overlay").onclick = () => {
    let el = document.getElementById("overlay")
    let active = el.buttonValue == true ? false : true
    el.buttonValue = active
    el.classList = active ? ["active"] : []
    setSettings()
}
document.getElementById("brightness").onclick = () => {
    let el = document.getElementById("brightness")
    let value = {
        "Brightness 1x": ["Brightness 2x", "brightness(2)"],
        "Brightness 2x": ["Brightness 4x", "brightness(4)"],
        "Brightness 4x": ["Brightness 1x", "brightness(1)"],
    }[el.innerHTML]
    el.innerHTML = value[0]
    img.style.filter = value[1]
}

function capture(mode) {
    let payload = {
        mode: mode
    }
    fetch('/capture', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload)
    }).then(response => response.json()).then((state) => {
        updateState(state)
    }).catch(error => {
        console.error('Error:', error);
    });
}

document.getElementById('capture_single').onclick = () => {
    capture("single")
}

document.getElementById('capture_continuous').onclick = () => {
    capture("continuous")
}

document.getElementById('capture_stop').onclick = () => {
    capture("stop")
}

document.getElementById('capture_darkframe').onclick = () => {
    capture("darkframe")
}

document.getElementById('put_calibration_image').onclick = () => {
    fetch('/put_calibration_image', {
        method: 'POST',
    }).then(response => response.json()).then((state) => {
        updateState(state)
    }).catch(error => {
        console.error('Error:', error);
    });
}

document.getElementById('reset_calibration').onclick = () => {
    fetch('/reset_calibration', {
        method: 'POST',
    }).then(response => response.json()).then((state) => {
        updateState(state)
    }).catch(error => {
        console.error('Error:', error);
    });
}

document.getElementById('calibrate').onclick = () => {

    let interval = setInterval(function () {
        let el = document.getElementById('calibrate')
        el.innerHTML += "."
    }, 1000)

    fetch('/calibrate', {
        method: 'POST',
    }).then(response => response.json()).then((state) => {
        document.getElementById('calibrate').innerHTML = "Calibrate"
        clearInterval(interval)
        updateState(state)
    }).catch(error => {
        document.getElementById('calibrate').innerHTML = "Calibrate"
        clearInterval(interval)
        console.error('Error:', error);
    });
}

window.addEventListener('DOMContentLoaded', () => {
    fetch('/get_state', {
        method: 'POST',
    }).then(response => response.json()).then((state) => {
        document.getElementById("exposure").value = state.camera_settings.exposure_ms
        document.getElementById("analog_gain").value = state.camera_settings.analog_gain
        document.getElementById("digital_gain").value = state.camera_settings.digital_gain
        document.getElementById("binning").value = state.camera_settings.binning
        document.getElementById("pattern_w").value = state.intrinsic_calibrator.pattern_width
        document.getElementById("pattern_h").value = state.intrinsic_calibrator.pattern_height
        document.getElementById("pattern_s").value = state.intrinsic_calibrator.pattern_size
    }).catch(error => {
        console.error('Error:', error);
    });
})
