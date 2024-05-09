let url = 'ws://' + window.location.host + "/image"
var ws = new WebSocket(url);

var img = document.getElementById('image');

ws.onmessage = function (event) {
    var blob = new Blob([event.data], { type: 'image/png' });
    img.src = URL.createObjectURL(blob);
};

function updateState(state) {
    console.log(state)

    if (state.intrinsic_image_count > 0) {
        document.getElementById('put_calibration_image').innerHTML = `Put (${state.intrinsic_image_count})`
    } else {
        document.getElementById('put_calibration_image').innerHTML = `Put`
    }
}

function setSettings() {
    let payload = {
        exposure_ms: document.getElementById('exposure').value,
        gain: document.getElementById('gain').value,
        digital_gain: document.getElementById('digital_gain').value,
        binning: document.getElementById('binning').value,
        pattern_width: document.getElementById('pattern_w').value,
        pattern_height: document.getElementById('pattern_h').value,
        pattern_size: document.getElementById('pattern_s').value,
    }
    fetch('/set_settings', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload)
    }).then(response => response.json()).then((data) => {
        updateState(data)
    }).catch(error => {
        console.error('Error:', error);
    });
}

document.getElementById("exposure").onchange = setSettings
document.getElementById("gain").onchange = setSettings
document.getElementById("digital_gain").onchange = setSettings
document.getElementById("binning").onchange = setSettings
document.getElementById("pattern_w").onchange = setSettings
document.getElementById("pattern_h").onchange = setSettings
document.getElementById("pattern_s").onchange = setSettings

function capture(mode) {
    let payload = {
        mode: mode
    }
    fetch('/capture', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload)
    }).then(response => response.json()).then((data) => {
        updateState(data)
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
    }).then(response => response.json()).then((data) => {
        updateState(data)
    }).catch(error => {
        console.error('Error:', error);
    });
}

document.getElementById('reset_calibration').onclick = () => {
    fetch('/reset_calibration', {
        method: 'POST',
    }).then(response => response.json()).then((data) => {
        updateState(data)
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
    }).then(response => response.json()).then((data) => {
        document.getElementById('calibrate').innerHTML = "Calibrate"
        clearInterval(interval)
        updateState(data)
    }).catch(error => {
        document.getElementById('calibrate').innerHTML = "Calibrate"
        clearInterval(interval)
        console.error('Error:', error);
    });
}
