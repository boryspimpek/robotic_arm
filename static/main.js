// Funkcja która ustawia kąt serwa
function moveServo(id, angle) {
    document.getElementById('angle' + id).innerText = angle;

    fetch('/move_servo', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({ id: id, angle: angle })
    });
}
// Funkcja która ustawia preset
function movePreset(presetName) {
    fetch('/move_preset/' + presetName, {
        method: 'POST'
    })
    .then(response => {
        if (response.ok) {
            // Jeśli preset się udał, odśwież suwaki
            updateSliders();
        } else {
            console.error('Preset error!');
        }
    });
}

// Funkcja która odświeża suwaki
function updateSliders() {
    fetch('/get_angles')
        .then(response => response.json())
        .then(data => {
            for (const id in data) {
                const angle = data[id];
                const slider = document.getElementById('slider' + id);
                const angleDisplay = document.getElementById('angle' + id);
                if (slider) slider.value = angle;
                if (angleDisplay) angleDisplay.innerText = angle;
            }
        });
}

// Controller
function startPad() {
    fetch('/start_pad', { method: 'POST' });
}

function stopPad() {
    fetch('/stop_pad', { method: 'POST' });
}

function savePosition() {
    fetch('/save_position', { method: 'POST' })
        .then(r => r.text())
        .then(alert)
        .catch(() => alert('Błąd przy zapisie.'));
}

function resetPositions() {
    fetch('/reset_positions', { method: 'POST' })
        .then(r => r.text())
        .then(alert)
        .catch(() => alert('Błąd przy usuwaniu.'));
}

function playPositions() {
    fetch('/play_positions', { method: 'POST' })
        .then(r => r.text())
        .then(alert)
        .catch(() => alert('Błąd przy odtwarzaniu.'));
}

function torqueOn() {
    fetch('/torque_on', { method: 'POST' })
        .then(r => r.text())
        .then(alert)
        .catch(() => alert('Błąd przy włączaniu torque'));
}

function torqueOff() {
    fetch('/torque_off', { method: 'POST' })
        .then(r => r.text())
        .then(alert)
        .catch(() => alert('Błąd przy wyłączaniu torque'));
}

// Ustaw suwaki na aktualne wartości serw po załadowaniu strony
window.onload = function () {
    fetch('/get_angles')
        .then(response => response.json())
        .then(data => {
            for (const id in data) {
                const angle = data[id];
                const slider = document.getElementById('slider' + id);
                const angleDisplay = document.getElementById('angle' + id);
                if (slider) slider.value = angle;
                if (angleDisplay) angleDisplay.innerText = angle;

                // Dodaj nasłuchiwanie przesunięcia suwaka
                if (slider) {
                    slider.oninput = function () {
                        moveServo(id, this.value);
                    };
                }
            }
        });
};
