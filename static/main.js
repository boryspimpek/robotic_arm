
let gripperState = 'closed';

const toggleGripper = () => {
    if (gripperState === 'closed') {
        // Jeśli gripper jest zamknięty, otwieramy go
        postRequest('/open_gripper')
            .then(response => response.text())
            .then(msg => {
                alert(msg);
                gripperState = 'opened'; // Zmieniamy stan na 'opened'
            })
            .catch(() => alert('Błąd przy otwieraniu grippera.'));
    } else {
        // Jeśli gripper jest otwarty, zamykamy go
        postRequest('/close_gripper')
            .then(response => response.text())
            .then(msg => {
                alert(msg);
                gripperState = 'closed'; // Zmieniamy stan na 'closed'
            })
            .catch(() => alert('Błąd przy zamykaniu grippera.'));
    }
};

// Helper function for POST requests
const postRequest = (url, body = null) => {
    const options = {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
    };
    if (body) options.body = JSON.stringify(body);
    return fetch(url, options);
};

// Servo control functions
const moveServo = (id, angle) => {
    document.getElementById(`angle${id}`).innerText = angle;
    postRequest('/move_servo', { id, angle });
};

const movePreset = (presetName) => {
    postRequest(`/move_preset/${presetName}`)
        .then(response => {
            if (response.ok) {
                updateSliders();
            } else {
                console.error('Preset error!');
            }
        });
};

// Slider update function
const updateSliders = () => {
    fetch('/get_angles')
        .then(response => response.json())
        .then(data => {
            Object.entries(data).forEach(([id, angle]) => {
                const slider = document.getElementById(`slider${id}`);
                const angleDisplay = document.getElementById(`angle${id}`);
                if (slider) slider.value = angle;
                if (angleDisplay) angleDisplay.innerText = angle;
            });
        });
};

// Controller functions
const startPad = () => postRequest('/start_pad');
const stopPad = () => {
    postRequest('/stop_pad')
        .then(response => {
            if (response.ok) {
                updateSliders();  // ← odśwież suwaki
                alert('Sterowanie padem wyłączone.');
            } else {
                alert('Błąd podczas wyłączania pada.');
            }
        })
        .catch(() => alert('Błąd połączenia z serwerem.'));
};

const savePosition = () => {
    postRequest('/save_position')
        .then(response => response.text())
        .then(alert)
        .catch(() => alert('Błąd przy zapisie.'));
};

const resetPositions = () => {
    postRequest('/reset_positions')
        .then(response => response.text())
        .then(alert)
        .catch(() => alert('Błąd przy usuwaniu.'));
};

const playPositions = () => {
    postRequest('/play_positions')
        .then(response => response.text())
        .then(msg => {
            alert(msg);
            waitForPlayDone();  // start sprawdzania
        })
        .catch(() => alert('Błąd przy odtwarzaniu.'));
};

function waitForPlayDone() {
    const interval = setInterval(() => {
        fetch('/play_status')
            .then(res => res.text())
            .then(text => {
                if (text === 'done') {
                    clearInterval(interval);
                    updateSliders();  // 🟢 Odśwież suwaki
                    alert('Sekwencja zakończona');
                }
            });
    }, 1000); // sprawdzaj co sekundę
}

const stopPositions = () => {
    postRequest('/stop_positions')
        .then(response => response.text())
        .then(msg => {
            alert(msg);
            setTimeout(updateSliders, 2000); 
        })
        .catch(() => alert('Błąd przy zatrzymywaniu sekwencji.'));
};

const torqueOn = () => {
    postRequest('/torque_on')
        .then(response => response.text())
        .then(alert)
        .catch(() => alert('Błąd przy włączaniu torque'));
};

const torqueOff = () => {
    postRequest('/torque_off')
        .then(response => response.text())
        .then(alert)
        .catch(() => alert('Błąd przy wyłączaniu torque'));
};

// Initialize sliders on page load
window.onload = () => {
    fetch('/get_angles')
        .then(response => response.json())
        .then(data => {
            Object.entries(data).forEach(([id, angle]) => {
                const slider = document.getElementById(`slider${id}`);
                const angleDisplay = document.getElementById(`angle${id}`);
                if (slider) slider.value = angle;
                if (angleDisplay) angleDisplay.innerText = angle;

                // Add event listener for slider input
                if (slider) {
                    slider.oninput = () => moveServo(id, slider.value);
                }
            });
        });
};
