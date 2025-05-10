let gripperState = 'closed';

const postRequest = (url, body = null) => {
    const options = {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
    };
    if (body) options.body = JSON.stringify(body);
    return fetch(url, options);
};

const setStatus = msg => {
    const el = document.getElementById('status');
    if (el) el.innerText = msg;
};

const toggleGripper = () => {
    const action = gripperState === 'closed' ? 'open_gripper' : 'close_gripper';
    const newState = gripperState === 'closed' ? 'opened' : 'closed';

    postRequest(`/${action}`)
        .then(response => response.text())
        .then(msg => {
            alert(msg);
            gripperState = newState;
        })
        .catch(() => alert(`Błąd przy ${action === 'open_gripper' ? 'otwieraniu' : 'zamykaniu'} grippera.`));
};

const moveServo = (id, angle) => {
    document.getElementById(`angle${id}`).innerText = angle;
    postRequest('/move_servo', { id, angle });
};

const goHome = () => {
    setStatus("Przesuwanie do pozycji domowej...");
    postRequest('/home_position')
        .then(r => r.text())
        .then(seconds => {
            const ms = parseFloat(seconds) * 1000;
            setTimeout(() => {
                updateSliders();
                setStatus("Pozycja domowa ustawiona.");
            }, ms + 200);
        });
};

const movePreset = presetName => {
    postRequest(`/move_preset/${presetName}`)
        .then(response => {
            if (response.ok) updateSliders();
            else console.error('Preset error!');
        });
};

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

const startPad = () => postRequest('/start_pad');

const stopPad = () => {
    postRequest('/stop_pad')
        .then(response => {
            if (response.ok) {
                updateSliders();
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
            waitForPlayDone();
        })
        .catch(() => alert('Błąd przy odtwarzaniu.'));
};

const waitForPlayDone = () => {
    const interval = setInterval(() => {
        fetch('/play_status')
            .then(res => res.text())
            .then(text => {
                if (text === 'done') {
                    clearInterval(interval);
                    updateSliders();
                    alert('Sekwencja zakończona');
                }
            });
    }, 1000);
};

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

const enableManualControl = () => {
    postRequest('/start_manual')
        .then(r => r.text())
        .then(seconds => {
            const ms = parseFloat(seconds) * 1000;

            [1, 2, 3, 4].forEach(id => {
                const slider = document.getElementById(`slider${id}`);
                if (slider) slider.disabled = false;
            });

            setStatus("Sterowanie ręczne aktywne");

            setTimeout(() => {
                updateSliders();
                setStatus("Pozycja startowa gotowa – steruj ręcznie");
            }, ms + 200);
        })
        .catch(() => alert("Błąd podczas uruchamiania sterowania ręcznego."));
};

const moveToPoint = () => {
    const x = parseFloat(document.getElementById('inputX').value);
    const y = parseFloat(document.getElementById('inputY').value);
    const z = parseFloat(document.getElementById('inputZ').value);

    if (isNaN(x) || isNaN(y) || isNaN(z)) {
        alert("Wprowadź poprawne wartości X, Y, Z");
        return;
    }

    postRequest('/move_to_point', { x, y, z })
        .then(response => response.text())
        .then(alert)
        .updateSliders()
        .catch(() => alert("Błąd podczas ruchu do punktu."));
};


window.onload = () => {
    fetch('/get_angles')
        .then(response => response.json())
        .then(data => {
            Object.entries(data).forEach(([id, angle]) => {
                const slider = document.getElementById(`slider${id}`);
                const angleDisplay = document.getElementById(`angle${id}`);
                if (slider) slider.value = angle;
                if (angleDisplay) angleDisplay.innerText = angle;

                if (slider) {
                    slider.oninput = () => moveServo(id, slider.value);
                }
            });
        });
};
